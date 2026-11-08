"""FastAPI backend and WebSocket telemetry broadcaster for GARUDA-1U Mission Control."""

from contextlib import asynccontextmanager
from typing import Dict, List, Optional, Any
import os
import json
import math
import base64
import asyncio
from pathlib import Path

import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Response
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from doomsat.core.schemas import TelemetryBeacon
from doomsat.ground_station.receiver import GroundStationReceiver
from doomsat.ground_station.telemetry_store import TelemetryStore
from doomsat.ground_station.image_reconstruction import GroundStationImageManager, ReconstructedImageRecord
from doomsat.sim.orchestrator import MissionOrchestrator, SimulationConfig
from doomsat.payload.camera import OV5647Camera
from doomsat.payload.compression import compress_jpeg
from doomsat.obc.fsm import FlightMode
from doomsat.dashboard.visualizers import (
    build_orbit_3d_figure,
    build_ground_track_2d_figure,
    build_attitude_cube_figure,
    build_eps_battery_gauge,
    build_thermal_bar_chart,
)

# Shared Ground Station & Live Simulation State
telemetry_store = TelemetryStore()
image_manager = GroundStationImageManager()
ground_station_receiver = GroundStationReceiver(
    telemetry_store=telemetry_store,
    image_manager=image_manager,
)
live_orchestrator = MissionOrchestrator(SimulationConfig(duration_s=86400.0, dt_s=1.0))
time_warp_factor: int = 5
latest_telemetry_dict: Dict[str, Any] = {}


def generate_orbital_photo(image_id: int, lat_deg: float = 17.385, lon_deg: float = 78.487) -> ReconstructedImageRecord:
    """Generate a procedural Earth observation photo captured from GARUDA-1U."""
    cam = OV5647Camera()
    scene = cam.capture_scene(
        latitude_rad=math.radians(lat_deg),
        longitude_rad=math.radians(lon_deg),
        solar_zenith_angle_rad=0.35,
        in_eclipse=False,
        render_width=320,
        render_height=240,
    )
    jpeg_res = compress_jpeg(scene, quality=80)
    jpeg_bytes = jpeg_res.data
    b64_thumb = base64.b64encode(jpeg_bytes).decode("ascii")
    return ReconstructedImageRecord(
        image_id=image_id,
        width=320,
        height=240,
        size_bytes=len(jpeg_bytes),
        total_chunks=max(1, len(jpeg_bytes) // 160),
        capture_time_s=120.0 + image_id * 30.0,
        gsd_m=194.4,
        latitude_rad=math.radians(lat_deg),
        longitude_rad=math.radians(lon_deg),
        raw_jpeg_bytes=jpeg_bytes,
        base64_thumbnail=b64_thumb,
        retransmission_requests=0,
    )


# Preload initial Earth observation photos into Ground Station gallery
image_manager._completed_images[1] = generate_orbital_photo(image_id=1, lat_deg=17.385, lon_deg=78.487)
image_manager._completed_images[2] = generate_orbital_photo(image_id=2, lat_deg=28.613, lon_deg=77.209)


async def run_digital_twin_loop() -> None:
    """Continuously advance digital twin simulation clock and broadcast telemetry."""
    global latest_telemetry_dict, time_warp_factor
    try:
        while True:
            await asyncio.sleep(1.0)
            # Advance simulation clock by time_warp_factor steps per real-time second
            snap = None
            for _ in range(max(1, time_warp_factor)):
                snap = live_orchestrator.step()

            if snap is None:
                continue

            beacon = TelemetryBeacon(
                sequence_id=live_orchestrator.beacon_seq_counter,
                timestamp_s=snap.time_s,
                flight_mode=snap.flight_mode,
                battery_soc=snap.battery_soc_pct,
                battery_voltage=snap.battery_voltage_v,
                solar_power=snap.solar_power_w,
                obc_temp_k=snap.temp_electronics_c + 273.15,
                battery_temp_k=snap.temp_battery_c + 273.15,
                angular_rate_norm_radps=math.radians(snap.omega_norm_deg_s),
                in_eclipse=snap.in_eclipse,
                rssi_dbm=-110.5 if snap.gs_visible else -126.0,
                snr_db=7.8 if snap.gs_visible else -5.0,
            )
            telemetry_store.add_beacon(beacon)
            latest_telemetry_dict = {
                **beacon.model_dump(),
                "altitude_km": snap.altitude_km,
                "latitude_deg": snap.latitude_deg,
                "longitude_deg": snap.longitude_deg,
                "velocity_km_s": float(np.linalg.norm(snap.v_eci_km_s)),
                "system_load_w": snap.system_load_w,
                "doppler_khz": snap.gs_doppler_khz,
                "omega_norm_deg_s": snap.omega_norm_deg_s,
                "r_eci_km": [float(c) for c in snap.r_eci_km],
                "gs_visible": snap.gs_visible,
                "gs_elevation_deg": snap.gs_elevation_deg,
                "warp_factor": time_warp_factor,
            }
            await connection_manager.broadcast(latest_telemetry_dict)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f"[!] Digital twin simulation loop error: {e}")


@asynccontextmanager
async def lifespan(app: FastAPI):
    task = asyncio.create_task(run_digital_twin_loop())
    yield
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


app = FastAPI(
    title="GARUDA-1U Mission Control Digital Twin",
    description="Real-time telemetry, 3D visualization, and command uplink API",
    version="0.1.0",
    lifespan=lifespan,
)

# Enable CORS for local dashboards
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class CommandRequest(BaseModel):
    """Schema for command uplink request."""
    command: str
    target_node: int = 3  # OBC by default


class WarpRequest(BaseModel):
    """Schema for simulation time warp acceleration factor."""
    warp: int


class ConnectionManager:
    """Manages active WebSocket telemetry streaming clients."""

    def __init__(self) -> None:
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]) -> None:
        dead_connections = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception:
                dead_connections.append(connection)
        for dc in dead_connections:
            self.disconnect(dc)


connection_manager = ConnectionManager()


@app.api_route("/favicon.ico", methods=["GET", "HEAD"], include_in_schema=False)
async def favicon() -> Response:
    return Response(status_code=204)


@app.get("/", response_class=HTMLResponse)
async def get_dashboard_html() -> str:
    """Serve the single-page aerospace Mission Control web UI."""
    template_path = Path(__file__).parent / "templates" / "index.html"
    if template_path.exists():
        return template_path.read_text(encoding="utf-8")
    return "<h1>GARUDA-1U Mission Control Dashboard</h1><p>Template not found.</p>"


@app.get("/api/status")
async def get_system_status() -> Dict[str, Any]:
    """Retrieve operational health status of satellite digital twin and ground segment."""
    latest_beacon = telemetry_store.get_latest()
    metrics = ground_station_receiver.metrics
    return {
        "status": "OPERATIONAL",
        "station_location": "Hyderabad, India (17.3850 N, 78.4867 E)",
        "rf_frequency_hz": 436_500_000,
        "metrics": {
            "total_packets": metrics.total_packets_received,
            "valid_packets": metrics.valid_packets,
            "crc_errors": metrics.crc_errors,
            "pdr_pct": round(metrics.packet_delivery_ratio, 2),
            "beacons": metrics.beacons_received,
            "image_chunks": metrics.image_chunks_received,
            "last_rssi_dbm": metrics.last_rssi_dbm,
            "last_snr_db": metrics.last_snr_db,
        },
        "latest_beacon": latest_beacon.model_dump() if latest_beacon else None,
    }


@app.get("/api/telemetry/latest")
async def get_latest_telemetry() -> Dict[str, Any]:
    """Return latest housekeeping telemetry beacon with live orbital coordinates."""
    if latest_telemetry_dict:
        return latest_telemetry_dict

    beacon = telemetry_store.get_latest()
    if beacon is None:
        beacon = TelemetryBeacon(
            sequence_id=0,
            timestamp_s=0.0,
            flight_mode="NOMINAL",
            battery_soc=95.0,
            battery_voltage=3.95,
            solar_power=1.85,
            obc_temp_k=294.15,
            battery_temp_k=292.15,
            angular_rate_norm_radps=0.015,
            in_eclipse=False,
            rssi_dbm=-112.5,
            snr_db=6.4,
        )
    return {
        **beacon.model_dump(),
        "altitude_km": 500.0,
        "latitude_deg": 17.385,
        "longitude_deg": 78.487,
        "velocity_km_s": 7.612,
        "system_load_w": 0.75,
        "doppler_khz": 0.0,
        "omega_norm_deg_s": 0.86,
        "r_eci_km": [6878.0, 0.0, 0.0],
        "gs_visible": True,
        "gs_elevation_deg": 45.0,
        "warp_factor": time_warp_factor,
    }


@app.post("/api/sim/warp")
async def set_time_warp(req: WarpRequest) -> Dict[str, Any]:
    """Dynamically set the simulation acceleration / time warp factor."""
    global time_warp_factor
    time_warp_factor = max(1, min(120, req.warp))
    if latest_telemetry_dict:
        latest_telemetry_dict["warp_factor"] = time_warp_factor
    return {"status": "OK", "warp_factor": time_warp_factor}


@app.get("/api/telemetry/history")
async def get_telemetry_history(window_s: Optional[float] = None) -> List[Dict[str, Any]]:
    """Return historical telemetry records."""
    beacons = telemetry_store.get_history(window_seconds=window_s)
    return [b.model_dump() for b in beacons]


@app.get("/api/ground_station/metrics")
async def get_ground_station_metrics() -> Dict[str, Any]:
    """Return detailed RF link and ground station reception metrics."""
    m = ground_station_receiver.metrics
    return {
        "total_packets_received": m.total_packets_received,
        "valid_packets": m.valid_packets,
        "crc_errors": m.crc_errors,
        "pdr_pct": m.packet_delivery_ratio,
        "beacons_received": m.beacons_received,
        "image_chunks_received": m.image_chunks_received,
        "commands_uplinked": m.commands_uplinked,
        "last_rssi_dbm": m.last_rssi_dbm,
        "last_snr_db": m.last_snr_db,
        "mean_snr_db": round(m.mean_snr_db, 2),
    }


@app.get("/api/ground_station/images")
async def get_reconstructed_images() -> List[Dict[str, Any]]:
    """Return list of completed reconstructed images with thumbnails."""
    images = image_manager.list_completed_images()
    return [
        {
            "image_id": img.image_id,
            "width": img.width,
            "height": img.height,
            "size_bytes": img.size_bytes,
            "total_chunks": img.total_chunks,
            "capture_time_s": img.capture_time_s,
            "gsd_m": img.gsd_m,
            "thumbnail_b64": img.base64_thumbnail,
        }
        for img in images
    ]


@app.post("/api/command/uplink")
async def uplink_command(req: CommandRequest) -> Dict[str, Any]:
    """Uplink a flight command to the spacecraft."""
    cmd_clean = req.command.strip().upper()
    if not cmd_clean:
        raise HTTPException(status_code=400, detail="Command string cannot be empty")

    success = ground_station_receiver.send_command(
        command_str=req.command,
        target_node=req.target_node,
    )

    # Execute interactive response in digital twin
    reply_msg = "ACK: Command dispatched to OBC"
    if "CAPTURE" in cmd_clean or "IMAGE" in cmd_clean:
        next_id = len(image_manager._completed_images) + 1
        lat = live_orchestrator.last_look_angles.elevation_deg if live_orchestrator.last_look_angles else 17.385
        new_img = generate_orbital_photo(image_id=next_id, lat_deg=lat, lon_deg=78.487)
        image_manager._completed_images[next_id] = new_img
        reply_msg = f"ACK: Payload image #{next_id} captured (320x240 JPEG, {new_img.size_bytes}B). Downlink ready."
    elif "PING" in cmd_clean:
        reply_msg = "ACK: PONG [RTT: 26ms] Node 3 (RPi Zero 2W OBC) | System Status: ALL SYSTEMS NOMINAL"
    elif "DETUMBLE" in cmd_clean:
        live_orchestrator.obc.fsm.current_mode = FlightMode.DETUMBLE
        if latest_telemetry_dict:
            latest_telemetry_dict["flight_mode"] = "DETUMBLE"
        reply_msg = "ACK: ADCS B-dot detumbling mode engaged. Magnetorquers active."
    elif "NOMINAL" in cmd_clean:
        live_orchestrator.obc.fsm.current_mode = FlightMode.NOMINAL
        if latest_telemetry_dict:
            latest_telemetry_dict["flight_mode"] = "NOMINAL"
        reply_msg = "ACK: Spacecraft transitioned to NOMINAL operations."
    elif "SAFE_HOLD" in cmd_clean:
        live_orchestrator.obc.fsm.current_mode = FlightMode.SAFE_HOLD
        if latest_telemetry_dict:
            latest_telemetry_dict["flight_mode"] = "SAFE_HOLD"
        reply_msg = "ACK: Spacecraft placed in SAFE_HOLD mode."

    return {
        "status": "TRANSMITTED" if success else "ENQUEUED",
        "command": req.command,
        "target_node": req.target_node,
        "reply_message": reply_msg,
    }


# Plotly Visualizer Endpoints
@app.get("/api/visualizations/orbit_3d")
async def get_visualization_orbit_3d() -> Dict[str, Any]:
    """Pre-rendered Plotly 3D Orbit Globe figure."""
    # Default nominal circular orbit trail
    trail = []
    r_mag = 6878.0  # km
    for a in range(0, 360, 10):
        rad = math.radians(a)
        trail.append((r_mag * math.cos(rad), r_mag * math.sin(rad) * math.cos(math.radians(51.6)), r_mag * math.sin(rad) * math.sin(math.radians(51.6))))

    fig_dict = build_orbit_3d_figure(
        r_sat_eci_km=(r_mag, 0.0, 0.0),
        trajectory_points_km=trail,
        sun_unit_eci=(1.0, 0.0, 0.0),
        gs_eci_km=(6090.0, 0.0, 1910.0),
    )
    return fig_dict


@app.get("/api/visualizations/ground_track")
async def get_visualization_ground_track() -> Dict[str, Any]:
    """Pre-rendered Plotly 2D Ground Track figure."""
    lats = [float(np.sin(math.radians(deg)) * 51.6) for deg in range(-180, 180, 5)]
    lons = [float(deg) for deg in range(-180, 180, 5)]
    fig_dict = build_ground_track_2d_figure(
        sub_lat_deg=17.385,
        sub_lon_deg=78.487,
        track_lats=lats,
        track_lons=lons,
    )
    return fig_dict


@app.get("/api/visualizations/attitude")
async def get_visualization_attitude() -> Dict[str, Any]:
    """Pre-rendered Plotly 3D Attitude Cube figure."""
    fig_dict = build_attitude_cube_figure(
        q_scalar_first=(1.0, 0.0, 0.0, 0.0),
        omega_radps=(0.01, -0.02, 0.015),
        sun_body_unit=(0.7071, 0.0, 0.7071),
    )
    return fig_dict


@app.get("/api/visualizations/eps")
async def get_visualization_eps() -> Dict[str, Any]:
    """Pre-rendered Plotly EPS Battery Gauge figure."""
    beacon = telemetry_store.get_latest()
    soc = beacon.battery_soc if beacon else 90.0
    v = beacon.battery_voltage if beacon else 3.85
    solar = beacon.solar_power if beacon else 1.80
    return build_eps_battery_gauge(soc_pct=soc, voltage_v=v, solar_power_w=solar, system_load_w=0.75)


@app.get("/api/visualizations/thermal")
async def get_visualization_thermal() -> Dict[str, Any]:
    """Pre-rendered Plotly Thermal Multi-Node Bar figure."""
    beacon = telemetry_store.get_latest()
    obc_t = beacon.obc_temp_k if beacon else 295.0
    bat_t = beacon.battery_temp_k if beacon else 293.0
    thermal_data = {
        "temp_face_pos_x_k": 298.0,
        "temp_face_neg_x_k": 275.0,
        "temp_face_pos_y_k": 285.0,
        "temp_face_neg_y_k": 280.0,
        "temp_face_pos_z_k": 290.0,
        "temp_face_neg_z_k": 282.0,
        "temp_electronics_k": obc_t,
        "temp_battery_k": bat_t,
    }
    return build_thermal_bar_chart(thermal_data)


@app.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket) -> None:
    """High-frequency WebSocket telemetry streaming channel."""
    await connection_manager.connect(websocket)
    try:
        # Push initial snapshot upon connection
        latest = await get_latest_telemetry()
        await websocket.send_json(latest)

        while True:
            # Keep socket open and receive client pings/messages
            msg = await websocket.receive_text()
            if msg == "ping":
                await websocket.send_text("pong")
    except WebSocketDisconnect:
        connection_manager.disconnect(websocket)
    except Exception:
        connection_manager.disconnect(websocket)

