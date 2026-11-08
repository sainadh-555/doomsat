"""FastAPI backend and WebSocket telemetry broadcaster for DOOMSat Mission Control."""

from typing import Dict, List, Optional, Any
import os
import json
import asyncio
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from doomsat.core.schemas import TelemetryBeacon
from doomsat.ground_station.receiver import GroundStationReceiver
from doomsat.ground_station.telemetry_store import TelemetryStore
from doomsat.ground_station.image_reconstruction import GroundStationImageManager
from doomsat.dashboard.visualizers import (
    build_orbit_3d_figure,
    build_ground_track_2d_figure,
    build_attitude_cube_figure,
    build_eps_battery_gauge,
    build_thermal_bar_chart,
)

app = FastAPI(
    title="DOOMSat-1U Mission Control Digital Twin",
    description="Real-time telemetry, 3D visualization, and command uplink API",
    version="0.1.0",
)

# Enable CORS for local dashboards
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Shared Ground Station State
telemetry_store = TelemetryStore()
image_manager = GroundStationImageManager()
ground_station_receiver = GroundStationReceiver(
    telemetry_store=telemetry_store,
    image_manager=image_manager,
)


class CommandRequest(BaseModel):
    """Schema for command uplink request."""
    command: str
    target_node: int = 3  # OBC by default


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


@app.get("/", response_class=HTMLResponse)
async def get_dashboard_html() -> str:
    """Serve the single-page aerospace Mission Control web UI."""
    template_path = Path(__file__).parent / "templates" / "index.html"
    if template_path.exists():
        return template_path.read_text(encoding="utf-8")
    return "<h1>DOOMSat-1U Mission Control Dashboard</h1><p>Template not found.</p>"


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
    """Return latest housekeeping telemetry beacon."""
    beacon = telemetry_store.get_latest()
    if beacon is None:
        # Synthesize nominal beacon if receiver has not yet ingested packets
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
    return beacon.model_dump()


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
    if not req.command.strip():
        raise HTTPException(status_code=400, detail="Command string cannot be empty")

    success = ground_station_receiver.send_command(
        command_str=req.command,
        target_node=req.target_node,
    )
    return {
        "status": "TRANSMITTED" if success else "ENQUEUED",
        "command": req.command,
        "target_node": req.target_node,
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

