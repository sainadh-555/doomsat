"""Tests for Mission Control Dashboard: FastAPI endpoints, WebSockets, and Plotly visualizers."""

import json
import pytest
from fastapi.testclient import TestClient

from doomsat.dashboard.app import app
from doomsat.dashboard.visualizers import (
    build_orbit_3d_figure,
    build_ground_track_2d_figure,
    build_attitude_cube_figure,
    build_eps_battery_gauge,
    build_thermal_bar_chart,
)


@pytest.fixture
def client():
    """FastAPI test client fixture."""
    return TestClient(app)


def test_dashboard_rest_endpoints(client):
    """Verify all REST API endpoints return 200 OK and valid schemas."""
    # 1. Dashboard UI HTML
    res_html = client.get("/")
    assert res_html.status_code == 200
    assert "DOOMSat-1U" in res_html.text

    # 2. System Status
    res_status = client.get("/api/status")
    assert res_status.status_code == 200
    data_status = res_status.json()
    assert data_status["status"] == "OPERATIONAL"
    assert "Hyderabad" in data_status["station_location"]
    assert "metrics" in data_status

    # 3. Latest Telemetry
    res_tel = client.get("/api/telemetry/latest")
    assert res_tel.status_code == 200
    data_tel = res_tel.json()
    assert "flight_mode" in data_tel
    assert "battery_soc" in data_tel

    # 4. Telemetry History
    res_hist = client.get("/api/telemetry/history?window_s=3600")
    assert res_hist.status_code == 200
    assert isinstance(res_hist.json(), list)

    # 5. Ground Station Metrics
    res_m = client.get("/api/ground_station/metrics")
    assert res_m.status_code == 200
    data_m = res_m.json()
    assert "total_packets_received" in data_m
    assert "pdr_pct" in data_m

    # 6. Reconstructed Images List
    res_img = client.get("/api/ground_station/images")
    assert res_img.status_code == 200
    assert isinstance(res_img.json(), list)


def test_dashboard_command_uplink(client):
    """Verify command uplink validation and dispatch."""
    # Valid command
    res_valid = client.post(
        "/api/command/uplink",
        json={"command": "CAPTURE_IMAGE", "target_node": 3},
    )
    assert res_valid.status_code == 200
    data = res_valid.json()
    assert data["command"] == "CAPTURE_IMAGE"
    assert data["status"] in ("TRANSMITTED", "ENQUEUED")

    # Empty command (should return HTTP 400 Bad Request)
    res_invalid = client.post(
        "/api/command/uplink",
        json={"command": "   ", "target_node": 3},
    )
    assert res_invalid.status_code == 400


def test_visualizer_orbit_3d_globe():
    """Verify 3D Orbit Globe Plotly figure generation."""
    fig_dict = build_orbit_3d_figure(
        r_sat_eci_km=(6878.0, 0.0, 0.0),
        trajectory_points_km=[(6878.0, 0.0, 0.0), (0.0, 6878.0, 0.0)],
        sun_unit_eci=(1.0, 0.0, 0.0),
        gs_eci_km=(6090.0, 0.0, 1910.0),
    )
    assert "data" in fig_dict
    assert "layout" in fig_dict
    # Check trace types: Surface (Earth), Scatter3d (satellite, orbit, sun, GS)
    trace_types = [t["type"] for t in fig_dict["data"]]
    assert "surface" in trace_types
    assert "scatter3d" in trace_types


def test_visualizer_ground_track_2d():
    """Verify 2D Ground Track Plotly figure generation."""
    fig_dict = build_ground_track_2d_figure(
        sub_lat_deg=17.385,
        sub_lon_deg=78.487,
        track_lats=[0.0, 10.0, 20.0],
        track_lons=[70.0, 75.0, 80.0],
    )
    assert "data" in fig_dict
    assert "layout" in fig_dict
    assert fig_dict["layout"]["xaxis"]["title"]["text"] == "Longitude [deg]"
    assert fig_dict["layout"]["yaxis"]["title"]["text"] == "Latitude [deg]"


def test_visualizer_attitude_cube():
    """Verify 3D Attitude Cube Plotly figure generation."""
    fig_dict = build_attitude_cube_figure(
        q_scalar_first=(1.0, 0.0, 0.0, 0.0),
        omega_radps=(0.02, -0.01, 0.05),
        sun_body_unit=(1.0, 0.0, 0.0),
    )
    assert "data" in fig_dict
    assert "layout" in fig_dict
    trace_types = [t["type"] for t in fig_dict["data"]]
    assert "mesh3d" in trace_types


def test_visualizer_eps_and_thermal():
    """Verify EPS battery gauge and thermal bar chart figures."""
    fig_eps = build_eps_battery_gauge(soc_pct=85.0, voltage_v=3.82, solar_power_w=1.9, system_load_w=0.75)
    assert "data" in fig_eps
    assert fig_eps["data"][0]["type"] == "indicator"
    assert fig_eps["data"][0]["value"] == 85.0

    fig_thm = build_thermal_bar_chart({
        "temp_face_pos_x_k": 298.0,
        "temp_face_neg_x_k": 275.0,
        "temp_face_pos_y_k": 285.0,
        "temp_face_neg_y_k": 280.0,
        "temp_face_pos_z_k": 290.0,
        "temp_face_neg_z_k": 282.0,
        "temp_electronics_k": 295.0,
        "temp_battery_k": 293.0,
    })
    assert "data" in fig_thm
    assert fig_thm["data"][0]["type"] == "bar"
    assert len(fig_thm["data"][0]["x"]) == 8


def test_dashboard_websocket_streaming(client):
    """Verify WebSocket client connection, initial telemetry frame, and ping-pong."""
    with client.websocket_connect("/ws/telemetry") as ws:
        # Receive initial telemetry snapshot
        init_frame = ws.receive_json()
        assert "flight_mode" in init_frame
        assert "battery_soc" in init_frame

        # Send ping, receive pong
        ws.send_text("ping")
        resp = ws.receive_text()
        assert resp == "pong"

