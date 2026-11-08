"""Interactive Plotly visualizers for DOOMSat Mission Control Dashboard."""

from typing import Dict, List, Optional, Tuple, Any
import math
import numpy as np
import plotly.graph_objects as go

from doomsat.core.constants import EARTH_RADIUS_EQUATORIAL
from doomsat.core.coordinates import quaternion_to_dcm


def build_orbit_3d_figure(
    r_sat_eci_km: Tuple[float, float, float] = (6878.0, 0.0, 0.0),
    trajectory_points_km: Optional[List[Tuple[float, float, float]]] = None,
    sun_unit_eci: Optional[Tuple[float, float, float]] = None,
    gs_eci_km: Optional[Tuple[float, float, float]] = None,
) -> Dict[str, Any]:
    """Generate interactive 3D Orbit Globe Plotly figure.

    Args:
        r_sat_eci_km: Current satellite position [x, y, z] in ECI [km].
        trajectory_points_km: List of orbital trajectory coordinates [km].
        sun_unit_eci: Unit vector pointing to Sun in ECI.
        gs_eci_km: Ground station position in ECI [km].

    Returns:
        Plotly figure dictionary suitable for JSON serialization and frontend rendering.
    """
    fig = go.Figure()

    # 1. Earth 3D Sphere (R = 6378.137 km)
    r_earth = EARTH_RADIUS_EQUATORIAL / 1000.0  # km
    phi = np.linspace(0, 2 * np.pi, 30)
    theta = np.linspace(-np.pi / 2, np.pi / 2, 20)
    phi_grid, theta_grid = np.meshgrid(phi, theta)

    x_earth = r_earth * np.cos(theta_grid) * np.cos(phi_grid)
    y_earth = r_earth * np.cos(theta_grid) * np.sin(phi_grid)
    z_earth = r_earth * np.sin(theta_grid)

    fig.add_trace(
        go.Surface(
            x=x_earth,
            y=y_earth,
            z=z_earth,
            colorscale=[[0.0, "#0b2545"], [0.5, "#134074"], [1.0, "#1d2d44"]],
            showscale=False,
            opacity=0.75,
            name="Earth",
            hoverinfo="skip",
        )
    )

    # 2. Orbital Trajectory Trail
    if trajectory_points_km and len(trajectory_points_km) > 1:
        traj_arr = np.array(trajectory_points_km)
        fig.add_trace(
            go.Scatter3d(
                x=traj_arr[:, 0],
                y=traj_arr[:, 1],
                z=traj_arr[:, 2],
                mode="lines",
                line=dict(color="#00d4ff", width=4),
                name="500km LEO Orbit",
            )
        )

    # 3. Current Satellite Marker
    fig.add_trace(
        go.Scatter3d(
            x=[r_sat_eci_km[0]],
            y=[r_sat_eci_km[1]],
            z=[r_sat_eci_km[2]],
            mode="markers+text",
            marker=dict(size=8, color="#ff0055", symbol="diamond"),
            text=["GARUDA-1U"],
            textposition="top center",
            name="GARUDA-1U",
        )
    )

    # 4. Sun Direction Beam
    if sun_unit_eci is not None:
        sun_scale = r_earth * 1.8
        sun_x = [0.0, sun_unit_eci[0] * sun_scale]
        sun_y = [0.0, sun_unit_eci[1] * sun_scale]
        sun_z = [0.0, sun_unit_eci[2] * sun_scale]
        fig.add_trace(
            go.Scatter3d(
                x=sun_x,
                y=sun_y,
                z=sun_z,
                mode="lines+markers",
                line=dict(color="#ffdd00", width=6, dash="solid"),
                marker=dict(size=[0, 10], color="#ffdd00"),
                name="Solar Vector",
            )
        )

    # 5. Ground Station Location (Hyderabad)
    if gs_eci_km is not None:
        fig.add_trace(
            go.Scatter3d(
                x=[gs_eci_km[0]],
                y=[gs_eci_km[1]],
                z=[gs_eci_km[2]],
                mode="markers+text",
                marker=dict(size=6, color="#00ff88", symbol="circle"),
                text=["Hyderabad GS"],
                textposition="bottom right",
                name="Ground Station",
            )
        )

    max_range = r_earth * 1.5
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#121824",
        plot_bgcolor="#0a0d14",
        font=dict(color="#e2e8f0"),
        title=dict(text="3D Orbital Trajectory & Earth Sphere (ECI)", font=dict(color="#00d4ff")),
        margin=dict(l=0, r=0, b=0, t=40),
        scene=dict(
            bgcolor="#0a0d14",
            xaxis=dict(range=[-max_range, max_range], title="X [km]", color="#94a3b8", gridcolor="#1e293b"),
            yaxis=dict(range=[-max_range, max_range], title="Y [km]", color="#94a3b8", gridcolor="#1e293b"),
            zaxis=dict(range=[-max_range, max_range], title="Z [km]", color="#94a3b8", gridcolor="#1e293b"),
            aspectmode="cube",
        ),
        legend=dict(x=0.02, y=0.98, font=dict(color="#e2e8f0")),
    )

    return fig.to_dict()


def build_ground_track_2d_figure(
    sub_lat_deg: float = 17.385,
    sub_lon_deg: float = 78.487,
    track_lats: Optional[List[float]] = None,
    track_lons: Optional[List[float]] = None,
    gs_lat_deg: float = 17.3850,
    gs_lon_deg: float = 78.4867,
) -> Dict[str, Any]:
    """Generate 2D Ground Track Map with Ground Station visibility mask.

    Args:
        sub_lat_deg: Sub-satellite geodetic latitude [deg].
        sub_lon_deg: Sub-satellite geodetic longitude [deg].
        track_lats: Orbital ground track latitudes [deg].
        track_lons: Orbital ground track longitudes [deg].
        gs_lat_deg: Ground station latitude [deg].
        gs_lon_deg: Ground station longitude [deg].

    Returns:
        Plotly figure dictionary.
    """
    fig = go.Figure()

    # 1. Ground Track Path
    if track_lats and track_lons:
        fig.add_trace(
            go.Scatter(
                x=track_lons,
                y=track_lats,
                mode="lines",
                line=dict(color="#00d4ff", width=2),
                name="Ground Track",
            )
        )

    # 2. Ground Station Visibility Footprint Circle (10 deg elevation mask ~ 18 deg angular radius on globe)
    gs_radius_deg = 18.0
    angles = np.linspace(0, 2 * np.pi, 60)
    gs_circle_lons = gs_lon_deg + (gs_radius_deg / max(0.1, math.cos(math.radians(gs_lat_deg)))) * np.cos(angles)
    gs_circle_lats = gs_lat_deg + gs_radius_deg * np.sin(angles)

    fig.add_trace(
        go.Scatter(
            x=gs_circle_lons,
            y=gs_circle_lats,
            mode="lines",
            fill="toself",
            fillcolor="rgba(0, 255, 136, 0.15)",
            line=dict(color="#00ff88", width=1, dash="dash"),
            name="Hyderabad LOS Footprint",
            hoverinfo="skip",
        )
    )

    # 3. Ground Station Marker
    fig.add_trace(
        go.Scatter(
            x=[gs_lon_deg],
            y=[gs_lat_deg],
            mode="markers+text",
            marker=dict(size=10, color="#00ff88", symbol="triangle-up"),
            text=["Hyderabad"],
            textposition="top right",
            name="Ground Station",
        )
    )

    # 4. Current Sub-Satellite Point
    fig.add_trace(
        go.Scatter(
            x=[sub_lon_deg],
            y=[sub_lat_deg],
            mode="markers+text",
            marker=dict(size=12, color="#ff0055", symbol="circle-cross"),
            text=["GARUDA-1U"],
            textposition="bottom center",
            name="Sub-Satellite",
        )
    )

    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#121824",
        plot_bgcolor="#090d16",
        font=dict(color="#e2e8f0"),
        title=dict(text="2D Orbital Ground Track & Station Contact Mask", font=dict(color="#00d4ff")),
        xaxis=dict(title="Longitude [deg]", range=[-180, 180], dtick=30, color="#94a3b8", gridcolor="#1e293b"),
        yaxis=dict(title="Latitude [deg]", range=[-90, 90], dtick=30, color="#94a3b8", gridcolor="#1e293b"),
        margin=dict(l=40, r=20, b=40, t=40),
        legend=dict(x=0.02, y=0.05, orientation="h", font=dict(color="#e2e8f0")),
    )

    return fig.to_dict()


def build_attitude_cube_figure(
    q_scalar_first: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    omega_radps: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    sun_body_unit: Optional[Tuple[float, float, float]] = None,
) -> Dict[str, Any]:
    """Generate 3D Attitude Orientation Cube rotated by spacecraft quaternion.

    Args:
        q_scalar_first: Attitude quaternion [q0, q1, q2, q3].
        omega_radps: Spacecraft body angular velocity [rad/s].
        sun_body_unit: Sun unit vector in body frame.

    Returns:
        Plotly figure dictionary.
    """
    # 1U Box corners: [-50mm, +50mm]
    s = 0.05  # 50 mm half-length
    corners = np.array([
        [-s, -s, -s], [s, -s, -s], [s, s, -s], [-s, s, -s],
        [-s, -s,  s], [s, -s,  s], [s, s,  s], [-s, s,  s],
    ], dtype=np.float64)

    # Rotate corners by DCM (transposed to rotate body into visual world frame)
    dcm = quaternion_to_dcm(np.array(q_scalar_first))
    rotated_corners = corners @ dcm

    # Triangles defining the 6 faces
    i_idx = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2]
    j_idx = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3]
    k_idx = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6]

    fig = go.Figure()

    fig.add_trace(
        go.Mesh3d(
            x=rotated_corners[:, 0],
            y=rotated_corners[:, 1],
            z=rotated_corners[:, 2],
            i=i_idx,
            j=j_idx,
            k=k_idx,
            color="#2a3f5f",
            opacity=0.85,
            name="1U Chassis",
        )
    )

    # Angular velocity vector arrow
    w_norm = float(np.linalg.norm(omega_radps))
    if w_norm > 1e-4:
        scale = 0.08 / max(0.1, w_norm)
        fig.add_trace(
            go.Scatter3d(
                x=[0.0, omega_radps[0] * scale],
                y=[0.0, omega_radps[1] * scale],
                z=[0.0, omega_radps[2] * scale],
                mode="lines+markers",
                line=dict(color="#ff0055", width=5),
                marker=dict(size=[0, 6], color="#ff0055"),
                name=f"Omega ({w_norm:.2f} rad/s)",
            )
        )

    # Sun vector arrow
    if sun_body_unit is not None:
        fig.add_trace(
            go.Scatter3d(
                x=[0.0, sun_body_unit[0] * 0.09],
                y=[0.0, sun_body_unit[1] * 0.09],
                z=[0.0, sun_body_unit[2] * 0.09],
                mode="lines+markers",
                line=dict(color="#ffdd00", width=6),
                marker=dict(size=[0, 8], color="#ffdd00"),
                name="Sun Direction",
            )
        )

    lim = 0.10
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#121824",
        plot_bgcolor="#0a0d14",
        font=dict(color="#e2e8f0"),
        title=dict(text="3D Spacecraft Attitude Orientation", font=dict(color="#00d4ff")),
        margin=dict(l=0, r=0, b=0, t=40),
        scene=dict(
            bgcolor="#0a0d14",
            xaxis=dict(range=[-lim, lim], title="X", color="#94a3b8", gridcolor="#1e293b"),
            yaxis=dict(range=[-lim, lim], title="Y", color="#94a3b8", gridcolor="#1e293b"),
            zaxis=dict(range=[-lim, lim], title="Z", color="#94a3b8", gridcolor="#1e293b"),
            aspectmode="cube",
        ),
        legend=dict(x=0.02, y=0.98, font=dict(color="#e2e8f0")),
    )

    return fig.to_dict()


def build_eps_battery_gauge(
    soc_pct: float = 90.0,
    voltage_v: float = 3.85,
    solar_power_w: float = 1.80,
    system_load_w: float = 0.75,
) -> Dict[str, Any]:
    """Generate EPS Battery State of Charge Dial Indicator.

    Args:
        soc_pct: Battery State of Charge [%].
        voltage_v: Terminal voltage [V].
        solar_power_w: Solar generation [W].
        system_load_w: System load power [W].

    Returns:
        Plotly figure dictionary.
    """
    fig = go.Figure(
        go.Indicator(
            mode="gauge+number",
            value=round(soc_pct, 1),
            title=dict(text=f"Battery SoC (%)<br><span style='font-size:12px;color:#aaa'>{voltage_v:.2f}V | Net: {solar_power_w - system_load_w:+.2f}W</span>"),
            number=dict(suffix="%"),
            gauge=dict(
                axis=dict(range=[0, 100], tickwidth=1, tickcolor="white"),
                bar=dict(color="#00d4ff"),
                bgcolor="#1d2d44",
                borderwidth=2,
                bordercolor="#3e5c76",
                steps=[
                    dict(range=[0, 20], color="rgba(255, 0, 85, 0.5)"),
                    dict(range=[20, 40], color="rgba(255, 221, 0, 0.4)"),
                    dict(range=[40, 100], color="rgba(0, 255, 136, 0.3)"),
                ],
                threshold=dict(
                    line=dict(color="red", width=3),
                    thickness=0.75,
                    value=20.0,  # LVD threshold
                ),
            ),
        )
    )

    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#121824",
        font=dict(color="#e2e8f0"),
        margin=dict(l=20, r=20, b=20, t=40),
        height=260,
    )
    return fig.to_dict()


def build_thermal_bar_chart(thermal_state_dict: Dict[str, float]) -> Dict[str, Any]:
    """Generate multi-node temperature distribution chart with operational threshold lines.

    Args:
        thermal_state_dict: Mapping of node name to temperature [K].

    Returns:
        Plotly figure dictionary.
    """
    # Convert K to °C for user clarity
    labels = ["+X Face", "-X Face", "+Y Face", "-Y Face", "+Z Face", "-Z Face", "Avionics", "Battery"]
    keys = [
        "temp_face_pos_x_k", "temp_face_neg_x_k",
        "temp_face_pos_y_k", "temp_face_neg_y_k",
        "temp_face_pos_z_k", "temp_face_neg_z_k",
        "temp_electronics_k", "temp_battery_k",
    ]

    temps_c = [thermal_state_dict.get(k, 293.15) - 273.15 for k in keys]
    colors = ["#00d4ff"] * 6 + ["#ffaa00", "#00ff88"]

    fig = go.Figure(
        data=[
            go.Bar(
                x=labels,
                y=temps_c,
                marker_color=colors,
                text=[f"{t:.1f}°C" for t in temps_c],
                textposition="auto",
            )
        ]
    )

    # Threshold horizontal lines
    # Battery safe charge lower limit: 0°C
    fig.add_hline(y=0.0, line_dash="dash", line_color="#ffdd00", annotation_text="0°C Bat Chg")
    # Battery safe charge upper limit: 45°C
    fig.add_hline(y=45.0, line_dash="dash", line_color="#ff0055", annotation_text="45°C Bat Max")

    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#121824",
        plot_bgcolor="#090d16",
        font=dict(color="#e2e8f0"),
        title=dict(text="Thermal Subsystem Multi-Node Temperature (°C)", font=dict(color="#00d4ff")),
        xaxis=dict(color="#94a3b8", gridcolor="#1e293b"),
        yaxis=dict(title="Temperature [°C]", range=[-35, 65], color="#94a3b8", gridcolor="#1e293b"),
        margin=dict(l=40, r=20, b=40, t=40),
        height=260,
    )
    return fig.to_dict()

