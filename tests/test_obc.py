"""Unit tests for DOOMSat On-Board Computer (RPi Zero 2 W) flight software."""

import pytest
from doomsat.obc import (
    FlightMode,
    OBCStateMachine,
    WatchdogSupervisor,
    CommandOpcode,
    Telecommand,
    CommandDispatcher,
    OBCFlightSoftware,
    OBC_POWER_PROFILE_W,
)


def test_fsm_initial_state_and_valid_transitions():
    """Verify FSM initial mode and legal state progression."""
    fsm = OBCStateMachine(initial_mode=FlightMode.BOOT, boot_delay_s=5.0)
    assert fsm.current_mode == FlightMode.BOOT

    # Illegal transition: BOOT -> TRANSMIT_IMAGE should be rejected
    assert not fsm.transition_to(FlightMode.TRANSMIT_IMAGE, current_time_s=1.0, reason="Test")
    assert fsm.current_mode == FlightMode.BOOT

    # Legal transition: BOOT -> SAFE_HOLD
    assert fsm.transition_to(FlightMode.SAFE_HOLD, current_time_s=6.0, reason="Boot done")
    assert fsm.current_mode == FlightMode.SAFE_HOLD


def test_fsm_low_battery_cutoff_and_hysteresis():
    """Verify emergency critical low power mode trigger and recovery hysteresis."""
    fsm = OBCStateMachine(initial_mode=FlightMode.NOMINAL)

    # Battery drops below 20% -> triggers CRITICAL_LOW_POWER
    new_mode = fsm.evaluate_step(
        current_time_s=10.0,
        battery_soc_pct=18.5,
        omega_norm_radps=0.005,
    )
    assert new_mode == FlightMode.CRITICAL_LOW_POWER

    # Battery rises to 35% -> still in CRITICAL_LOW_POWER (hysteresis requires >= 40%)
    new_mode = fsm.evaluate_step(
        current_time_s=20.0,
        battery_soc_pct=35.0,
        omega_norm_radps=0.005,
    )
    assert new_mode == FlightMode.CRITICAL_LOW_POWER

    # Battery rises to 42% -> recovers to SAFE_HOLD
    new_mode = fsm.evaluate_step(
        current_time_s=30.0,
        battery_soc_pct=42.0,
        omega_norm_radps=0.005,
    )
    assert new_mode == FlightMode.SAFE_HOLD


def test_fsm_detumble_progression():
    """Verify tumbling rate triggers DETUMBLE and rate reduction returns to NOMINAL."""
    fsm = OBCStateMachine(initial_mode=FlightMode.NOMINAL)

    # High spin rate (0.08 rad/s > 0.05 rad/s)
    new_mode = fsm.evaluate_step(
        current_time_s=10.0,
        battery_soc_pct=85.0,
        omega_norm_radps=0.08,
    )
    assert new_mode == FlightMode.DETUMBLE

    # Reduced spin rate (0.008 rad/s <= 0.01 rad/s)
    new_mode = fsm.evaluate_step(
        current_time_s=50.0,
        battery_soc_pct=84.0,
        omega_norm_radps=0.008,
    )
    assert new_mode == FlightMode.NOMINAL


def test_fsm_ground_station_pass_transitions():
    """Verify downlink mode transitions during ground station passes."""
    fsm = OBCStateMachine(initial_mode=FlightMode.NOMINAL)

    # GS visible with no pending images -> TRANSMIT_TELEMETRY
    new_mode = fsm.evaluate_step(
        current_time_s=100.0,
        battery_soc_pct=90.0,
        omega_norm_radps=0.005,
        gs_visible=True,
        image_downlink_pending=False,
    )
    assert new_mode == FlightMode.TRANSMIT_TELEMETRY

    # GS passes out of view -> returns to NOMINAL
    new_mode = fsm.evaluate_step(
        current_time_s=150.0,
        battery_soc_pct=89.0,
        omega_norm_radps=0.005,
        gs_visible=False,
    )
    assert new_mode == FlightMode.NOMINAL

    # GS visible with pending image -> TRANSMIT_IMAGE
    new_mode = fsm.evaluate_step(
        current_time_s=200.0,
        battery_soc_pct=88.0,
        omega_norm_radps=0.005,
        gs_visible=True,
        image_downlink_pending=True,
    )
    assert new_mode == FlightMode.TRANSMIT_IMAGE


def test_obc_power_profiles():
    """Verify OBC power consumption profile across all flight modes."""
    fsm = OBCStateMachine(initial_mode=FlightMode.NOMINAL)
    assert fsm.current_power_draw_w == 0.70

    fsm.transition_to(FlightMode.IMAGE_CAPTURE, current_time_s=10.0)
    assert fsm.current_power_draw_w == 1.45

    fsm.transition_to(FlightMode.SAFE_HOLD, current_time_s=20.0)
    assert fsm.current_power_draw_w == 0.40


def test_watchdog_supervisor():
    """Verify hardware watchdog timer, kick mechanism, and timeout detection."""
    wd = WatchdogSupervisor(timeout_s=60.0, initial_time_s=0.0)

    # Health check at t = 30s is healthy
    assert wd.check_health(current_time_s=30.0)
    assert wd.time_until_timeout(current_time_s=30.0) == 30.0

    # Kick at t = 45s resets timer
    wd.kick(current_time_s=45.0)
    assert wd.time_until_timeout(current_time_s=45.0) == 60.0

    # No kick until t = 110s (elapsed 65s > 60s) -> expired
    assert not wd.check_health(current_time_s=110.0)
    assert wd.is_triggered
    assert wd.fault_count == 1


def test_command_dispatcher():
    """Verify telecommand dispatching, parameter extraction, and execution receipts."""
    from doomsat.obc.commands import CommandExecutionReceipt
    dispatcher = CommandDispatcher()
    dispatcher.register_handler(
        CommandOpcode.PING,
        lambda cmd: CommandExecutionReceipt(
            sequence_id=cmd.sequence_id,
            opcode=cmd.opcode,
            success=True,
            status_message="PONG",
            execution_time_s=cmd.timestamp_s,
        )
    )

    cmd = Telecommand(opcode=CommandOpcode.PING, sequence_id=1, timestamp_s=10.0)
    receipt = dispatcher.dispatch(cmd)

    assert receipt.opcode == CommandOpcode.PING
    assert receipt.sequence_id == 1
    assert receipt.success
    assert len(dispatcher.command_history) == 1


def test_integrated_obc_flight_software():
    """Verify complete integrated OBC software execution and watchdog recovery."""
    obc = OBCFlightSoftware(initial_mode=FlightMode.NOMINAL, watchdog_timeout_s=60.0)

    # Step at t = 10s: normal nominal execution
    mode = obc.step(current_time_s=10.0, battery_soc_pct=95.0, omega_norm_radps=0.005)
    assert mode == FlightMode.NOMINAL

    # Watchdog kicks periodically
    obc.watchdog.kick(current_time_s=10.0)

    # Advance time to t = 80s without kicking -> watchdog forces SAFE_HOLD reset
    mode = obc.step(current_time_s=80.0, battery_soc_pct=94.0, omega_norm_radps=0.005)
    assert mode == FlightMode.SAFE_HOLD
    assert obc.reboot_count == 1
