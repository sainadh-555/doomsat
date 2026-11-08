"""Central OBC Flight Software coordinator for DOOMSat.

Simulates the Linux-based flight software running on the Raspberry Pi Zero 2 W:
- State Machine management
- Watchdog tracking
- Telecommand processing
- Telemetry ring buffer logging
- Dynamic power consumption accounting
"""

from typing import List, Optional, Deque
from collections import deque
import logging

from doomsat.obc.fsm import OBCStateMachine, FlightMode
from doomsat.obc.watchdog import WatchdogSupervisor
from doomsat.obc.commands import (
    CommandDispatcher,
    Telecommand,
    CommandOpcode,
    CommandExecutionReceipt,
)
from doomsat.core.schemas import TelemetryBeacon

logger = logging.getLogger(__name__)


class OBCFlightSoftware:
    """Integrated flight software simulator for the Raspberry Pi Zero 2 W OBC.

    Attributes:
        fsm: Finite State Machine tracking operational flight modes.
        watchdog: Hardware watchdog supervisor.
        dispatcher: Telecommand router and execution engine.
        telemetry_buffer: In-memory circular ring buffer of recent telemetry beacons.
    """

    def __init__(
        self,
        initial_mode: FlightMode = FlightMode.BOOT,
        boot_delay_s: float = 0.0,
        watchdog_timeout_s: float = 60.0,
        telemetry_buffer_capacity: int = 1000,
    ):
        self.fsm = OBCStateMachine(initial_mode=initial_mode, boot_delay_s=boot_delay_s)
        self.watchdog = WatchdogSupervisor(timeout_s=watchdog_timeout_s)
        self.dispatcher = CommandDispatcher()
        self.telemetry_buffer: Deque[TelemetryBeacon] = deque(maxlen=telemetry_buffer_capacity)
        self.reboot_count: int = 0
        self.capture_pending: bool = False
        self.image_queue: list[int] = []  # Image IDs awaiting downlink

        self._setup_default_commands()

    def _setup_default_commands(self) -> None:
        """Register built-in handlers for standard telecommands."""
        self.dispatcher.register_handler(CommandOpcode.PING, self._handle_ping)
        self.dispatcher.register_handler(CommandOpcode.SET_FLIGHT_MODE, self._handle_set_mode)
        self.dispatcher.register_handler(CommandOpcode.TRIGGER_CAPTURE, self._handle_trigger_capture)
        self.dispatcher.register_handler(CommandOpcode.FORCE_REBOOT, self._handle_force_reboot)

    def _handle_ping(self, cmd: Telecommand) -> CommandExecutionReceipt:
        return CommandExecutionReceipt(
            sequence_id=cmd.sequence_id,
            opcode=cmd.opcode,
            success=True,
            status_message="PONG: OBC operational",
            execution_time_s=cmd.timestamp_s,
        )

    def _handle_set_mode(self, cmd: Telecommand) -> CommandExecutionReceipt:
        mode_str = cmd.parameters.get("mode")
        try:
            target_mode = FlightMode(mode_str)
            success = self.fsm.transition_to(
                target_mode,
                cmd.timestamp_s,
                f"Telecommand override (seq {cmd.sequence_id})"
            )
            msg = f"Mode transitioned to {target_mode.value}" if success else f"Transition to {target_mode.value} rejected"
            return CommandExecutionReceipt(
                sequence_id=cmd.sequence_id,
                opcode=cmd.opcode,
                success=success,
                status_message=msg,
                execution_time_s=cmd.timestamp_s,
            )
        except Exception as e:
            return CommandExecutionReceipt(
                sequence_id=cmd.sequence_id,
                opcode=cmd.opcode,
                success=False,
                status_message=f"Invalid mode parameter: {mode_str} ({str(e)})",
                execution_time_s=cmd.timestamp_s,
            )

    def _handle_trigger_capture(self, cmd: Telecommand) -> CommandExecutionReceipt:
        if self.fsm.current_mode == FlightMode.CRITICAL_LOW_POWER:
            return CommandExecutionReceipt(
                sequence_id=cmd.sequence_id,
                opcode=cmd.opcode,
                success=False,
                status_message="Capture rejected: In CRITICAL_LOW_POWER mode",
                execution_time_s=cmd.timestamp_s,
            )
        self.capture_pending = True
        return CommandExecutionReceipt(
            sequence_id=cmd.sequence_id,
            opcode=cmd.opcode,
            success=True,
            status_message="Camera capture scheduled",
            execution_time_s=cmd.timestamp_s,
        )

    def _handle_force_reboot(self, cmd: Telecommand) -> CommandExecutionReceipt:
        self.reboot_count += 1
        self.fsm.current_mode = FlightMode.BOOT
        self.fsm.mode_entry_time_s = cmd.timestamp_s
        self.watchdog.kick(cmd.timestamp_s)
        return CommandExecutionReceipt(
            sequence_id=cmd.sequence_id,
            opcode=cmd.opcode,
            success=True,
            status_message=f"Forced reboot executed (reboot count={self.reboot_count})",
            execution_time_s=cmd.timestamp_s,
        )

    def step(
        self,
        current_time_s: float,
        battery_soc_pct: float,
        omega_norm_radps: float,
        gs_visible: bool = False,
    ) -> FlightMode:
        """Execute one simulation cycle of the OBC software."""
        # Check watchdog health
        is_healthy = self.watchdog.check_health(current_time_s)
        if not is_healthy:
            logger.warning(f"OBC Watchdog expired at t={current_time_s}s! Forcing safe mode reset.")
            self.reboot_count += 1
            self.fsm.transition_to(FlightMode.SAFE_HOLD, current_time_s, "Watchdog timeout reset")
            self.watchdog.kick(current_time_s)

        # Autonomous FSM progression
        image_downlink_pending = len(self.image_queue) > 0
        new_mode = self.fsm.evaluate_step(
            current_time_s=current_time_s,
            battery_soc_pct=battery_soc_pct,
            omega_norm_radps=omega_norm_radps,
            gs_visible=gs_visible,
            image_downlink_pending=image_downlink_pending,
            capture_requested=self.capture_pending,
        )

        # If we entered IMAGE_CAPTURE, clear pending flag
        if new_mode == FlightMode.IMAGE_CAPTURE:
            self.capture_pending = False

        return new_mode

    def record_telemetry(self, beacon: TelemetryBeacon) -> None:
        """Store a housekeeping beacon into the circular ring buffer."""
        self.telemetry_buffer.append(beacon)

    @property
    def power_consumption_w(self) -> float:
        """Current instantaneous electrical power drawn by the OBC subsystem [W]."""
        return self.fsm.current_power_draw_w
