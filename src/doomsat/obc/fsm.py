"""OBC Finite State Machine (FSM) and flight operating modes.

Models the flight software operating states on the Raspberry Pi Zero 2 W:
- BOOT: Post-deployment initialization and RF silence timer.
- SAFE_HOLD: Low-power stabilization mode.
- DETUMBLE: Active attitude detumbling.
- NOMINAL: Routine orbital operations and beaconing.
- IMAGE_CAPTURE: Nadir Earth observation.
- TRANSMIT_TELEMETRY: High-rate housekeeping downlink during GS pass.
- TRANSMIT_IMAGE: Packetized image downlink during GS pass.
- CRITICAL_LOW_POWER: Emergency survival mode (SoC < 20%).
"""

from enum import Enum
from typing import Dict, Set, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class FlightMode(str, Enum):
    BOOT = "BOOT"
    SAFE_HOLD = "SAFE_HOLD"
    DETUMBLE = "DETUMBLE"
    NOMINAL = "NOMINAL"
    IMAGE_CAPTURE = "IMAGE_CAPTURE"
    TRANSMIT_TELEMETRY = "TRANSMIT_TELEMETRY"
    TRANSMIT_IMAGE = "TRANSMIT_IMAGE"
    CRITICAL_LOW_POWER = "CRITICAL_LOW_POWER"


# Permissible state transitions matrix
VALID_TRANSITIONS: Dict[FlightMode, Set[FlightMode]] = {
    FlightMode.BOOT: {
        FlightMode.SAFE_HOLD,
        FlightMode.CRITICAL_LOW_POWER,
    },
    FlightMode.SAFE_HOLD: {
        FlightMode.DETUMBLE,
        FlightMode.NOMINAL,
        FlightMode.CRITICAL_LOW_POWER,
    },
    FlightMode.DETUMBLE: {
        FlightMode.NOMINAL,
        FlightMode.SAFE_HOLD,
        FlightMode.CRITICAL_LOW_POWER,
    },
    FlightMode.NOMINAL: {
        FlightMode.IMAGE_CAPTURE,
        FlightMode.TRANSMIT_TELEMETRY,
        FlightMode.TRANSMIT_IMAGE,
        FlightMode.DETUMBLE,
        FlightMode.SAFE_HOLD,
        FlightMode.CRITICAL_LOW_POWER,
    },
    FlightMode.IMAGE_CAPTURE: {
        FlightMode.NOMINAL,
        FlightMode.SAFE_HOLD,
        FlightMode.CRITICAL_LOW_POWER,
    },
    FlightMode.TRANSMIT_TELEMETRY: {
        FlightMode.TRANSMIT_IMAGE,
        FlightMode.NOMINAL,
        FlightMode.SAFE_HOLD,
        FlightMode.CRITICAL_LOW_POWER,
    },
    FlightMode.TRANSMIT_IMAGE: {
        FlightMode.TRANSMIT_TELEMETRY,
        FlightMode.NOMINAL,
        FlightMode.SAFE_HOLD,
        FlightMode.CRITICAL_LOW_POWER,
    },
    FlightMode.CRITICAL_LOW_POWER: {
        FlightMode.SAFE_HOLD,
    },
}

# OBC Power consumption profile [Watts]
OBC_POWER_PROFILE_W: Dict[FlightMode, float] = {
    FlightMode.BOOT: 0.85,                 # Booting Linux OS + diagnostics
    FlightMode.SAFE_HOLD: 0.40,            # CPU throttled, minimum background daemons
    FlightMode.DETUMBLE: 0.70,             # Processing ADCS rates
    FlightMode.NOMINAL: 0.70,              # Baseline idle flight software
    FlightMode.IMAGE_CAPTURE: 1.45,        # OBC full clock + Camera CSI-2 active (700mW + 250mW + ISP)
    FlightMode.TRANSMIT_TELEMETRY: 0.80,   # Packet assembly & UART streaming
    FlightMode.TRANSMIT_IMAGE: 0.95,       # High-throughput packet streaming
    FlightMode.CRITICAL_LOW_POWER: 0.25,   # CPU ultra-low power standby
}


class OBCStateMachine:
    """Manages OBC flight mode transitions, conditions, and power states."""

    def __init__(self, initial_mode: FlightMode = FlightMode.BOOT, boot_delay_s: float = 0.0):
        self.current_mode: FlightMode = initial_mode
        self.previous_mode: Optional[FlightMode] = None
        self.mode_entry_time_s: float = 0.0
        self.boot_delay_s: float = boot_delay_s
        self.boot_completed: bool = (initial_mode != FlightMode.BOOT)
        self.transition_history: list[Tuple[float, FlightMode, FlightMode, str]] = []

    def can_transition(self, target_mode: FlightMode) -> bool:
        """Check if transition from current mode to target mode is legally permitted."""
        if target_mode == self.current_mode:
            return True
        allowed = VALID_TRANSITIONS.get(self.current_mode, set())
        return target_mode in allowed

    def transition_to(self, target_mode: FlightMode, current_time_s: float, reason: str = "") -> bool:
        """Execute a state transition if permitted.

        Args:
            target_mode: Requested FlightMode.
            current_time_s: Current simulation timestamp [s].
            reason: Engineering rationale for the transition.

        Returns:
            True if transition succeeded, False otherwise.
        """
        if target_mode == self.current_mode:
            return True

        if not self.can_transition(target_mode):
            logger.warning(
                f"Illegal flight mode transition requested: {self.current_mode} -> {target_mode} (Reason: {reason})"
            )
            return False

        self.previous_mode = self.current_mode
        self.current_mode = target_mode
        self.mode_entry_time_s = current_time_s
        self.transition_history.append((current_time_s, self.previous_mode, target_mode, reason))

        if target_mode != FlightMode.BOOT:
            self.boot_completed = True

        return True

    def evaluate_step(
        self,
        current_time_s: float,
        battery_soc_pct: float,
        omega_norm_radps: float,
        gs_visible: bool = False,
        image_downlink_pending: bool = False,
        capture_requested: bool = False,
    ) -> FlightMode:
        """Evaluate autonomous flight mode transitions based on spacecraft state.

        Rules:
        1. If SoC < 20%, transition immediately to CRITICAL_LOW_POWER.
        2. In CRITICAL_LOW_POWER, only recover to SAFE_HOLD when SoC > 40%.
        3. In BOOT, exit to SAFE_HOLD after boot_delay_s has elapsed.
        4. If tumbling (omega_norm > 0.05 rad/s ~= 2.86 deg/s), transition to DETUMBLE.
        5. If detumbled (omega_norm <= 0.01 rad/s ~= 0.57 deg/s) and in DETUMBLE, transition to NOMINAL.
        6. In NOMINAL, if capture_requested and battery > 50%, transition to IMAGE_CAPTURE.
        7. In NOMINAL, if GS visible, transition to TRANSMIT_IMAGE (if pending) or TRANSMIT_TELEMETRY.
        8. In TRANSMIT_*, if GS no longer visible, return to NOMINAL.
        """
        # Rule 1: Low battery emergency cut-off
        if battery_soc_pct < 20.0 and self.current_mode != FlightMode.CRITICAL_LOW_POWER:
            self.transition_to(
                FlightMode.CRITICAL_LOW_POWER,
                current_time_s,
                f"Critical battery cutoff: SoC={battery_soc_pct:.1f}% < 20.0%"
            )
            return self.current_mode

        # Rule 2: Recovery from critical low power
        if self.current_mode == FlightMode.CRITICAL_LOW_POWER:
            if battery_soc_pct >= 40.0:
                self.transition_to(
                    FlightMode.SAFE_HOLD,
                    current_time_s,
                    f"Battery recovered: SoC={battery_soc_pct:.1f}% >= 40.0%"
                )
            return self.current_mode

        # Rule 3: Post-ejection boot sequence
        if self.current_mode == FlightMode.BOOT:
            if (current_time_s - self.mode_entry_time_s) >= self.boot_delay_s:
                self.transition_to(
                    FlightMode.SAFE_HOLD,
                    current_time_s,
                    "Boot sequence & RF silence delay completed"
                )
            return self.current_mode

        # Rule 4 & 5: Detumbling logic
        if omega_norm_radps > 0.05:  # Tumbling threshold (~2.86 deg/s)
            if self.current_mode != FlightMode.DETUMBLE:
                self.transition_to(
                    FlightMode.DETUMBLE,
                    current_time_s,
                    f"Excessive angular rate: {omega_norm_radps:.3f} rad/s > 0.05 rad/s"
                )
                return self.current_mode

        if self.current_mode == FlightMode.DETUMBLE:
            if omega_norm_radps <= 0.01:  # Detumbled threshold (~0.57 deg/s)
                self.transition_to(
                    FlightMode.NOMINAL,
                    current_time_s,
                    f"Detumble completed: {omega_norm_radps:.4f} rad/s <= 0.01 rad/s"
                )
                return self.current_mode

        # Rule 6: Image capture trigger
        if self.current_mode == FlightMode.NOMINAL and capture_requested:
            if battery_soc_pct >= 50.0:
                self.transition_to(
                    FlightMode.IMAGE_CAPTURE,
                    current_time_s,
                    "Triggering scheduled payload image capture"
                )
                return self.current_mode

        # Rule 7: Ground station downlink pass
        if gs_visible:
            if self.current_mode == FlightMode.NOMINAL:
                if image_downlink_pending:
                    self.transition_to(
                        FlightMode.TRANSMIT_IMAGE,
                        current_time_s,
                        "Ground station in view: Downlinking payload imagery"
                    )
                else:
                    self.transition_to(
                        FlightMode.TRANSMIT_TELEMETRY,
                        current_time_s,
                        "Ground station in view: Downlinking housekeeping telemetry"
                    )
                return self.current_mode
        else:
            # Rule 8: End of pass return to nominal
            if self.current_mode in (FlightMode.TRANSMIT_IMAGE, FlightMode.TRANSMIT_TELEMETRY):
                self.transition_to(
                    FlightMode.NOMINAL,
                    current_time_s,
                    "Ground station pass concluded"
                )
                return self.current_mode

        return self.current_mode

    @property
    def current_power_draw_w(self) -> float:
        """Current estimated electrical power consumption of the OBC subsystem [W]."""
        return OBC_POWER_PROFILE_W.get(self.current_mode, 0.70)
