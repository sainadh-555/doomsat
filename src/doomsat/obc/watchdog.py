"""Hardware Watchdog Supervisor for DOOMSat OBC.

Supervises the periodic heartbeat handshake between the Raspberry Pi Zero 2 W
and the ESP32 TT&C Controller to prevent software hang in orbit.
"""

from typing import Optional, List, Tuple


class WatchdogSupervisor:
    """Manages OBC hardware watchdog timer, fault detection, and reboot statistics.

    Attributes:
        timeout_s: Maximum duration between pings before watchdog triggers a reset [s].
        last_kick_time_s: Timestamp of last received heartbeat [s].
        fault_count: Total count of watchdog expiration events.
        is_triggered: True if timer has expired without a kick.
    """

    def __init__(self, timeout_s: float = 60.0, initial_time_s: float = 0.0):
        self.timeout_s: float = float(timeout_s)
        self.last_kick_time_s: float = float(initial_time_s)
        self.fault_count: int = 0
        self.is_triggered: bool = False
        self.fault_log: List[Tuple[float, str]] = []

    def kick(self, current_time_s: float) -> None:
        """Reset the watchdog timer upon receiving a valid heartbeat pulse."""
        self.last_kick_time_s = float(current_time_s)
        self.is_triggered = False

    def check_health(self, current_time_s: float) -> bool:
        """Evaluate if the watchdog has expired.

        Args:
            current_time_s: Current simulation timestamp [s].

        Returns:
            True if healthy (kick received within timeout), False if expired.
        """
        elapsed = current_time_s - self.last_kick_time_s
        if elapsed > self.timeout_s:
            if not self.is_triggered:
                self.is_triggered = True
                self.fault_count += 1
                self.fault_log.append(
                    (current_time_s, f"Watchdog expired: {elapsed:.2f}s elapsed without heartbeat")
                )
            return False
        return True

    def time_until_timeout(self, current_time_s: float) -> float:
        """Calculate remaining seconds before watchdog triggers [s]."""
        elapsed = current_time_s - self.last_kick_time_s
        return max(0.0, self.timeout_s - elapsed)
