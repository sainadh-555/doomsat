"""On-Board Computer (Raspberry Pi Zero 2 W) flight software simulation package."""

from doomsat.obc.fsm import (
    FlightMode,
    OBCStateMachine,
    VALID_TRANSITIONS,
    OBC_POWER_PROFILE_W,
)
from doomsat.obc.watchdog import WatchdogSupervisor
from doomsat.obc.commands import (
    CommandOpcode,
    Telecommand,
    CommandExecutionReceipt,
    CommandDispatcher,
)
from doomsat.obc.flight_software import OBCFlightSoftware

__all__ = [
    "FlightMode",
    "OBCStateMachine",
    "VALID_TRANSITIONS",
    "OBC_POWER_PROFILE_W",
    "WatchdogSupervisor",
    "CommandOpcode",
    "Telecommand",
    "CommandExecutionReceipt",
    "CommandDispatcher",
    "OBCFlightSoftware",
]
