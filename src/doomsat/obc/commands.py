"""Telecommand definition, parsing, and execution dispatcher for DOOMSat OBC."""

from enum import IntEnum
from typing import Dict, Any, Callable, Optional, List, Tuple
from pydantic import BaseModel, Field

from doomsat.obc.fsm import FlightMode


class CommandOpcode(IntEnum):
    PING = 0x01
    SET_FLIGHT_MODE = 0x02
    TRIGGER_CAPTURE = 0x10
    ABORT_DOWNLINK = 0x11
    FORCE_REBOOT = 0x20
    CLEAR_FAULTS = 0x30


class Telecommand(BaseModel):
    """Represents an uplinked telecommand frame for the OBC."""
    opcode: CommandOpcode
    sequence_id: int = Field(..., ge=0, description="Uplink command counter")
    parameters: Dict[str, Any] = Field(default_factory=dict)
    timestamp_s: float = Field(0.0, description="Command dispatch epoch [s]")


class CommandExecutionReceipt(BaseModel):
    """Acknowledgement of command execution."""
    sequence_id: int
    opcode: CommandOpcode
    success: bool
    status_message: str
    execution_time_s: float


class CommandDispatcher:
    """Dispatches telecommands to registered subsystem handlers."""

    def __init__(self):
        self._handlers: Dict[CommandOpcode, Callable[[Telecommand], CommandExecutionReceipt]] = {}
        self.command_history: List[CommandExecutionReceipt] = []

    def register_handler(
        self,
        opcode: CommandOpcode,
        handler: Callable[[Telecommand], CommandExecutionReceipt],
    ) -> None:
        """Register a handler function for a specific opcode."""
        self._handlers[opcode] = handler

    def dispatch(self, command: Telecommand) -> CommandExecutionReceipt:
        """Execute an incoming telecommand and log its receipt."""
        handler = self._handlers.get(command.opcode)
        if handler is None:
            receipt = CommandExecutionReceipt(
                sequence_id=command.sequence_id,
                opcode=command.opcode,
                success=False,
                status_message=f"No handler registered for opcode 0x{command.opcode:02X}",
                execution_time_s=command.timestamp_s,
            )
        else:
            try:
                receipt = handler(command)
            except Exception as e:
                receipt = CommandExecutionReceipt(
                    sequence_id=command.sequence_id,
                    opcode=command.opcode,
                    success=False,
                    status_message=f"Exception executing command: {str(e)}",
                    execution_time_s=command.timestamp_s,
                )

        self.command_history.append(receipt)
        return receipt
