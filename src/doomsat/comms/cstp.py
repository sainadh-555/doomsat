"""CubeSat Space Transfer Protocol (CSTP) and SLIP framing implementation for DOOMSat.

Implements layered packet communication according to DOOM-ICD-001:
- Layer 1/2: SLIP (RFC 1055) frame delimitation and byte stuffing.
- Layer 2/3: Binary CSTP packet header, CRC-16-CCITT integrity, and payload routing.
"""

from enum import IntEnum
import struct
from typing import Optional, List, Tuple
from pydantic import BaseModel, Field

from doomsat.payload.fragmenter import compute_crc16_ccitt

# SLIP (Serial Line Internet Protocol, RFC 1055) Framing Constants
SLIP_END: int = 0xC0      # Frame delimiter
SLIP_ESC: int = 0xDB      # Escape character
SLIP_ESC_END: int = 0xDC  # Escaped END byte (0xDB 0xDC represents 0xC0)
SLIP_ESC_ESC: int = 0xDD  # Escaped ESC byte (0xDB 0xDD represents 0xDB)

# CSTP Protocol Constants
CSTP_SYNC_WORD: bytes = b"\xD0\x01"  # "DOOMSat-1" magic marker (2 bytes)
CSTP_HEADER_LEN: int = 10            # Sync (2) + Src (1) + Dst (1) + Type (1) + Seq (2) + Len (2) + Pad/Reserved (1)
CSTP_MAX_PAYLOAD: int = 220          # Maximum Transmission Unit (MTU) payload size [bytes]


class NodeID(IntEnum):
    """Subsystem node identifiers for packet source and destination routing."""
    GROUND_STATION = 0x01
    SPACECRAFT_TTC = 0x02
    OBC = 0x03
    ADCS = 0x04
    PAYLOAD = 0x05
    EPS = 0x06
    BROADCAST = 0xFF


class MessageType(IntEnum):
    """CSTP Message types and application opcodes."""
    PING = 0x01
    PONG = 0x02
    TELEMETRY_BEACON = 0x10
    ADCS_TELEMETRY = 0x11
    EPS_TELEMETRY = 0x12
    IMAGE_DESCRIPTOR = 0x20
    IMAGE_CHUNK = 0x21
    CHUNK_ACK_NACK = 0x22
    COMMAND_REQUEST = 0x30
    COMMAND_RESPONSE = 0x31


def slip_encode(data: bytes) -> bytes:
    """Encode binary data using SLIP framing and byte stuffing.

    Encapsulates the frame with SLIP_END (0xC0) at start and end.
    Replaces 0xC0 with 0xDB 0xDC, and 0xDB with 0xDB 0xDD.

    Args:
        data: Arbitrary binary data.

    Returns:
        Framed bytes.
    """
    encoded = bytearray([SLIP_END])
    for byte in data:
        if byte == SLIP_END:
            encoded.extend([SLIP_ESC, SLIP_ESC_END])
        elif byte == SLIP_ESC:
            encoded.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            encoded.append(byte)
    encoded.append(SLIP_END)
    return bytes(encoded)


def slip_decode(data: bytes) -> bytes:
    """Decode a SLIP-framed byte sequence, removing escape bytes and delimiters.

    Args:
        data: SLIP-encoded binary stream.

    Returns:
        Original unstuffed binary data.

    Raises:
        ValueError if the frame is malformed or improperly terminated.
    """
    # Strip leading/trailing delimiters
    raw = data.strip(bytes([SLIP_END]))
    decoded = bytearray()
    i = 0
    n = len(raw)

    while i < n:
        byte = raw[i]
        if byte == SLIP_ESC:
            if i + 1 >= n:
                raise ValueError("Malformed SLIP frame: trailing escape character")
            next_byte = raw[i + 1]
            if next_byte == SLIP_ESC_END:
                decoded.append(SLIP_END)
            elif next_byte == SLIP_ESC_ESC:
                decoded.append(SLIP_ESC)
            else:
                raise ValueError(f"Malformed SLIP frame: invalid escape sequence 0xDB 0x{next_byte:02X}")
            i += 2
        elif byte == SLIP_END:
            # Spurious internal delimiter
            i += 1
        else:
            decoded.append(byte)
            i += 1

    return bytes(decoded)


class CSTPPacket:
    """Represents a structured binary CSTP (CubeSat Space Transfer Protocol) packet.

    Header Format (10 bytes):
        - Sync Word: 2 bytes (0xD0 0x01)
        - Source Node ID: 1 byte
        - Destination Node ID: 1 byte
        - Message Type: 1 byte
        - Sequence Number: 2 bytes (uint16, big-endian)
        - Payload Length: 2 bytes (uint16, big-endian, <= 220)
        - Reserved / Flags: 1 byte

    Body & Footer:
        - Payload: N bytes
        - CRC-16-CCITT: 2 bytes (uint16, big-endian, over header + payload)
    """

    def __init__(
        self,
        src_id: NodeID | int,
        dst_id: NodeID | int,
        msg_type: MessageType | int,
        sequence_num: int,
        payload: bytes = b"",
        flags: int = 0x00,
    ):
        if len(payload) > CSTP_MAX_PAYLOAD:
            raise ValueError(f"Payload length {len(payload)} exceeds CSTP MTU of {CSTP_MAX_PAYLOAD} bytes")

        self.src_id: int = int(src_id)
        self.dst_id: int = int(dst_id)
        self.msg_type: int = int(msg_type)
        self.sequence_num: int = int(sequence_num) & 0xFFFF
        self.payload: bytes = bytes(payload)
        self.flags: int = int(flags) & 0xFF

    @property
    def payload_length(self) -> int:
        return len(self.payload)

    def pack(self) -> bytes:
        """Serialize packet to binary format with CRC-16 checksum (unframed).

        Returns:
            Binary bytes (12 + N bytes).
        """
        # Header (10 bytes): >2s B B B H H B
        header = struct.pack(
            ">2sBBBHB",
            CSTP_SYNC_WORD,
            self.src_id,
            self.dst_id,
            self.msg_type,
            self.sequence_num,
            self.payload_length,
        )
        # Note: struct.pack(">2sBBBHB") is 2+1+1+1+2+1 = 8 bytes.
        # Let's use exact 10 bytes:
        # Sync (2) + Src (1) + Dst (1) + Type (1) + Flags (1) + Seq (2) + Len (2) = 10 bytes!
        header = struct.pack(
            ">2sBBBBHH",
            CSTP_SYNC_WORD,
            self.src_id,
            self.dst_id,
            self.msg_type,
            self.flags,
            self.sequence_num,
            self.payload_length,
        )

        body = header + self.payload
        crc = compute_crc16_ccitt(body)
        return body + struct.pack(">H", crc)

    @classmethod
    def unpack(cls, raw_data: bytes) -> "CSTPPacket":
        """Deserialize raw binary data into a CSTPPacket and verify CRC-16.

        Args:
            raw_data: Unframed packet bytes (minimum 12 bytes).

        Returns:
            Validated CSTPPacket instance.

        Raises:
            ValueError if header is invalid or CRC check fails.
        """
        if len(raw_data) < 12:
            raise ValueError(f"Packet length {len(raw_data)} is less than minimum CSTP size (12 bytes)")

        # Verify sync word
        sync = raw_data[:2]
        if sync != CSTP_SYNC_WORD:
            raise ValueError(f"Invalid CSTP sync word: expected {CSTP_SYNC_WORD.hex()}, got {sync.hex()}")

        # Unpack header (10 bytes)
        sync_word, src, dst, m_type, flags, seq, p_len = struct.unpack(">2sBBBBHH", raw_data[:10])

        if len(raw_data) < 10 + p_len + 2:
            raise ValueError(f"Packet truncated: expected {10 + p_len + 2} bytes, got {len(raw_data)}")

        payload = raw_data[10 : 10 + p_len]
        received_crc = struct.unpack(">H", raw_data[10 + p_len : 10 + p_len + 2])[0]

        # Verify CRC
        computed_crc = compute_crc16_ccitt(raw_data[: 10 + p_len])
        if received_crc != computed_crc:
            raise ValueError(
                f"CSTP CRC-16 mismatch: received 0x{received_crc:04X}, computed 0x{computed_crc:04X}"
            )

        packet = cls(
            src_id=src,
            dst_id=dst,
            msg_type=m_type,
            sequence_num=seq,
            payload=payload,
            flags=flags,
        )
        return packet

    def to_slip_frame(self) -> bytes:
        """Serialize packet and wrap in SLIP framing for serial/UART/radio transmission."""
        return slip_encode(self.pack())

    @classmethod
    def from_slip_frame(cls, slip_data: bytes) -> "CSTPPacket":
        """Unwrap SLIP framing and deserialize CSTPPacket."""
        unwrapped = slip_decode(slip_data)
        return cls.unpack(unwrapped)
