"""
hardware_interface.py
SurgiTwin Embedded Hardware Abstraction Layer & Binary Protocol Engine.

Implements robust binary packet framing, CRC16-CCITT integrity verification,
message sequencing, timeout handling, and transport abstraction supporting both
virtual in-memory SIMULATION mode and physical SERIAL port communication.
"""

from dataclasses import dataclass
from enum import Enum, IntEnum
import struct
import time
from typing import List, Optional, Tuple, Union
import numpy as np


class HardwareMode(Enum):
    SIMULATION = "SIMULATION"
    SERIAL = "SERIAL"


class HardwareState(IntEnum):
    BOOT = 0
    READY = 1
    MOVING = 2
    ESTOPPED = 3
    FAULT = 4
    DISCONNECTED = 5


class MsgType(IntEnum):
    CMD_JOINTS = 0x01
    CMD_ESTOP = 0x02
    CMD_RESET = 0x03
    CMD_PING = 0x04
    ACK = 0x06
    NACK = 0x15
    TLM_FEEDBACK = 0x81
    TLM_STATUS = 0x82


@dataclass
class Packet:
    """Decoded protocol packet."""
    msg_type: MsgType
    seq_num: int
    payload: bytes
    crc: int
    valid: bool
    error_message: str = ""


@dataclass
class JointTelemetry:
    """Telemetry data returned by microcontroller joint controllers."""
    timestamp_ms: int
    joint_positions_rad: np.ndarray  # (6,) in radians
    state: HardwareState
    fault_code: int
    is_estopped: bool


class CRC16CCITT:
    """CRC16-CCITT calculation engine (polynomial 0x1021, initial 0xFFFF)."""

    @staticmethod
    def calculate(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc


class ProtocolCodec:
    """
    Binary packet encoder and decoder for SurgiTwin embedded bus.

    Frame format:
    [0xAA, 0x55] (2B) | MsgType (1B) | Seq (1B) | Len (1B) | Payload (N B) | CRC16 (2B) | [0x0D, 0x0A] (2B)
    """

    START_BYTES = bytes([0xAA, 0x55])
    END_BYTES = bytes([0x0D, 0x0A])
    MIN_PACKET_LEN = 9  # 2 + 1 + 1 + 1 + 0 + 2 + 2

    @classmethod
    def encode(cls, msg_type: MsgType, seq_num: int, payload: bytes = b"") -> bytes:
        """
        Encode message into framed binary packet.
        """
        seq = seq_num & 0xFF
        length = len(payload) & 0xFF

        header_and_body = bytes([int(msg_type), seq, length]) + payload
        crc = CRC16CCITT.calculate(header_and_body)

        packet = cls.START_BYTES + header_and_body + struct.pack(">H", crc) + cls.END_BYTES
        return packet

    @classmethod
    def decode(cls, data: bytes) -> Packet:
        """
        Decode and validate binary packet.
        """
        if len(data) < cls.MIN_PACKET_LEN:
            return Packet(
                msg_type=MsgType.NACK, seq_num=0, payload=b"", crc=0,
                valid=False, error_message=f"Packet too short ({len(data)} < {cls.MIN_PACKET_LEN})",
            )

        if not data.startswith(cls.START_BYTES):
            return Packet(
                msg_type=MsgType.NACK, seq_num=0, payload=b"", crc=0,
                valid=False, error_message="Invalid start of frame markers",
            )

        if not data.endswith(cls.END_BYTES):
            return Packet(
                msg_type=MsgType.NACK, seq_num=0, payload=b"", crc=0,
                valid=False, error_message="Invalid end of frame markers",
            )

        msg_type_raw = data[2]
        seq_num = data[3]
        payload_len = data[4]

        expected_total = 2 + 3 + payload_len + 2 + 2
        if len(data) != expected_total:
            return Packet(
                msg_type=MsgType.NACK, seq_num=seq_num, payload=b"", crc=0,
                valid=False, error_message=f"Length mismatch: header says {payload_len} B, total {len(data)} != {expected_total}",
            )

        payload = data[5:5 + payload_len]
        crc_received = struct.unpack(">H", data[5 + payload_len:7 + payload_len])[0]

        crc_computed = CRC16CCITT.calculate(data[2:5 + payload_len])
        if crc_received != crc_computed:
            return Packet(
                msg_type=MsgType.NACK, seq_num=seq_num, payload=payload, crc=crc_received,
                valid=False, error_message=f"CRC failure: received 0x{crc_received:04X}, computed 0x{crc_computed:04X}",
            )

        try:
            m_type = MsgType(msg_type_raw)
        except ValueError:
            return Packet(
                msg_type=MsgType.NACK, seq_num=seq_num, payload=payload, crc=crc_received,
                valid=False, error_message=f"Unknown message type: 0x{msg_type_raw:02X}",
            )

        return Packet(
            msg_type=m_type,
            seq_num=seq_num,
            payload=payload,
            crc=crc_received,
            valid=True,
            error_message="",
        )

    @staticmethod
    def encode_joint_commands(joints_rad: Union[List[float], np.ndarray]) -> bytes:
        """
        Encode 6 joint angles (radians) into 12-byte payload (int16 milliradians).
        """
        arr = np.asarray(joints_rad, dtype=np.float64)
        if len(arr) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(arr)}")

        # Convert to milliradians
        mrad = np.round(arr * 1000.0).astype(np.int16)
        return struct.pack("<6h", *mrad)

    @staticmethod
    def decode_joint_feedback(payload: bytes) -> JointTelemetry:
        """
        Decode 17-byte TLM_FEEDBACK payload:
        6x int16 mrad (12B) + status byte (1B) + timestamp_ms (4B uint32).
        """
        if len(payload) < 17:
            raise ValueError(f"Feedback payload too short: {len(payload)} < 17 bytes")

        mrad = struct.unpack("<6h", payload[:12])
        status_byte = payload[12]
        ts_ms = struct.unpack("<I", payload[13:17])[0]

        joints_rad = np.array(mrad, dtype=np.float64) / 1000.0
        state = HardwareState(status_byte & 0x07)
        is_estopped = bool(status_byte & 0x08)
        fault_code = (status_byte >> 4) & 0x0F

        return JointTelemetry(
            timestamp_ms=ts_ms,
            joint_positions_rad=joints_rad,
            state=state,
            fault_code=fault_code,
            is_estopped=is_estopped,
        )


class HardwareInterface:
    """
    Hardware abstraction layer mediating between ROS 2 / controller and the microcontroller bus.
    """

    def __init__(
        self,
        mode: HardwareMode = HardwareMode.SIMULATION,
        serial_port: Optional[str] = None,
        baud_rate: int = 115200,
        timeout: float = 0.1,
    ):
        self.mode = mode
        self.serial_port_name = serial_port
        self.baud_rate = int(baud_rate)
        self.timeout = float(timeout)

        self.seq_num = 0
        self.is_connected = False
        self.hardware_state = HardwareState.DISCONNECTED
        self.last_telemetry: Optional[JointTelemetry] = None
        self.last_comm_time = 0.0

        # In-memory virtual transport endpoint for simulation mode
        self.virtual_transceiver = None

    def connect(self) -> bool:
        """Establish connection with simulated or physical microcontroller."""
        if self.mode == HardwareMode.SIMULATION:
            self.is_connected = True
            self.hardware_state = HardwareState.READY
            self.last_comm_time = time.time()
            return True

        # Physical serial connection attempt
        if self.serial_port_name:
            try:
                import serial
                self.serial = serial.Serial(
                    port=self.serial_port_name,
                    baudrate=self.baud_rate,
                    timeout=self.timeout,
                )
                self.is_connected = True
                self.hardware_state = HardwareState.READY
                self.last_comm_time = time.time()
                return True
            except Exception:
                self.is_connected = False
                self.hardware_state = HardwareState.DISCONNECTED
                return False

        self.is_connected = False
        self.hardware_state = HardwareState.DISCONNECTED
        return False

    def disconnect(self):
        """Safely disconnect interface."""
        self.is_connected = False
        self.hardware_state = HardwareState.DISCONNECTED

    def send_joint_command(self, joints_rad: Union[List[float], np.ndarray]) -> bool:
        """
        Send joint target position packet to hardware.
        """
        if not self.is_connected or self.hardware_state == HardwareState.DISCONNECTED:
            return False

        payload = ProtocolCodec.encode_joint_commands(joints_rad)
        packet = ProtocolCodec.encode(MsgType.CMD_JOINTS, self.seq_num, payload)
        self.seq_num = (self.seq_num + 1) & 0xFF

        return self._transmit(packet)

    def send_estop(self, assert_stop: bool = True) -> bool:
        """
        Send emergency stop command.
        """
        if not self.is_connected:
            return False

        payload = bytes([1 if assert_stop else 0])
        packet = ProtocolCodec.encode(MsgType.CMD_ESTOP, self.seq_num, payload)
        self.seq_num = (self.seq_num + 1) & 0xFF

        ok = self._transmit(packet)
        if assert_stop:
            self.hardware_state = HardwareState.ESTOPPED
        return ok

    def send_reset(self) -> bool:
        """
        Send reset command to clear faults / E-stop.
        """
        if not self.is_connected:
            return False

        packet = ProtocolCodec.encode(MsgType.CMD_RESET, self.seq_num, b"")
        self.seq_num = (self.seq_num + 1) & 0xFF
        return self._transmit(packet)

    def _transmit(self, packet_bytes: bytes) -> bool:
        """Transmit raw bytes over configured transport."""
        self.last_comm_time = time.time()
        if self.mode == HardwareMode.SIMULATION:
            if self.virtual_transceiver is not None:
                return self.virtual_transceiver.receive_packet(packet_bytes)
            return True
        elif self.mode == HardwareMode.SERIAL and hasattr(self, "serial"):
            try:
                self.serial.write(packet_bytes)
                return True
            except Exception:
                self.is_connected = False
                self.hardware_state = HardwareState.DISCONNECTED
                return False
        return False

