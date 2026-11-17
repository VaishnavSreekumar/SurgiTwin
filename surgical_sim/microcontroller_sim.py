"""
microcontroller_sim.py
SurgiTwin Embedded Microcontroller Simulation Engine.

Simulates an embedded microcontroller (Arduino Due / ESP32) controlling 6 joint actuators
with optical encoder feedback, closed-loop actuator lag, state transitions,
active E-stop braking, telemetry generation, and fault injection capabilities.
"""

from collections import deque
import struct
import time
from typing import Deque, List, Optional, Tuple, Union
import numpy as np

from surgical_sim.hardware_interface import (
    CRC16CCITT,
    HardwareInterface,
    HardwareState,
    JointTelemetry,
    MsgType,
    Packet,
    ProtocolCodec,
)


class MicrocontrollerSimulator:
    """
    Virtual firmware simulator for 6-DOF surgical arm microcontroller.
    """

    ENCODER_RESOLUTION = 4096  # counts per revolution
    RAD_PER_COUNT = (2.0 * np.pi) / ENCODER_RESOLUTION  # ~0.00153 rad

    def __init__(self, initial_joints_rad: Optional[np.ndarray] = None, update_rate_hz: float = 50.0):
        self.rate_hz = float(update_rate_hz)
        self.dt = 1.0 / self.rate_hz

        # Kinematic state
        if initial_joints_rad is not None:
            self.q_target = np.asarray(initial_joints_rad, dtype=np.float64).copy()
            self.q_actual = np.asarray(initial_joints_rad, dtype=np.float64).copy()
        else:
            self.q_target = np.zeros(6, dtype=np.float64)
            self.q_actual = np.zeros(6, dtype=np.float64)

        self.dq_actual = np.zeros(6, dtype=np.float64)
        self.state = HardwareState.READY
        self.is_estopped = False
        self.fault_code = 0
        self.rx_seq_num = 0
        self.tx_seq_num = 0
        self.boot_time = time.time()

        # Telemetry transmission queue
        self.tx_queue: Deque[bytes] = deque()

        # Fault injection controls
        self.fault_drop_packets = False
        self.fault_corrupt_crc = False
        self.fault_disconnected = False

    def reset(self, initial_joints_rad: Optional[np.ndarray] = None):
        """Reset virtual microcontroller to clean boot state."""
        if initial_joints_rad is not None:
            self.q_target = np.asarray(initial_joints_rad, dtype=np.float64).copy()
            self.q_actual = np.asarray(initial_joints_rad, dtype=np.float64).copy()
        else:
            self.q_target.fill(0.0)
            self.q_actual.fill(0.0)

        self.dq_actual.fill(0.0)
        self.state = HardwareState.READY
        self.is_estopped = False
        self.fault_code = 0
        self.tx_queue.clear()
        self.fault_drop_packets = False
        self.fault_corrupt_crc = False
        self.fault_disconnected = False

    def receive_packet(self, packet_bytes: bytes) -> bool:
        """
        Process incoming packet from host hardware interface.
        """
        if self.fault_disconnected:
            return False

        if self.fault_drop_packets:
            return True  # Silently dropped

        packet = ProtocolCodec.decode(packet_bytes)
        if not packet.valid:
            # Send NACK with error
            self._send_nack(packet.seq_num, 0x01)
            return False

        self.rx_seq_num = packet.seq_num

        if packet.msg_type == MsgType.CMD_JOINTS:
            if self.is_estopped or self.state == HardwareState.FAULT:
                self._send_nack(packet.seq_num, 0x02)  # Reject: estopped or faulted
                return False

            if len(packet.payload) != 12:
                self._send_nack(packet.seq_num, 0x03)
                return False

            mrad = struct.unpack("<6h", packet.payload)
            self.q_target = np.array(mrad, dtype=np.float64) / 1000.0
            self.state = HardwareState.MOVING
            self._send_ack(packet.seq_num)
            return True

        elif packet.msg_type == MsgType.CMD_ESTOP:
            assert_stop = bool(packet.payload[0]) if len(packet.payload) > 0 else True
            if assert_stop:
                self.is_estopped = True
                self.state = HardwareState.ESTOPPED
                # Immediately arrest motion
                self.dq_actual.fill(0.0)
                self.q_target = self.q_actual.copy()
            else:
                self.is_estopped = False
                self.state = HardwareState.READY

            self._send_ack(packet.seq_num)
            return True

        elif packet.msg_type == MsgType.CMD_RESET:
            self.is_estopped = False
            self.fault_code = 0
            self.state = HardwareState.READY
            self._send_ack(packet.seq_num)
            return True

        elif packet.msg_type == MsgType.CMD_PING:
            self._send_ack(packet.seq_num)
            return True

        return False

    def step(self, dt: Optional[float] = None) -> JointTelemetry:
        """
        Execute one discrete microcontroller firmware loop cycle.
        """
        time_step = float(dt) if dt is not None else self.dt

        if not self.is_estopped and self.state == HardwareState.MOVING:
            # Low-pass lag filter: dq = (q_target - q_actual) / tau
            tau = 0.08  # Actuator response time constant
            err = self.q_target - self.q_actual
            self.dq_actual = err / tau
            self.q_actual += self.dq_actual * time_step

            # Check if settled within tolerance
            if np.all(np.abs(err) < 0.001):
                self.state = HardwareState.READY
                self.dq_actual.fill(0.0)

        # Quantize through simulated optical encoders
        q_quantized = np.round(self.q_actual / self.RAD_PER_COUNT) * self.RAD_PER_COUNT

        # Status byte: [fault_code (4b) | is_estopped (1b) | state (3b)]
        status_byte = (int(self.state) & 0x07)
        if self.is_estopped:
            status_byte |= 0x08
        status_byte |= ((self.fault_code & 0x0F) << 4)

        elapsed_ms = int((time.time() - self.boot_time) * 1000.0) & 0xFFFFFFFF
        telemetry = JointTelemetry(
            timestamp_ms=elapsed_ms,
            joint_positions_rad=q_quantized.copy(),
            state=self.state,
            fault_code=self.fault_code,
            is_estopped=self.is_estopped,
        )

        # Generate framed TLM_FEEDBACK packet
        mrad = np.round(q_quantized * 1000.0).astype(np.int16)
        payload = struct.pack("<6h", *mrad) + bytes([status_byte]) + struct.pack("<I", elapsed_ms)

        pkt = ProtocolCodec.encode(MsgType.TLM_FEEDBACK, self.tx_seq_num, payload)
        self.tx_seq_num = (self.tx_seq_num + 1) & 0xFF

        if self.fault_corrupt_crc:
            # Invert last CRC byte before delimiter
            pkt = pkt[:-3] + bytes([pkt[-3] ^ 0xFF]) + pkt[-2:]

        if not self.fault_disconnected and not self.fault_drop_packets:
            self.tx_queue.append(pkt)

        return telemetry

    def _send_ack(self, ack_seq: int):
        pkt = ProtocolCodec.encode(MsgType.ACK, self.tx_seq_num, bytes([ack_seq & 0xFF]))
        self.tx_seq_num = (self.tx_seq_num + 1) & 0xFF
        if not self.fault_disconnected and not self.fault_drop_packets:
            self.tx_queue.append(pkt)

    def _send_nack(self, nack_seq: int, error_code: int):
        pkt = ProtocolCodec.encode(MsgType.NACK, self.tx_seq_num, bytes([nack_seq & 0xFF, error_code & 0xFF]))
        self.tx_seq_num = (self.tx_seq_num + 1) & 0xFF
        if not self.fault_disconnected and not self.fault_drop_packets:
            self.tx_queue.append(pkt)

