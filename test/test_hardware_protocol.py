"""
test_hardware_protocol.py
Unit tests for embedded binary protocol encoding, decoding, and checksum verification.
"""

import struct
import numpy as np
import pytest

from surgical_sim.hardware_interface import (
    CRC16CCITT,
    HardwareInterface,
    HardwareMode,
    HardwareState,
    MsgType,
    Packet,
    ProtocolCodec,
)


def test_packet_encode_decode_round_trip():
    """Verify valid packet encodes with proper framing and decodes accurately."""
    payload = b"TEST_PAYLOAD_123"
    packet_bytes = ProtocolCodec.encode(MsgType.CMD_PING, seq_num=42, payload=payload)

    pkt = ProtocolCodec.decode(packet_bytes)
    assert pkt.valid
    assert pkt.msg_type == MsgType.CMD_PING
    assert pkt.seq_num == 42
    assert pkt.payload == payload


def test_crc16_failure_detection():
    """Corrupted payload or CRC bits must trigger CRC failure."""
    packet_bytes = bytearray(ProtocolCodec.encode(MsgType.CMD_PING, seq_num=1, payload=b"DATA"))

    # Corrupt a single byte in the payload
    packet_bytes[5] ^= 0xFF

    pkt = ProtocolCodec.decode(bytes(packet_bytes))
    assert not pkt.valid
    assert "CRC failure" in pkt.error_message


def test_frame_delimiter_failure():
    """Mismatched start or end markers must be rejected."""
    packet_bytes = bytearray(ProtocolCodec.encode(MsgType.CMD_PING, seq_num=1, payload=b"DATA"))

    # Corrupt start byte
    packet_bytes[0] = 0x00
    pkt = ProtocolCodec.decode(bytes(packet_bytes))
    assert not pkt.valid
    assert "Invalid start of frame" in pkt.error_message

    # Corrupt end byte
    packet_bytes = bytearray(ProtocolCodec.encode(MsgType.CMD_PING, seq_num=1, payload=b"DATA"))
    packet_bytes[-1] = 0x00
    pkt = ProtocolCodec.decode(bytes(packet_bytes))
    assert not pkt.valid
    assert "Invalid end of frame" in pkt.error_message


def test_truncated_packet_rejection():
    """Packets shorter than minimum frame length must be rejected."""
    pkt = ProtocolCodec.decode(b"\xAA\x55\x01")
    assert not pkt.valid
    assert "too short" in pkt.error_message


def test_joint_command_and_feedback_codec():
    """Verify joint angles (rad) to millirad payload and telemetry decoding."""
    q_rad = np.array([0.150, -0.320, 0.450, -0.800, 1.250, -1.570])
    payload = ProtocolCodec.encode_joint_commands(q_rad)
    assert len(payload) == 12

    # Simulate feedback packet (12B joints + 1B status + 4B timestamp)
    status_byte = int(HardwareState.MOVING) | 0x08  # MOVING and ESTOPPED flag
    ts_ms = 123456
    tlm_payload = payload + bytes([status_byte]) + struct.pack("<I", ts_ms)

    tlm = ProtocolCodec.decode_joint_feedback(tlm_payload)
    assert np.allclose(tlm.joint_positions_rad, q_rad, atol=0.001)
    assert tlm.state == HardwareState.MOVING
    assert tlm.is_estopped
    assert tlm.timestamp_ms == ts_ms

