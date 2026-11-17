"""
test_microcontroller_sim.py
Unit tests for virtual MicrocontrollerSimulator firmware execution and fault injection.
"""

import numpy as np
import pytest

from surgical_sim.hardware_interface import (
    HardwareInterface,
    HardwareMode,
    HardwareState,
    MsgType,
    ProtocolCodec,
)
from surgical_sim.microcontroller_sim import MicrocontrollerSimulator


@pytest.fixture
def mcu():
    sim = MicrocontrollerSimulator()
    return sim


def test_mcu_initialization(mcu):
    """Verify MCU initializes to clean READY state."""
    assert mcu.state == HardwareState.READY
    assert not mcu.is_estopped
    assert np.allclose(mcu.q_actual, np.zeros(6))


def test_mcu_joint_command_execution(mcu):
    """Verify joint command packet updates target and transitions to MOVING."""
    q_cmd = np.array([0.2, -0.4, 0.6, -0.8, 1.0, -1.2])
    payload = ProtocolCodec.encode_joint_commands(q_cmd)
    pkt_bytes = ProtocolCodec.encode(MsgType.CMD_JOINTS, seq_num=1, payload=payload)

    accepted = mcu.receive_packet(pkt_bytes)
    assert accepted
    assert mcu.state == HardwareState.MOVING
    assert np.allclose(mcu.q_target, q_cmd, atol=0.001)

    # Step simulation until settled
    for _ in range(50):
        mcu.step(dt=0.02)

    assert np.allclose(mcu.q_actual, q_cmd, atol=0.005)
    assert mcu.state == HardwareState.READY


def test_mcu_estop_halts_motion(mcu):
    """Emergency stop packet immediately arrests velocity and enters ESTOPPED state."""
    mcu.q_target = np.ones(6)
    mcu.state = HardwareState.MOVING
    mcu.dq_actual = np.ones(6) * 0.5

    estop_pkt = ProtocolCodec.encode(MsgType.CMD_ESTOP, seq_num=2, payload=bytes([1]))
    accepted = mcu.receive_packet(estop_pkt)

    assert accepted
    assert mcu.is_estopped
    assert mcu.state == HardwareState.ESTOPPED
    assert np.allclose(mcu.dq_actual, np.zeros(6))

    # Commands during E-stop must be rejected
    cmd_pkt = ProtocolCodec.encode(MsgType.CMD_JOINTS, seq_num=3, payload=ProtocolCodec.encode_joint_commands(np.zeros(6)))
    assert not mcu.receive_packet(cmd_pkt)


def test_mcu_reset_clears_estop(mcu):
    """CMD_RESET clears E-stop state and restores READY."""
    mcu.is_estopped = True
    mcu.state = HardwareState.ESTOPPED

    reset_pkt = ProtocolCodec.encode(MsgType.CMD_RESET, seq_num=4, payload=b"")
    accepted = mcu.receive_packet(reset_pkt)

    assert accepted
    assert not mcu.is_estopped
    assert mcu.state == HardwareState.READY


def test_mcu_fault_injection_packet_drop(mcu):
    """Dropping packets suppresses transmission to queues."""
    mcu.fault_drop_packets = True
    mcu.step(dt=0.02)

    assert len(mcu.tx_queue) == 0


def test_mcu_fault_injection_crc_corruption(mcu):
    """Corrupted CRC generates invalid outgoing packets."""
    mcu.fault_corrupt_crc = True
    mcu.step(dt=0.02)

    assert len(mcu.tx_queue) > 0
    corrupted_pkt = mcu.tx_queue.popleft()
    decoded = ProtocolCodec.decode(corrupted_pkt)
    assert not decoded.valid
    assert "CRC failure" in decoded.error_message
