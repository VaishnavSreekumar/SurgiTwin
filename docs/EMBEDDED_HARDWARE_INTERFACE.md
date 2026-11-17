# SurgiTwin — Embedded Hardware Abstraction & Binary Protocol

## 1. Embedded Communication Architecture

SurgiTwin provides a Hardware Abstraction Layer (HAL) connecting the high-level ROS 2 controller to embedded microcontrollers (e.g. Arduino Due, ESP32, or STM32) governing joint servo drives.

```
ROS 2 Architecture (/joint_states, /safety/emergency_stop)
                          |
                          v
        +-----------------------------------+
        | HardwareInterface (HAL)           |
        |  - Mode: SIMULATION or SERIAL     |
        |  - ProtocolCodec Framing & CRC16  |
        |  - Sequence Tracking & Timeout    |
        +-----------------------------------+
                          |
        +-----------------+-----------------+
        |                                   |
        v                                   v
[SIMULATION Mode]                   [SERIAL Mode]
Virtual UART in-memory loopback     Physical USB-Serial Port
        |                           (/dev/ttyUSB0 / /dev/ttyACM0)
        v                                   v
MicrocontrollerSimulator            Physical Microcontroller
- 6 Actuator lag channels           (Arduino / ESP32 Firmware)
- Optical encoder feedback
- Active E-stop electromagnetic damping
```

## 2. Binary Packet Protocol Specification

Communication employs a robust, byte-oriented framed protocol with CRC16-CCITT integrity verification.

### Frame Layout (Overhead: 9 Bytes)
```
+--------------+----------+---------+--------+----------------+-----------+--------------+
| Start Marker | Msg Type | Seq Num | Length | Payload        | CRC16     | End Marker   |
| 0xAA 0x55    | 1 Byte   | 1 Byte  | 1 Byte | N Bytes        | 2 Bytes   | 0x0D 0x0A    |
+--------------+----------+---------+--------+----------------+-----------+--------------+
```

### Message Definitions
| Message Name | Code | Direction | Payload Length | Description |
|---|:---:|:---:|:---:|---|
| `CMD_JOINTS` | `0x01` | Host $\to$ MCU | 12 Bytes | 6x int16 milliradians ($q_i \times 1000$) |
| `CMD_ESTOP` | `0x02` | Host $\to$ MCU | 1 Byte | `0x01` assert E-stop, `0x00` clear E-stop |
| `CMD_RESET` | `0x03` | Host $\to$ MCU | 0 Bytes | Reset faults and restore ready state |
| `CMD_PING` | `0x04` | Host $\to$ MCU | 0 Bytes | Connectivity test |
| `ACK` | `0x06` | MCU $\to$ Host | 1 Byte | Sequence number acknowledged |
| `NACK` | `0x15` | MCU $\to$ Host | 2 Bytes | Nacked sequence number and error code |
| `TLM_FEEDBACK` | `0x81` | MCU $\to$ Host | 17 Bytes | 6x int16 mrad (12B) + status byte (1B) + timestamp_ms (4B uint32) |
| `TLM_STATUS` | `0x82` | MCU $\to$ Host | 3 Bytes | State (1B) + fault code (1B) + loop_hz (1B) |

### CRC16-CCITT Verification
Polynomial: `0x1021`, Initial value: `0xFFFF`.
Calculated over `[MsgType, SeqNum, Length, ...Payload...]`.
Any packet failing CRC check is immediately rejected.

## 3. Microcontroller Firmware State Machine

```
   +--------+
   |  BOOT  |
   +--------+
        |
        v
   +--------+      CMD_JOINTS      +--------+
   | READY  | -------------------> | MOVING |
   +--------+ <------------------- +--------+
        |         Target Reached        |
        |                               |
        +---------------+---------------+
                        |
                        | CMD_ESTOP (or HW E-stop)
                        v
                  +-----------+
                  |  ESTOPPED |  (Motors actively damped; commands rejected)
                  +-----------+
                        |
                        | CMD_RESET
                        v
                  +-----------+
                  |   READY   |
                  +-----------+
```

