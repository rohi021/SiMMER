# Wiring Guide — 2× PCA9685 + Raspberry Pi 4

This document provides complete wiring instructions for connecting two PCA9685 16-channel PWM servo driver boards to a Raspberry Pi 4 for the SiMMER hexapod robot.

---

## Table of Contents

- [Components Required](#components-required)
- [Understanding the PCA9685](#understanding-the-pca9685)
- [I2C Address Configuration](#i2c-address-configuration)
- [Raspberry Pi 4 GPIO Pinout (I2C)](#raspberry-pi-4-gpio-pinout-i2c)
- [Wiring Diagram — I2C Bus](#wiring-diagram--i2c-bus)
- [Power Supply Wiring](#power-supply-wiring)
- [Servo-to-Channel Mapping](#servo-to-channel-mapping)
- [Step-by-Step Wiring Procedure](#step-by-step-wiring-procedure)
- [Verification and Testing](#verification-and-testing)
- [Troubleshooting](#troubleshooting)
- [Safety Notes](#safety-notes)

---

## Components Required

| Component | Quantity | Notes |
|---|---|---|
| Raspberry Pi 4 | 1 | Any RAM variant (2/4/8 GB) |
| PCA9685 16-ch PWM board | 2 | Adafruit or compatible breakout |
| Servo motors | 21 | 18 for legs + 3 for manipulator |
| Jumper wires (female-female) | 8 | For I2C + power connections |
| External 5–6 V servo power supply | 1–2 | Must supply sufficient current for all servos |
| Soldering iron + solder | — | For address jumper on Board 2 |
| Breadboard or terminal block | Optional | For power distribution |

---

## Understanding the PCA9685

Each PCA9685 board provides:

- **16 PWM output channels** (numbered 0–15) for servo control
- **I2C interface** (SDA + SCL) for communication with the Raspberry Pi
- **V+** servo power rail — supplies power to all connected servos on that board
- **VCC** logic power (3.3 V or 5 V from the Pi)
- **Configurable I2C address** via solder jumpers (A0–A5)

Key facts:
- Default I2C address: **0x40**
- The address can be changed by bridging solder pads A0–A5 on the board
- Multiple PCA9685 boards can share a single I2C bus — each board needs a **unique address**
- The `i2cpwm_board` ROS package addresses servos sequentially: servos 1–16 map to the board at 0x40, servos 17–32 map to the board at 0x41, and so on

---

## I2C Address Configuration

The two boards must have **different I2C addresses** on the shared bus.

| Board | I2C Address | Address Jumpers | Role in SiMMER |
|---|---|---|---|
| **Board 1** | `0x40` | None bridged (default) | Legs 1, 3, 5 + manipulator joint m_2 |
| **Board 2** | `0x41` | **Bridge A0** only | Legs 2, 4, 6 + manipulator joints m_1, m_3 |

### How to set the address on Board 2

On the PCA9685 breakout board, locate the **A0** solder jumper pad (usually near the top-right of the board). Apply a small solder bridge across the A0 pad to connect the two sides. This changes the address from `0x40` to `0x41`.

```
Board 2 Address Jumpers:
  A0  A1  A2  A3  A4  A5
  [■]  ○   ○   ○   ○   ○     ← Bridge A0 only

  ■ = bridged (soldered)
  ○ = open (default)
```

> **Why A0?** Each jumper corresponds to one bit in the address. The base address is `0x40` (binary `1000000`). Bridging A0 sets the least significant bit, giving `0x41` (binary `1000001`).

---

## Raspberry Pi 4 GPIO Pinout (I2C)

The Raspberry Pi 4 uses **I2C bus 1** on the following GPIO pins:

```
Raspberry Pi 4 — 40-Pin GPIO Header (relevant pins)
┌─────────────────────────────────────────┐
│  Pin 1  [3V3 Power]    [5V Power] Pin 2 │
│  Pin 3  [GPIO 2/SDA1]  [5V Power] Pin 4 │
│  Pin 5  [GPIO 3/SCL1]  [GND]     Pin 6 │
│  ...                                    │
│  Pin 9  [GND]                           │
│  ...                                    │
│  Pin 14 [GND]                           │
│  ...                                    │
└─────────────────────────────────────────┘
```

| Function | GPIO | Pin # | Wire Color (suggested) |
|---|---|---|---|
| **SDA** (I2C data) | GPIO 2 | Pin 3 | Blue |
| **SCL** (I2C clock) | GPIO 3 | Pin 5 | Yellow |
| **3.3 V** (logic power) | — | Pin 1 | Red |
| **GND** (ground) | — | Pin 6, 9, or 14 | Black |

---

## Wiring Diagram — I2C Bus

Both PCA9685 boards connect to the **same I2C bus** in parallel (daisy-chain or star topology).

```
                    ┌──────────────────────┐
                    │   Raspberry Pi 4     │
                    │                      │
                    │  Pin 3 (SDA) ────────┼──── SDA bus
                    │  Pin 5 (SCL) ────────┼──── SCL bus
                    │  Pin 1 (3V3) ────────┼──── VCC bus (logic)
                    │  Pin 6 (GND) ────────┼──── GND bus
                    └──────────────────────┘
                                │
                 ┌──────────────┴──────────────┐
                 │                             │
    ┌────────────▼────────────┐   ┌────────────▼────────────┐
    │   PCA9685 — Board 1     │   │   PCA9685 — Board 2     │
    │   Address: 0x40         │   │   Address: 0x41         │
    │                         │   │   (A0 bridged)          │
    │  SDA ◄── SDA bus        │   │  SDA ◄── SDA bus        │
    │  SCL ◄── SCL bus        │   │  SCL ◄── SCL bus        │
    │  VCC ◄── 3V3 bus        │   │  VCC ◄── 3V3 bus        │
    │  GND ◄── GND bus        │   │  GND ◄── GND bus        │
    │                         │   │                         │
    │  V+ ◄── Servo PSU (+)   │   │  V+ ◄── Servo PSU (+)  │
    │  GND ◄── Servo PSU (−)  │   │  GND ◄── Servo PSU (−) │
    │                         │   │                         │
    │  Ch 0–15: Servos        │   │  Ch 0–15: Servos        │
    └─────────────────────────┘   └─────────────────────────┘
```

### Option A — Daisy Chain (recommended)

The PCA9685 breakout boards have both an input header and an output header. Wire the Pi to Board 1's input, then run Board 1's output to Board 2's input:

```
Raspberry Pi  ──►  Board 1 (input)  ──►  Board 1 (output)  ──►  Board 2 (input)
   SDA (Pin 3) ────► SDA in ───────────── SDA out ──────────────► SDA in
   SCL (Pin 5) ────► SCL in ───────────── SCL out ──────────────► SCL in
   3V3 (Pin 1) ────► VCC    ───────────── VCC     ──────────────► VCC
   GND (Pin 6) ────► GND    ───────────── GND     ──────────────► GND
```

> The PCA9685 boards have built-in passthrough headers specifically for daisy-chaining. No Y-splitters or breadboard needed.

### Option B — Star Topology

If you prefer direct connections, wire both boards independently to the Pi GPIO header (sharing the same SDA, SCL, 3V3, GND pins). Use a small breadboard or terminal block to split the signals.

---

## Power Supply Wiring

> ⚠️ **CRITICAL**: Servos must be powered by an external power supply — **never** from the Raspberry Pi's 5 V pins. The Pi cannot supply enough current for 21 servos.

### Servo Power (V+ rail)

| Connection | From | To |
|---|---|---|
| Servo power (+) | External PSU (+) terminal | Board 1 **V+** screw terminal |
| Servo power (+) | External PSU (+) terminal | Board 2 **V+** screw terminal |
| Servo power (−) | External PSU (−) terminal | Board 1 **GND** screw terminal |
| Servo power (−) | External PSU (−) terminal | Board 2 **GND** screw terminal |

### Power Supply Requirements

| Parameter | Recommendation |
|---|---|
| Voltage | 5.0–6.0 V (match your servo rated voltage) |
| Current capacity | ≥ 15 A (21 servos × ~0.5–0.7 A each under load) |
| Type | Regulated switching PSU or LiPo battery with BEC |

### Common Ground

All grounds must be connected together:

```
Raspberry Pi GND ──── Board 1 GND ──── Board 2 GND ──── Servo PSU (−)
```

This shared ground is essential for reliable I2C communication. Without it, the data signals will have no common reference and communication will fail.

### Logic Power (VCC)

The PCA9685 VCC pin accepts 3.3 V or 5 V logic power. Connect it to the Raspberry Pi's **3.3 V (Pin 1)**. This powers only the PCA9685 logic chip — it does NOT power the servos.

---

## Servo-to-Channel Mapping

The `i2cpwm_board` ROS package uses a sequential numbering scheme:

- **Servos 1–16** → Board 1 (0x40), channels 0–15
- **Servos 17–32** → Board 2 (0x41), channels 0–15

The formula is: `channel = (servo_number - 1) % 16`, `board = (servo_number - 1) / 16 + 1`

### Board 1 — Address 0x40

Drives Legs 1, 3, 5 and manipulator joint m_2.

| Channel | Servo # | Joint | Robot Part | Description |
|---|---|---|---|---|
| 0 | 1 | `joint_1_1` | Leg 1 | Coxa (hip yaw) |
| 1 | 2 | `joint_1_2` | Leg 1 | Femur (hip pitch) |
| 2 | 3 | `joint_1_3` | Leg 1 | Tibia (knee) |
| 3 | — | — | — | *Unused* |
| 4 | 5 | `joint_3_1` | Leg 3 | Coxa (hip yaw) |
| 5 | 6 | `joint_3_2` | Leg 3 | Femur (hip pitch) |
| 6 | 7 | `joint_3_3` | Leg 3 | Tibia (knee) |
| 7 | — | — | — | *Unused* |
| 8 | 9 | `joint_5_1` | Leg 5 | Coxa (hip yaw) |
| 9 | 10 | `joint_5_2` | Leg 5 | Femur (hip pitch) |
| 10 | 11 | `joint_5_3` | Leg 5 | Tibia (knee) |
| 11 | — | — | — | *Unused* |
| 12 | 13 | `joint_m_2` | Manipulator | Joint 2 (shoulder pitch) |
| 13 | — | — | — | *Unused* |
| 14 | — | — | — | *Unused* |
| 15 | — | — | — | *Unused* |

### Board 2 — Address 0x41

Drives Legs 2, 4, 6 and manipulator joints m_1, m_3.

| Channel | Servo # | Joint | Robot Part | Description |
|---|---|---|---|---|
| 0 | 17 | `joint_2_1` | Leg 2 | Coxa (hip yaw) |
| 1 | 18 | `joint_2_2` | Leg 2 | Femur (hip pitch) |
| 2 | 19 | `joint_2_3` | Leg 2 | Tibia (knee) |
| 3 | — | — | — | *Unused* |
| 4 | 21 | `joint_4_1` | Leg 4 | Coxa (hip yaw) |
| 5 | 22 | `joint_4_2` | Leg 4 | Femur (hip pitch) |
| 6 | 23 | `joint_4_3` | Leg 4 | Tibia (knee) |
| 7 | — | — | — | *Unused* |
| 8 | 25 | `joint_6_1` | Leg 6 | Coxa (hip yaw) |
| 9 | 26 | `joint_6_2` | Leg 6 | Femur (hip pitch) |
| 10 | 27 | `joint_6_3` | Leg 6 | Tibia (knee) |
| 11 | — | — | — | *Unused* |
| 12 | 29 | `joint_m_1` | Manipulator | Joint 1 (base rotation) |
| 13 | 30 | `joint_m_3` | Manipulator | Joint 3 (elbow) |
| 14 | — | — | — | *Unused* |
| 15 | — | — | — | *Unused* |

> **Channels 3, 7, 11 are intentionally left empty** on both boards. This groups each leg's 3 servos into contiguous blocks of 4 channels, making the wiring layout clean and consistent.

---

## Step-by-Step Wiring Procedure

### Step 1 — Set the I2C Address on Board 2

1. Take the PCA9685 board designated as **Board 2**.
2. Locate the **A0** solder jumper on the PCB (near the address jumper block).
3. Apply a small solder bridge across A0.
4. Leave **Board 1** unchanged (default address 0x40).

### Step 2 — Connect the I2C Bus

Using the daisy-chain method:

1. Connect **Raspberry Pi Pin 3 (SDA)** → Board 1 **SDA** (input side).
2. Connect **Raspberry Pi Pin 5 (SCL)** → Board 1 **SCL** (input side).
3. Connect **Raspberry Pi Pin 1 (3V3)** → Board 1 **VCC** (input side).
4. Connect **Raspberry Pi Pin 6 (GND)** → Board 1 **GND** (input side).
5. Connect Board 1 **SDA** (output side) → Board 2 **SDA** (input side).
6. Connect Board 1 **SCL** (output side) → Board 2 **SCL** (input side).
7. Connect Board 1 **VCC** (output side) → Board 2 **VCC** (input side).
8. Connect Board 1 **GND** (output side) → Board 2 **GND** (input side).

### Step 3 — Connect the External Servo Power Supply

1. Connect the **positive (+)** terminal of the servo power supply to the **V+** screw terminal on **Board 1**.
2. Connect the **positive (+)** terminal of the servo power supply to the **V+** screw terminal on **Board 2**.
3. Connect the **negative (−)** terminal of the servo power supply to the **GND** screw terminal on **Board 1**.
4. Connect the **negative (−)** terminal of the servo power supply to the **GND** screw terminal on **Board 2**.

> If using separate power supplies for each board, ensure all GND connections are still tied together (common ground with the Pi).

### Step 4 — Connect Servos to Board 1 (0x40)

Plug each servo's 3-pin connector (signal, V+, GND) into the corresponding channel header on Board 1. Match the orientation — usually the signal wire (white or orange) goes to the pin closest to the board edge, and the dark wire (brown or black) goes to the GND pin.

| Channel | Servo Wire → | Joint |
|---|---|---|
| 0 | Leg 1 coxa servo | `joint_1_1` |
| 1 | Leg 1 femur servo | `joint_1_2` |
| 2 | Leg 1 tibia servo | `joint_1_3` |
| 4 | Leg 3 coxa servo | `joint_3_1` |
| 5 | Leg 3 femur servo | `joint_3_2` |
| 6 | Leg 3 tibia servo | `joint_3_3` |
| 8 | Leg 5 coxa servo | `joint_5_1` |
| 9 | Leg 5 femur servo | `joint_5_2` |
| 10 | Leg 5 tibia servo | `joint_5_3` |
| 12 | Manipulator joint 2 servo | `joint_m_2` |

### Step 5 — Connect Servos to Board 2 (0x41)

| Channel | Servo Wire → | Joint |
|---|---|---|
| 0 | Leg 2 coxa servo | `joint_2_1` |
| 1 | Leg 2 femur servo | `joint_2_2` |
| 2 | Leg 2 tibia servo | `joint_2_3` |
| 4 | Leg 4 coxa servo | `joint_4_1` |
| 5 | Leg 4 femur servo | `joint_4_2` |
| 6 | Leg 4 tibia servo | `joint_4_3` |
| 8 | Leg 6 coxa servo | `joint_6_1` |
| 9 | Leg 6 femur servo | `joint_6_2` |
| 10 | Leg 6 tibia servo | `joint_6_3` |
| 12 | Manipulator joint 1 servo | `joint_m_1` |
| 13 | Manipulator joint 3 servo | `joint_m_3` |

### Step 6 — Double-Check All Connections

Before powering on:

- [ ] Board 1 address = 0x40 (no jumpers bridged)
- [ ] Board 2 address = 0x41 (A0 bridged)
- [ ] SDA, SCL, VCC, GND all connected between Pi and both boards
- [ ] External servo power supply connected to V+ and GND on both boards
- [ ] All servo connectors oriented correctly (signal, V+, GND)
- [ ] Common ground shared between Pi, both boards, and servo PSU
- [ ] No servo power supply connected to the Pi's 5 V or 3.3 V pins

---

## Verification and Testing

### 1. Enable I2C on the Raspberry Pi

```bash
sudo raspi-config
# Navigate to: Interface Options → I2C → Enable
sudo reboot
```

### 2. Detect the Boards

```bash
sudo apt-get install -y i2c-tools
i2cdetect -y 1
```

Expected output:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: 40 41 -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

Both `40` and `41` should appear. If either is missing:
- Check the solder bridge on Board 2 (address jumper A0)
- Verify all 4 wires (SDA, SCL, VCC, GND) are connected
- Ensure the Pi's I2C interface is enabled

### 3. Test with the ROS i2cpwm_board Node

```bash
# Terminal 1 — start roscore
roscore

# Terminal 2 — start the i2cpwm_board node
rosrun i2cpwm_board i2cpwm_board

# Terminal 3 — send a test command (move servo 1 to center)
rostopic pub -1 /servos_absolute i2cpwm_board/ServoArray \
  "{servos: [{servo: 1, value: 303}]}"
```

The servo on Board 1, channel 0 should move to its center position.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `i2cdetect` shows nothing | I2C not enabled, or wiring error | Run `raspi-config` to enable I2C; check SDA/SCL wires |
| Only `0x40` shows | Board 2 address not set | Solder the A0 jumper on Board 2 |
| `0x40` and `0x41` both show `0x40` | A0 jumper not properly bridged | Re-solder A0 with a clean bridge |
| Servos jitter or don't move | Insufficient power supply current | Use a beefier servo PSU (≥ 15 A at 5–6 V) |
| Servos on Board 2 don't respond | Wrong servo numbers in config | Verify servo numbers 17–32 for Board 2 in `joints.yaml` |
| I2C errors in `dmesg` | Missing pull-up resistors or long wires | Keep I2C wires under 30 cm; PCA9685 has built-in pull-ups |
| Pi freezes or reboots | Servo power fed into Pi GPIO | Ensure V+ is only from external PSU, not from Pi pins |
| Random servo glitches | Ground loop or missing common ground | Connect all GND together: Pi + Board 1 + Board 2 + PSU |

---

## Safety Notes

1. **Never power servos from the Raspberry Pi** — The Pi's 5 V rail can supply ~1 A total. A single servo under load can draw 0.5–1 A. Attempting to power multiple servos from the Pi will cause brownouts, data corruption, or permanent damage.

2. **Always connect the external power supply ground to the Pi's ground** — Without a common ground reference, I2C signals cannot be interpreted correctly and may cause erratic behavior or damage the I2C bus.

3. **Connect all wires before applying power** — Hot-plugging I2C devices while the bus is active can cause address conflicts or lock up the bus.

4. **Verify servo connector orientation** — Reversing the 3-pin servo connector can damage the servo or the PCA9685. Signal (white/orange) faces the board edge on most PCA9685 breakouts.

5. **Use appropriate wire gauges** — The I2C signal wires (SDA, SCL) can be thin (28 AWG), but the power wires to V+ must be thick enough for the total servo current (16 AWG or thicker for 15+ A).

6. **Keep I2C wires short** — I2C is designed for board-to-board communication. Keep wire lengths under 30 cm for reliable operation at default speeds. If longer runs are needed, reduce the I2C clock speed.

---

## Quick-Reference Wiring Summary

```
┌─────────────────┐          ┌─────────────────────────────┐
│ Raspberry Pi 4  │          │  PCA9685 Board 1 (0x40)     │
│                 │          │                             │
│  Pin 3 (SDA)  ──┼─────────►  SDA                        │
│  Pin 5 (SCL)  ──┼─────────►  SCL                        │
│  Pin 1 (3V3)  ──┼─────────►  VCC                        │
│  Pin 6 (GND)  ──┼─────────►  GND                        │
│                 │          │                             │
│                 │          │  V+ ◄──── Servo PSU (+)     │
│                 │          │  GND ◄─── Servo PSU (−)     │
│                 │          │                             │
│                 │          │  Ch 0:  Leg 1 coxa          │
│                 │          │  Ch 1:  Leg 1 femur         │
│                 │          │  Ch 2:  Leg 1 tibia         │
│                 │          │  Ch 4:  Leg 3 coxa          │
│                 │          │  Ch 5:  Leg 3 femur         │
│                 │          │  Ch 6:  Leg 3 tibia         │
│                 │          │  Ch 8:  Leg 5 coxa          │
│                 │          │  Ch 9:  Leg 5 femur         │
│                 │          │  Ch 10: Leg 5 tibia         │
│                 │          │  Ch 12: Manipulator j2      │
│                 │          │                             │
│                 │          │  SDA out ──┐                │
│                 │          │  SCL out ──┤                │
│                 │          │  VCC out ──┤                │
│                 │          │  GND out ──┤                │
└─────────────────┘          └────────────┼────────────────┘
                                          │
                             ┌────────────▼────────────────┐
                             │  PCA9685 Board 2 (0x41)     │
                             │  (A0 jumper bridged)        │
                             │                             │
                             │  SDA ◄── from Board 1       │
                             │  SCL ◄── from Board 1       │
                             │  VCC ◄── from Board 1       │
                             │  GND ◄── from Board 1       │
                             │                             │
                             │  V+ ◄──── Servo PSU (+)     │
                             │  GND ◄─── Servo PSU (−)     │
                             │                             │
                             │  Ch 0:  Leg 2 coxa          │
                             │  Ch 1:  Leg 2 femur         │
                             │  Ch 2:  Leg 2 tibia         │
                             │  Ch 4:  Leg 4 coxa          │
                             │  Ch 5:  Leg 4 femur         │
                             │  Ch 6:  Leg 4 tibia         │
                             │  Ch 8:  Leg 6 coxa          │
                             │  Ch 9:  Leg 6 femur         │
                             │  Ch 10: Leg 6 tibia         │
                             │  Ch 12: Manipulator j1      │
                             │  Ch 13: Manipulator j3      │
                             └─────────────────────────────┘
```

---

*This wiring guide corresponds to the servo configuration in [`lmm_sc/config/joints.yaml`](../lmm_sc/config/joints.yaml). If you modify the channel assignments, update `joints.yaml` accordingly.*
