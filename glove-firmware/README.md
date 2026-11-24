# LiteWing Gesture-Control Glove Firmware

## Overview

This firmware runs on an independently powered **ESP32 DevKit V1** installed on a wearable gesture glove. It communicates directly with the **LiteWing ESP32-S3 Drone** over Wi-Fi UDP on dedicated port `2391`, completely isolated from the official LiteWing mobile application on port `2390`.

---

## Hardware Components & Wiring

| Component | ESP32 Pin | Function | Notes |
|---|---|---|---|
| **MPU6050 VCC** | `3.3V` | Power supply | Clean 3.3V output |
| **MPU6050 GND** | `GND` | Ground | Common system ground |
| **MPU6050 SDA** | `GPIO21` | I²C Data | Standard ESP32 I²C SDA |
| **MPU6050 SCL** | `GPIO22` | I²C Clock | Standard ESP32 I²C SCL |
| **LDR / Flex Sensor** | `GPIO34` | Dead-man switch | ADC1 input with 10kΩ pull-down resistor |
| **Emergency Stop Button** | `GPIO4` | Hardware E-stop | Normally open momentary button to GND (internal pull-up enabled) |
| **Mode Button** | `GPIO5` | Flight mode toggle | Momentary button to GND (toggles manual/altitude assist) |
| **Status LED** | `GPIO2` | Visual indicator | Onboard ESP32 LED |

---

## Safety Features

1. **Dead-Man Switch Safety Interlock:**
   - The LDR / Flex sensor acts as an active hold interlock.
   - When the user's hand is open or the glove is removed, the throttle is **forced to 0** instantly.
   - Props stop immediately upon releasing the hand grip.
2. **Dedicated Emergency Stop Button:**
   - Pressing the E-stop button on `GPIO4` asserts an immediate emergency stop packet.
   - The drone cuts motor power without delay.
3. **App-First Arbitration:**
   - Whenever the official LiteWing mobile app sends a command, the drone switches ownership to the app.
   - The glove cannot override mobile app commands.
4. **Failsafe Timeout:**
   - If Wi-Fi packets from the glove stop for more than 250ms, the drone enters a failsafe level and cuts thrust.

---

## Binary Packet Format (UDP Port 2391)

| Offset | Field | Type | Description |
|---:|---|---|---|
| 0 | `magic0` | `uint8_t` | Magic byte 1 (`0xAA`) |
| 1 | `magic1` | `uint8_t` | Magic byte 2 (`0x55`) |
| 2 | `version` | `uint8_t` | Protocol version (`1`) |
| 3 | `sequence` | `uint8_t` | Rolling counter (0-255) |
| 4-7 | `timestamp_ms` | `uint32_t` | Sender timestamp in ms |
| 8-9 | `roll` | `int16_t` | Roll setpoint in 0.01° units (e.g., 1000 = 10.00°) |
| 10-11 | `pitch` | `int16_t` | Pitch setpoint in 0.01° units |
| 12-13 | `yaw` | `int16_t` | Yaw rate in 0.1°/s units |
| 14-15 | `thrust` | `uint16_t` | Thrust value (0 - 60000) |
| 16 | `deadman` | `uint8_t` | `1` = Hand active, `0` = Cut throttle |
| 17 | `emergency` | `uint8_t` | `1` = E-stop asserted |
| 18 | `mode` | `uint8_t` | `0` = Manual, `1` = Altitude Assist |
| 19 | `crc` | `uint8_t` | XOR checksum of bytes 0 through 18 |

Total packet length: **20 bytes**.

---

## Flashing Instructions

### Using Arduino IDE
1. Install **ESP32 by Espressif** board support in the Arduino Board Manager.
2. Select board: `DOIT ESP32 DEVKIT V1` (or generic `ESP32 Dev Module`).
3. Open `glove_firmware.ino`.
4. Connect the glove ESP32 via micro-USB and click **Upload**.

### Using PlatformIO
```bash
cd glove-firmware
pio run --target upload
```

---

## Operating Instructions

1. Power on the LiteWing drone first. Wait for its blue/green LED sequence.
2. Power on the gesture glove. Keep your hand steady and level for 2 seconds during sensor calibration.
3. The glove LED will flash while searching for `LiteWing_*` SoftAP.
4. Once connected, the glove LED turns **solid ON**.
5. Close your hand / cover the dead-man sensor to enable flight control.
6. Tilt forward/backward to pitch, tilt left/right to roll, twist hand to yaw.
7. Open your hand at any moment to cut thrust instantly.
