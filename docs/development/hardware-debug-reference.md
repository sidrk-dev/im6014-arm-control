# IM6014 Hardware & Communication Debug Reference

## 1. Physical Hardware & Bus Topology

- **Motor Model:** Unitree IM6014 / N6014B-12.6 integrated cycloid actuator.
- **Factory Settings:** ID `0`, baud rate `6,000,000` (6 Mbps), 8N1, default mode `0` (Stop).
- **Reduction Ratio:** Exact ratio $r = 38/3 \approx 12.6667$.
- **Power Supply:** 24–72V DC (bench tested at 36V DC via XT30).
- **USB-to-RS485 Dongle:** Unitree official adapter based on FTDI FT4232H Quad High-Speed USB-UART IC (`VID:PID 0403:6011`, Serial `FTC2RQ7H`).
- **Wiring Chain:** Host PC USB $\rightarrow$ Unitree USB-RS485 adapter $\rightarrow$ 3-pin GH1.25 cable (A, B, GND) $\rightarrow$ XT30(2+2) breakout board $\rightarrow$ IM6014 actuator.

---

## 2. Port Mapping & Interface Discrepancy

The FT4232H chip exposes four hardware channels (Interfaces 0–3). On this board, **only Channel D (Interface 3) is wired to the RS-485 differential transceiver**.

| Channel | FTDI Descriptor Serial | Windows Device & COM Port | Linux Serial Device Node | Purpose on this Dongle |
| :---: | :---: | :---: | :---: | :---: |
| Port A | `FTC2RQ7HA` | Unassigned / not used | `/dev/ttyUSB0` | Unwired / internal UART |
| Port B | `FTC2RQ7HB` | Unassigned / not used | `/dev/ttyUSB1` | Unwired / internal UART |
| Port C | `FTC2RQ7HC` | Unassigned / not used | `/dev/ttyUSB2` | Unwired / internal UART |
| **Port D** | **`FTC2RQ7HD`** | **`COM10`** | **`/dev/ttyUSB3`** | **RS-485 Transceiver (Motor Bus)** |

### Persistent udev Symlink
On Linux, always address the motor through the interface 3 by-id symlink:
```text
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2RQ7H-if03-port0 -> /dev/ttyUSB3
```
Running commands against `/dev/ttyUSB0` will fail to reach the motor.

---

## 3. Linux Driver & Kernel Troubleshooting

### ModemManager Interference
By default on Linux (e.g. Fedora / Ubuntu), `ModemManager` detects newly plugged FTDI serial ports and sends cellular `AT` probe strings at 115200 baud, which can corrupt the motor's packet parser and cause lockouts.

**Fix:** Add `ID_MM_DEVICE_IGNORE` to udev rules (`/etc/udev/rules.d/99-ftdi.rules`):
```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", MODE="0666", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", ENV{ID_MM_DEVICE_IGNORE}="1"
```

### Flow Control & Baudrate Timeout (`-110` / `-71`)
When opening the port using PySerial without explicit flags, the kernel `ftdi_sio` driver attempts to send `FTDI_SIO_SET_FLOW_CTRL_REQUEST` on Endpoint 0. Because Channel D only connects TX/RX (no modem handshake lines), the control transfer can stall:
```text
ftdi_sio ttyUSB3: failed to set flow control: -71 (EPROTO) or -110 (ETIMEDOUT)
```
**Fix in Software:**
Always open the port with `rtscts=False, dsrdtr=False, xonxoff=False`.

**Fix for USB PHY Latch-up:**
When 36V DC power is switched on or cables are connected, ground transients can cause the FT4232H USB PHY to enter an unresponsive state. Because motherboard USB ports do not power-gate 5V VBUS on software resets:
1. Physically unplug the USB adapter for 3 seconds and plug it back in; or
2. Power cycle the port via sysfs:
   ```bash
   echo 1 | sudo tee /sys/bus/usb/devices/usb3/3-0:1.0/usb3-port2/disable
   sleep 1
   echo 0 | sudo tee /sys/bus/usb/devices/usb3/3-0:1.0/usb3-port2/disable
   ```

---

## 4. Protocol & Codec Rules

### Request Frame (20 bytes, Host $\rightarrow$ Motor)
- **Header:** `0xFE 0xEE`
- **Byte 2:** `id | (status << 4) | (timeout << 7)`
  - `status = 0`: Stop mode (unpowered / idle hold)
  - `status = 1`: FOC mode
  - `timeout = 1`: Enable 1-second watchdog (motor halts if no packet arrives for > 1 s)
- **Byte 3:** Reserved (`0x00`)
- **Bytes 4–5:** `tor_des` (int16, $\times 2560$)
- **Bytes 6–7:** `spd_des` (int16, $\times 64 / 2\pi$)
- **Bytes 8–11:** `pos_des` (**signed int32**, $\times 32768 / 2\pi$)  
  *(Note: Official Unitree demo mistakenly packed this as unsigned `I` in `struct.pack('<hhIhh')`, which raises a crash on negative angles).*
- **Bytes 12–13:** `k_pos` (int16, $\times 12800$)
- **Bytes 14–15:** `k_spd` (int16, $\times 51200$)
- **Bytes 16–19:** CRC32-MPEG2 (little-endian word order, polynomial `0x04C11DB7`, init `0xFFFFFFFF`)

### Response Frame (26 bytes, Motor $\rightarrow$ Host)
- **Header:** `0xFC 0xEE`
- **Byte 2:** `id | (status << 4) | (timeout << 7)`
- **Byte 3:** Driver temperature (signed int8, ℃)
- **Byte 4:** Winding temperature (**unsigned uint8**, ℃)
- **Byte 5:** Terminal voltage (**unsigned uint8**, raw / 2.0 = Volts)
- **Bytes 6–7:** Measured rotor torque (int16, / 2560 Nm)
- **Bytes 8–9:** Measured rotor speed (int16, / 64 $\times 2\pi$ rad/s)
- **Bytes 10–13:** Measured rotor position (signed int32, / 32768 $\times 2\pi$ rad)
- **Bytes 14–17:** `MError` (uint32 bitmask)
- **Bytes 18–19:** `MWarning` (bits 13–15 of uint16)
- **Bytes 20–21:** Reserved
- **Bytes 22–25:** CRC32-MPEG2 (covers bytes 2–21)

---

## 5. Control Tuning & Slow Rotation Parameters

Hybrid control formula:
$$\tau = \tau_{ff} + K_p \times (p_{des} - p) + K_d \times (\omega_{des} - \omega)$$

### Velocity Mode (Spinning Slowly)
- `mode = 1` (FOC), `timeout = 1` (watchdog active)
- `outputSpd = 1.0` to `2.0` rad/s ($\approx 9.5$ to $19$ RPM)
- `Kd = 1.5` to `3.0` $\text{N}\cdot\text{m}/(\text{rad/s})$ (recommended no-load range $0.5$–$6.0$)
- `Kp = 0.0` (**must be 0**; any position stiffness creates opposing restoring torque)
- `outputTor = 0.0`, `outputPos = 0.0`

### Position Mode (Small Angle Movements)
- `Kp = 15.0` $\text{N}\cdot\text{m}/\text{rad}$, `Kd = 3.0` $\text{N}\cdot\text{m}/(\text{rad/s})$
- Travel limit: $\pm 90^\circ$ maximum, tested safely at $30^\circ$ and $88.5^\circ$.
