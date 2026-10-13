# IM6014 Hardware & Communication Debug Reference

The adapter details in the first two sections describe the historical FT4232H bench adapter. They are not U2D2 evidence. The ordered U2D2 has not arrived; read [U2D2 connection and verification](../u2d2.md) for its intended setup and current verification boundary.

## Historical motor and FT4232H setup

- **Motor Model:** Unitree IM6014 / N6014B-12.6 integrated cycloid actuator.
- **Factory Settings:** ID `0`, baud rate `6,000,000` (6 Mbps), 8N1, default mode `0` (Stop).
- **Reduction Ratio:** Exact ratio $r = 38/3 \approx 12.6667$.
- **Power Supply:** 24–72V DC (bench tested at 36V DC via XT30).
- **USB-to-RS485 Dongle:** Unitree official adapter based on FTDI FT4232H Quad High-Speed USB-UART IC (`VID:PID 0403:6011`, Serial `FTC2RQ7H`).
- **Wiring Chain:** Host PC USB $\rightarrow$ Unitree USB-RS485 adapter $\rightarrow$ 3-pin GH1.25 cable (A, B, GND) $\rightarrow$ XT30(2+2) breakout board $\rightarrow$ IM6014 actuator.

---

## Historical FT4232H port mapping

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

## Linux driver and kernel troubleshooting

The previously suggested ModemManager probe and FTDI flow-control explanations are **unverified hypotheses**, not established causes of a stale or missing reply. Do not use a world-writable (`0666`) device rule or privileged USB sysfs reset commands as a workaround.

If the adapter disappears or feedback becomes stale, keep the motor in Stop, reconnect the adapter, refresh the passive port list, and select the path that matches the inspected cable. Verify motor power and ID 0 before another connection. A listed path alone does not confirm adapter identity or motor communication. Current protocol feedback provides the communication evidence.

The U2D2 guide gives the specific manual `latency_timer` read, write, and read-back steps. Do not apply those steps to an unverified tty.

## Protocol and codec rules

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

## Control tuning and slow rotation parameters

Hybrid control formula:
$$\tau = \tau_{ff} + K_p \times (p_{des} - p) + K_d \times (\omega_{des} - \omega)$$

### Separate protocol and legacy bench velocity recipe

This recipe is for the separate protocol/legacy bench surface. It is not an arm-dashboard setting, and it was not revalidated in the current usability release. The J1 dashboard defaults to 0.5 rpm and applies a 1 rpm maximum through position control; use `docs/arm-quickstart.md` for that flow. The values below must not be treated as its commissioning defaults.
- `mode = 1` (FOC), `timeout = 1` (watchdog active)
- `outputSpd = 1.0` to `2.0` rad/s ($\approx 9.5$ to $19$ RPM)
- `Kd = 1.5` to `3.0` $\text{N}\cdot\text{m}/(\text{rad/s})$ (recommended no-load range $0.5$–$6.0$)
- `Kp = 0.0` (**must be 0**; any position stiffness creates opposing restoring torque)
- `outputTor = 0.0`, `outputPos = 0.0`

### Position Mode (Small Angle Movements)
- `Kp = 15.0` $\text{N}\cdot\text{m}/\text{rad}$, `Kd = 3.0` $\text{N}\cdot\text{m}/(\text{rad/s})$
- Travel limit: $\pm 90^\circ$ maximum, tested safely at $30^\circ$ and $88.5^\circ$.
