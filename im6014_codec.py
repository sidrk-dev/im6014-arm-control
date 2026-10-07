import math
import struct

RATIO = 38 / 3
RPM_MIN = -32768 / 64 / RATIO * 60
RPM_MAX = 32767 / 64 / RATIO * 60
POSITION_MIN = -2147483648 / 32768 / RATIO * math.tau
POSITION_MAX = 2147483647 / 32768 / RATIO * math.tau


def crc32(data):
    if len(data) % 4:
        raise ValueError('CRC input must contain complete four-byte words')
    crc = 0xFFFFFFFF
    for start in range(0, len(data), 4):
        for byte in data[start:start + 4][::-1]:
            crc ^= byte << 24
            for _ in range(8):
                crc = ((crc << 1) ^ (0x04C11DB7 if crc & 0x80000000 else 0)) & 0xFFFFFFFF
    return crc


def command(motor_id, mode=0, rpm=0, kd=0, torque=0, watchdog=True, position=0, kp=0):
    if not 0 <= motor_id <= 14 or mode not in (0, 1):
        raise ValueError('Individual ID 0–14 and Stop/FOC modes only')
    if not all(math.isfinite(v) for v in (rpm, kd, torque, position, kp)):
        raise ValueError('Nonfinite control input')
    if not RPM_MIN <= rpm <= RPM_MAX:
        raise ValueError('Speed cannot be encoded in the motor protocol; this is not a rated-speed limit')
    if not 0 <= kd <= 6 or not 0 <= kp <= 30 or not -3.5 <= torque <= 3.5:
        raise ValueError('Bench limits: Kd 0–6, Kp 0–30, feedforward torque ±3.5 Nm')
    if not POSITION_MIN <= position <= POSITION_MAX:
        raise ValueError('Position cannot be encoded in the motor protocol')
    position_raw = round(position / math.tau * RATIO * 32768)
    if mode == 0 and (rpm or kd or torque or position or kp) or mode == 1 and not watchdog:
        raise ValueError('Stop must have zero targets; FOC requires watchdog')
    raw = b'\xfe\xee' + bytes([motor_id | mode << 4 | int(watchdog) << 7, 0])
    raw += struct.pack('<hhihh', round(torque / RATIO * 2560),
                       round(rpm / 60 * RATIO * 64), position_raw, round(kp / RATIO ** 2 * 12800),
                       round(kd / RATIO ** 2 * 51200))
    return raw + struct.pack('<I', crc32(raw))


def feedback(raw, motor_id):
    if len(raw) != 26 or raw[:2] != b'\xfc\xee':
        raise RuntimeError('Invalid feedback length/header')
    if struct.unpack('<I', raw[22:])[0] != crc32(raw[2:22]):
        raise RuntimeError('Feedback CRC mismatch')
    m, td, tw, v, tq, sp, pos, err, flags, _, _ = struct.unpack('<BbBBhhiIHBB', raw[2:22])
    if m & 15 != motor_id or (m >> 4) & 7 not in (0, 1):
        raise RuntimeError('Feedback ID/mode mismatch')
    return dict(id=m & 15, mode=(m >> 4) & 7, timeout=m >> 7,
                driver_c=td, winding_c=tw, voltage=v / 2,
                torque_nm=tq / 2560 * RATIO, rpm=sp / 64 / RATIO * 60,
                position_rad=pos / 32768 / RATIO * math.tau,
                errors=err, warnings=(flags >> 13) & 7, raw=raw.hex())
