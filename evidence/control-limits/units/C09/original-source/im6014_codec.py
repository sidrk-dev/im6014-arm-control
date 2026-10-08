import math
import struct

RATIO = 38 / 3
RPM_MIN = -32768 / 64 / RATIO * 60
RPM_MAX = 32767 / 64 / RATIO * 60
POSITION_MIN = -2147483648 / 32768 / RATIO * math.tau
POSITION_MAX = 2147483647 / 32768 / RATIO * math.tau
RAW_POSITION_MIN = -2147483648
RAW_POSITION_MAX = 2147483647


def raw_to_position(raw_position):
    if type(raw_position) is not int or not RAW_POSITION_MIN <= raw_position <= RAW_POSITION_MAX:
        raise ValueError('raw_position must be a signed 32-bit integer')
    return raw_position / 32768 / RATIO * math.tau


def position_to_raw(position):
    position = _finite_control(position, 'Position')
    return _signed_raw(position / math.tau * RATIO * 32768,
                       RAW_POSITION_MIN, RAW_POSITION_MAX, 'Position')


def _motor_id(value, name='Individual ID'):
    if type(value) is not int or not 0 <= value <= 14:
        raise ValueError(f'{name} must be an integer from 0 to 14')
    return value


def _finite_control(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite int or float')
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise ValueError(f'{name} must be finite') from error
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def _signed_raw(value, minimum, maximum, name):
    raw = round(value)
    if not minimum <= raw <= maximum:
        raise ValueError(f'{name} cannot be represented by the motor protocol')
    return raw


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
    motor_id = _motor_id(motor_id)
    if type(mode) is not int or mode not in (0, 1):
        raise ValueError('Individual ID 0–14 and Stop/FOC modes only')
    if type(watchdog) is not bool:
        raise ValueError('Watchdog must be a bool')
    rpm, kd, torque, position, kp = (
        _finite_control(value, name) for value, name in
        ((rpm, 'Speed'), (kd, 'Kd'), (torque, 'Feedforward torque'),
         (position, 'Position'), (kp, 'Kp'))
    )
    if not RPM_MIN <= rpm <= RPM_MAX:
        raise ValueError('Speed cannot be encoded in the motor protocol; this is not a rated-speed limit')
    if not 0 <= kd <= 6 or not 0 <= kp <= 30 or not -3.5 <= torque <= 3.5:
        raise ValueError('Bench limits: Kd 0–6, Kp 0–30, feedforward torque ±3.5 Nm')
    if not POSITION_MIN <= position <= POSITION_MAX:
        raise ValueError('Position cannot be encoded in the motor protocol')
    torque_raw = _signed_raw(torque / RATIO * 2560, -32768, 32767, 'Feedforward torque')
    rpm_raw = _signed_raw(rpm / 60 * RATIO * 64, -32768, 32767, 'Speed')
    position_raw = _signed_raw(position / math.tau * RATIO * 32768,
                               -2147483648, 2147483647, 'Position')
    kp_raw = _signed_raw(kp / RATIO ** 2 * 12800, -32768, 32767, 'Kp')
    kd_raw = _signed_raw(kd / RATIO ** 2 * 51200, -32768, 32767, 'Kd')
    if mode == 0 and (rpm or kd or torque or position or kp) or mode == 1 and not watchdog:
        raise ValueError('Stop must have zero targets; FOC requires watchdog')
    raw = b'\xfe\xee' + bytes([motor_id | mode << 4 | int(watchdog) << 7, 0])
    raw += struct.pack('<hhihh', torque_raw, rpm_raw, position_raw, kp_raw, kd_raw)
    return raw + struct.pack('<I', crc32(raw))


def feedback(raw, motor_id):
    motor_id = _motor_id(motor_id, 'Expected motor ID')
    if not isinstance(raw, (bytes, bytearray, memoryview)):
        raise ValueError('Feedback frame must be a bytes-like buffer')
    try:
        raw = bytes(raw)
    except (TypeError, ValueError) as error:
        raise ValueError('Feedback frame must be a bytes-like buffer') from error
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


def decode_command(raw):
    if not isinstance(raw, (bytes, bytearray, memoryview)):
        raise ValueError('Command frame must be a bytes-like buffer')
    try:
        raw = bytes(raw)
    except (TypeError, ValueError) as error:
        raise ValueError('Command frame must be a bytes-like buffer') from error
    if len(raw) != 20 or raw[:2] != b'\xfe\xee':
        raise RuntimeError('Invalid command length/header')
    if struct.unpack('<I', raw[16:])[0] != crc32(raw[:16]):
        raise RuntimeError('Command CRC mismatch')
    m = raw[2]
    motor_id = m & 0x0F
    mode = (m >> 4) & 0x07
    watchdog = bool(m & 0x80)
    tq, sp, pos, kp, kd = struct.unpack('<hhihh', raw[4:16])
    return dict(id=motor_id, mode=mode, watchdog=watchdog,
                torque_raw=tq, rpm_raw=sp, position_raw=pos,
                kp_raw=kp, kd_raw=kd,
                torque_nm=tq / 2560 * RATIO, rpm=sp / 64 / RATIO * 60,
                position_rad=pos / 32768 / RATIO * math.tau,
                kp=kp * (RATIO ** 2) / 12800,
                kd=kd * (RATIO ** 2) / 51200,
                raw=raw.hex())
