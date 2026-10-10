"""Single-owner, single-motor J1 manual authority.

The owner thread is the only code that opens, writes, reads, or closes the
serial connection. HTTP and input threads may only validate bounded requests,
update authority under the lock, and wake that thread.
"""

from collections import OrderedDict, deque
import math
import queue
import secrets
import struct
import threading
import time

from im6014_codec import (
    POSITION_MAX,
    POSITION_MIN,
    RATIO,
    _kd_raw,
    _kp_raw,
    command,
    command_with_raw_position,
    decode_command,
    position_to_raw,
    raw_to_position,
)
from im6014_port import PortConfig, inspect_port, open_managed_port
from im6014_serial import (
    PacketCapture,
    WriteCancelledError,
    capture_exchange,
    exchange,
)
from im6014_transport import WriteResult
from spin_5rpm import PORT, healthy

MAX_SESSIONS = 16
MAX_QUEUE = 64
MAX_RECEIPTS_PER_SESSION = 256
MAX_EVENTS = 100
MAX_TICKETS = 64
MAX_PROTECTION_HISTORY = 100
MAX_REFERENCE_HISTORY = 256

TICKET_EXPIRY_S = 0.250
INGRESS_LEASE_S = 0.250
FEEDBACK_MAX_AGE_S = 0.100
SCHEDULE_GAP_MAX_S = 0.080
PER_STEP_DT_MAX_S = 0.020

MAX_ACCEL_RAD_S2 = 0.3
MAX_RATE_RPM = 1.0
MAX_MEASURED_SPEED_RPM = 1.5
MAX_MEASURED_LEAD_RAD = 0.15
DEFAULT_EFFORT_CEILING_NM = 1.0

DEFAULT_SETTINGS = {
    'kp': 15.0,
    'kd': 3.0,
    'max_rpm': 0.5,
    'effort_limit_nm': 1.0,
    'known_angle_deg': 0.0,
    'direction': 1,
    'lower_deg': -90.0,
    'upper_deg': 90.0,
}

_ACTIONS = frozenset((
    'connect', 'disconnect', 'check', 'ack', 'calibrate', 'configure', 'arm', 'release'
))


def _finite_number(value, name):
    if type(value) not in (int, float):
        raise ValueError(f'{name} must be a finite number')
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise ValueError(f'{name} must be a finite number') from error
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


class DemoSerial:
    """Packet-level simulator; all calls still run through the owner thread."""

    is_open = True
    in_waiting = 26
    timeout = 0.005
    write_timeout = 0.05

    def __init__(self, initial_pos=0.0, clock=time.monotonic):
        self.clock = clock
        self.data = b''
        self.states = {
            0: dict(
                mode=0, timeout=1, rpm=0.0, pos=float(initial_pos), torque=0.0,
                voltage=48.0, driver_c=35, winding_c=31, errors=0, warnings=0,
            )
        }
        self.previous = self._sim_time()
        self.inject = None
        self.fail_close = False

    def _sim_time(self):
        try:
            value = self.clock()
            if type(value) not in (int, float) or not math.isfinite(float(value)):
                raise ValueError('invalid simulation clock')
            return float(value)
        except Exception:



            return time.monotonic()

    def reset_input_buffer(self):
        self.data = b''

    def write(self, packet):
        now = self._sim_time()
        dt = min(0.1, max(1e-4, now - self.previous))
        self.previous = now
        motor_id, mode, wd = packet[2] & 15, (packet[2] >> 4) & 7, packet[2] >> 7
        if motor_id not in self.states or self.inject == 'timeout':
            return len(packet)
        state = self.states[motor_id]
        old_mode, old_timeout = state['mode'], state['timeout']
        torque, speed, position, kp, kd = struct.unpack('<hhihh', packet[4:16])
        state['mode'] = mode
        if not wd and mode == 0:
            state['timeout'] = 0
        target = speed / 64 / RATIO * 60
        if not mode:
            state['rpm'] = 0.0
            state['torque'] = 0.0
        elif not state['timeout']:
            if kp:
                target_pos = position / 32768 / RATIO * math.tau
                error = target_pos - state['pos']
                state['torque'] = (
                    kp / 12800 * RATIO ** 2 * error
                    - kd / 51200 * RATIO ** 2 * state['rpm'] * math.tau / 60
                )
                state['rpm'] += state['torque'] / 0.05 * dt * 60 / math.tau
                state['pos'] += (target_pos - state['pos']) * min(1.0, dt * 15)
                if abs(state['pos'] - target_pos) > 0.005:
                    state['pos'] = target_pos - 0.002 * math.copysign(1, target_pos - state['pos'])
                state['torque'] = min(0.2, max(-0.2, state['torque']))
            elif kd:
                state['rpm'] += (target - state['rpm']) * min(1.0, dt * 12)
            else:
                state['rpm'] += torque / 2560 * RATIO * dt * 2
            if not kp:
                state['torque'] = torque / 2560 * RATIO + kd / 51200 * RATIO ** 2 * (
                    target - state['rpm']
                ) * math.tau / 60
        state['pos'] += state['rpm'] * math.tau / 60 * dt
        if self.inject == 'overload':
            state['torque'] = 2.5

        body = struct.pack(
            '<BbBBhhiIHBB',
            motor_id | old_mode << 4 | old_timeout << 7,
            state['driver_c'], state['winding_c'], round(state['voltage'] * 2),
            round(state['torque'] / RATIO * 2560),
            round(state['rpm'] / 60 * RATIO * 64),
            round(state['pos'] / math.tau * RATIO * 32768),
            1 if self.inject == 'fault' else state['errors'],
            state['warnings'], 0, 0,
        )
        from im6014_codec import crc32
        self.data = b'\xfc\xee' + body + struct.pack('<I', crc32(body))
        if self.inject == 'crc':
            self.data = self.data[:-1] + bytes([self.data[-1] ^ 1])
        return len(packet)

    def read(self, size):
        data, self.data = self.data[:size], self.data[size:]
        if not data:
            time.sleep(min(0.001, self.timeout) if self.timeout is not None else 0.001)
        return data

    def close(self):
        if self.fail_close:
            raise OSError('Injected serial close failure')
        self.is_open = False


class ArmManualSession:
    """One serialized authority for J1, with bounded session and ticket state."""

    def __init__(self, physical_opt_in=False, serial_factory=None, clock=time.monotonic):
        if type(physical_opt_in) is not bool:
            raise ValueError('physical_opt_in must be an exact bool')
        if serial_factory is not None and not callable(serial_factory):
            raise ValueError('serial_factory must be callable')
        if not callable(clock):
            raise ValueError('clock must be callable')
        self.physical_opt_in = physical_opt_in
        self.serial_factory = serial_factory
        self.clock = clock
        self._started_host_s = time.monotonic()

        self.lock = threading.RLock()
        self.jobs = queue.Queue(maxsize=MAX_QUEUE)
        self.stop_requested = threading.Event()
        self.stop_work = threading.Event()
        self.closing = threading.Event()
        self.close_requested = threading.Event()
        self.wake = threading.Event()
        self.closed = threading.Event()
        self.generation = 0
        self._close_requests = 0

        self.bus = None
        self.port_owner = None
        self.sessions = set()
        self.parameter_drafts = {}
        self.receipts = {}
        self._pending_by_session = {}
        self.owner = None
        self.tickets = OrderedDict()
        self.last_seq = -1
        self.current_device = None

        self.calibrated = False
        self.q_zero = 0.0
        self.armed = False
        self.stop_confirmed = False
        self.stop_latched = True
        self.deadman_active = False
        self.require_neutral = True
        self.target_axes = [0.0, 0.0, 0.0]
        self.v_j2 = self.v_j3 = 0.0
        self.intent = None
        self.intent_expires_at = None

        self.settings = dict(DEFAULT_SETTINGS)
        self.settings_revision = 0
        self.checked_revision = -1
        self.q_ref = 0.0
        self.q_ideal = 0.0
        self.v_ref = 0.0
        self.q_j2 = 0.0
        self.v_j2 = 0.0
        self.q_j3 = 0.0
        self.v_j3 = 0.0
        self.last_reference_raw = None
        self.reference_history = deque(maxlen=MAX_REFERENCE_HISTORY)

        self._last_clock_value = None
        self.clock_fault = None
        self.last_reply_time = None
        self.last_ingress_time = None
        self.last_poll_host_s = 0.0
        self.last_step_host_s = None

        self.protection_fault = None
        self.protection_history = []
        self.latest = None
        self.events = deque(maxlen=MAX_EVENTS)


        self.packet_capture = PacketCapture()

        self.data = dict(
            state='Disconnected',
            reason='Select an adapter and connect.',
            connected=False,
            demo=False,
            calibrated=False,
            armed=False,
            stop_confirmed=False,
            port='',
            baud=6000000,
            id=0,
            connection=0,
        )

        self.thread = threading.Thread(
            target=self._loop,
            daemon=True,
            name='ArmManual serial owner',
        )
        self.thread.start()

    def _event(self, text):
        with self.lock:
            elapsed = max(0.0, time.monotonic() - getattr(self, '_started_host_s', time.monotonic()))
            self.events.append(f'{elapsed:8.2f}s  {text}')

    def _clock_now_locked(self):
        with self.lock:
            try:
                value = self.clock()
                if type(value) not in (int, float):
                    raise ValueError('Clock must return a finite nonnegative number')
                value = float(value)
                if not math.isfinite(value) or value < 0:
                    raise ValueError('Clock must return a finite nonnegative number')
                if self._last_clock_value is not None and value < self._last_clock_value:
                    raise ValueError('Clock moved backwards')
            except Exception as error:
                first_error = self.clock_fault is None
                self.clock_fault = str(error) or type(error).__name__
                if first_error:
                    self._latch_stop_locked('Clock invalid; Stop and re-check required')
                raise ValueError('Clock must be finite, nonnegative, and nondecreasing') from error
            self._last_clock_value = value
            self.clock_fault = None
            return value

    def _latch_stop_locked(self, reason):
        self.generation += 1
        self.stop_requested.set()
        self.stop_work.set()
        self.stop_latched = True
        self.armed = False
        self.data['armed'] = False
        self.stop_confirmed = False
        self.data['stop_confirmed'] = False
        self.require_neutral = True
        self.deadman_active = False
        self.target_axes = [0.0, 0.0, 0.0]
        self.v_j2 = self.v_j3 = 0.0
        self.intent = None
        self.intent_expires_at = None
        self.tickets.clear()
        if self.data['connected'] and self.data['state'] != 'Fault — serial close unconfirmed':
            self.data['state'] = 'Stopped · re-check required'
            self.data['reason'] = str(reason)[:240]
        self._event('STOP LATCHED: ' + str(reason)[:180])
        self.wake.set()

    def _record_overload(self, reason):
        with self.lock:
            self.protection_fault = str(reason)[:240]
            self.protection_history.append(
                {'time_s': time.monotonic(), 'reason': self.protection_fault}
            )
            if len(self.protection_history) > MAX_PROTECTION_HISTORY:
                del self.protection_history[:-MAX_PROTECTION_HISTORY]
            self._event('PROTECTION FAULT: ' + self.protection_fault)

    def _record_receipt(self, session, op_id, status, reason):
        with self.lock:
            if session not in self.sessions:
                return False
            rec_map = self.receipts.get(session)
            if rec_map is None:
                return False
            while len(rec_map) >= MAX_RECEIPTS_PER_SESSION and op_id not in rec_map:
                terminal = next((key for key, value in rec_map.items()
                                 if value['status'] != 'pending'), None)
                if terminal is None:
                    return False
                del rec_map[terminal]
            rec_map[op_id] = {'status': status, 'reason': str(reason)[:240]}
            return True

    def create_session(self) -> str:
        with self.lock:
            if len(self.sessions) >= MAX_SESSIONS:
                raise RuntimeError('Maximum sessions reached')
            session = secrets.token_hex(16)
            self.sessions.add(session)
            self.receipts[session] = OrderedDict()
            self._pending_by_session[session] = 0
            return session

    def controls_motor(self, session):
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')
            return session == self.owner and self.data['connected']

    def input_sequence(self, session):
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')
            return self.last_seq if session == self.owner else -1

    def edit_parameters(self, session, values):
        """Save finite drafts without changing applied settings or motion authority."""
        if not isinstance(values, dict) or not values or not set(values) <= set(DEFAULT_SETTINGS):
            raise ValueError('Unknown parameter draft fields')
        draft = {key: _finite_number(value, key) for key, value in values.items()}
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')
            self.parameter_drafts[session] = draft
        return dict(draft)

    def retire_session(self, session: str) -> bool:
        with self.lock:
            if session not in self.sessions:
                return False
            if session == self.owner and (
                self.bus is not None or self.port_owner is not None
                or self._pending_by_session.get(session, 0) > 0
            ):
                return False
            self.sessions.remove(session)
            self.parameter_drafts.pop(session, None)
            self.receipts.pop(session, None)
            self._pending_by_session.pop(session, None)
            for key, ticket in list(self.tickets.items()):
                if ticket['session'] == session:
                    del self.tickets[key]
            if session == self.owner:
                self.owner = None
            return True

    def _normalize_action(self, action, values):
        if type(action) is not str or action not in _ACTIONS:
            raise ValueError('Unknown action')
        if type(values) is not dict:
            raise ValueError('Action values must be an object')
        v = dict(values)
        if action == 'connect':
            demo = v.get('demo')
            port = v.get('port', '')
            baud = v.get('baud', 6000000)
            motor_id = v.get('id', 0)
            if type(demo) is not bool:
                raise ValueError('demo must be an exact bool')
            if type(motor_id) is not int or motor_id != 0:
                raise ValueError('Manual J1 accepts motor ID 0 only')
            if type(baud) is not int or baud not in (4000000, 6000000):
                raise ValueError('Baud must be integer 4000000 or 6000000')
            if type(port) is not str:
                raise ValueError('Port must be a string')
            if demo:
                return {'demo': True, 'port': '', 'baud': baud, 'id': 0}
            if not self.physical_opt_in:
                raise ValueError('Physical mode requires physical_opt_in=True')
            if baud != 6000000:
                raise ValueError('Physical mode restricted to baud 6000000')
            if not port.strip():
                raise ValueError('Physical port must be a non-empty string')
            if len(port) > 240:
                raise ValueError('Physical port path is too long')
            return {'demo': False, 'port': port.strip(), 'baud': 6000000, 'id': 0}
        if action in ('disconnect', 'check', 'release'):
            if v:
                raise ValueError(f'{action} does not accept values')
            return {}
        if action in ('ack', 'arm'):
            if v.get('supported') is not True:
                raise ValueError('Supported acknowledgment must be the exact bool true')
            return {'supported': True}
        if action == 'calibrate':
            known = _finite_number(v.get('known_angle_deg'), 'known_angle_deg')
            direction = v.get('direction')
            lower = _finite_number(v.get('lower_deg'), 'lower_deg')
            upper = _finite_number(v.get('upper_deg'), 'upper_deg')
            if type(direction) is not int or direction not in (-1, 1):
                raise ValueError('Direction must be the integer +1 or -1')
            if lower < -90.0 or upper > 90.0 or lower >= upper:
                raise ValueError('Travel limits must satisfy -90 <= lower < upper <= 90 degrees')
            if not lower <= known <= upper:
                raise ValueError('known_angle_deg must be within the travel limits')
            return {
                'known_angle_deg': known, 'direction': direction,
                'lower_deg': lower, 'upper_deg': upper,
            }
        if action == 'configure':
            kp = _finite_number(v.get('kp'), 'kp')
            kd = _finite_number(v.get('kd'), 'kd')
            max_rpm = _finite_number(v.get('max_rpm'), 'max_rpm')
            effort = _finite_number(v.get('effort_limit_nm'), 'effort_limit_nm')
            if not 0 < kp <= 30 or _kp_raw(kp) <= 0:
                raise ValueError('Kp must be positive, encodable, and at most 30')
            if not 0 <= kd <= 6 or _kd_raw(kd) < 0:
                raise ValueError('Kd must be between 0 and 6')
            if (not 0 < max_rpm <= MAX_RATE_RPM
                    or math.floor(max_rpm / 60 * RATIO * 64) < 1):
                raise ValueError(f'max_rpm must be in (0, {MAX_RATE_RPM}]')
            if not 0 < effort <= DEFAULT_EFFORT_CEILING_NM:
                raise ValueError(f'effort_limit_nm must be in (0, {DEFAULT_EFFORT_CEILING_NM}]')
            return {'kp': kp, 'kd': kd, 'max_rpm': max_rpm, 'effort_limit_nm': effort}
        raise ValueError('Unknown action')

    def _prune_tickets_locked(self, now):
        for key, ticket in list(self.tickets.items()):
            if ticket['used'] or now >= ticket['expires_at']:
                del self.tickets[key]

    def request(self, session: str, action: str, values: dict, *, ticket=None) -> str:
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')
            op_id = secrets.token_hex(8)
            try:
                payload = self._normalize_action(action, values)
            except (TypeError, ValueError, OverflowError) as error:
                self._record_receipt(session, op_id, 'rejected', str(error))
                return op_id



            if self.jobs.full():
                self._record_receipt(session, op_id, 'rejected', 'Queue full')
                return op_id
            if self.closing.is_set():
                self._record_receipt(session, op_id, 'rejected', 'Session closing')
                return op_id

            reserved_owner = False
            if action == 'connect':
                if self.owner not in (None, session):
                    self._record_receipt(session, op_id, 'rejected', 'Another session owns the bus')
                    return op_id
                if self.bus is not None or self.port_owner is not None:
                    self._record_receipt(session, op_id, 'rejected', 'Disconnect before reconnecting')
                    return op_id
                if self.owner is None:
                    self.owner = session
                    reserved_owner = True
            elif self.owner is not None and session != self.owner:
                self._record_receipt(session, op_id, 'rejected', 'Observer cannot control the owner session')
                return op_id

            ticket_generation = None
            ticket_expiry = None
            ticket_record = None
            if ticket is not None:
                if type(ticket) is not str:
                    if reserved_owner:
                        self.owner = None
                    self._record_receipt(session, op_id, 'rejected', 'Ticket must be a string')
                    return op_id
                ticket_record = self.tickets.get(ticket)
                if (ticket_record is None or ticket_record['used']
                        or ticket_record['session'] != session
                        or ticket_record['generation'] != self.generation):
                    if reserved_owner:
                        self.owner = None
                    self._record_receipt(session, op_id, 'rejected', 'Ticket is invalid or no longer current')
                    return op_id
                try:
                    now = self._clock_now_locked()
                except ValueError as error:
                    if reserved_owner:
                        self.owner = None
                    self._record_receipt(session, op_id, 'rejected', str(error))
                    return op_id
                if now >= ticket_record['expires_at']:
                    del self.tickets[ticket]
                    if reserved_owner:
                        self.owner = None
                    self._record_receipt(session, op_id, 'rejected', 'Ticket expired')
                    return op_id
                ticket_generation = ticket_record['generation']
                ticket_expiry = ticket_record['expires_at']

            if ticket_record is not None:
                ticket_record['used'] = True
            job = (
                self.generation, session, op_id, action, payload,
                ticket_generation, ticket_expiry,
            )
            self._record_receipt(session, op_id, 'pending', '')
            try:
                self.jobs.put_nowait(job)
            except queue.Full:
                if ticket_record is not None:
                    ticket_record['used'] = False
                if reserved_owner:
                    self.owner = None
                self._record_receipt(session, op_id, 'rejected', 'Queue full')
                return op_id
            self._pending_by_session[session] = self._pending_by_session.get(session, 0) + 1
            self.wake.set()
            return op_id

    def receipt(self, session: str, op: str) -> dict:
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')
            rec_map = self.receipts.get(session, {})
            if op not in rec_map:
                return {'status': 'rejected', 'reason': 'Unknown operation'}
            result = rec_map[op]
            return {'status': result['status'], 'reason': result['reason']}

    def issue_ticket(self, session: str) -> str:
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')



            if self.owner is not None and session != self.owner:
                raise ValueError('Session is not owner')
            if self.closing.is_set():
                raise ValueError('Session is closing')
            now = self._clock_now_locked()
            self._prune_tickets_locked(now)
            if len(self.tickets) >= MAX_TICKETS:
                raise RuntimeError('Maximum active tickets reached')
            token = secrets.token_hex(16)
            self.tickets[token] = {
                'session': session,
                'issued_at': now,
                'expires_at': now + TICKET_EXPIRY_S,
                'generation': self.generation,
                'used': False,
            }
            return token

    def submit(self, session: str, ticket: str, seq: int, deadman: bool, axes, device: str) -> None:
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')
            if self.owner is None or session != self.owner:
                raise ValueError('Session is not the current owner')
            if not self.armed or not self.data['connected']:
                raise ValueError('Session is not armed and connected')
            if type(ticket) is not str:
                raise ValueError('Ticket must be a string')
            record = self.tickets.get(ticket)
            if record is None or record['used']:
                raise ValueError('Unknown or used ticket')
            if record['session'] != session or record['generation'] != self.generation:
                raise ValueError('Ticket session or generation mismatch')
            if type(seq) is not int or seq < 0:
                raise ValueError('Sequence must be a nonnegative integer')
            if seq <= self.last_seq:
                raise ValueError('Sequence must be strictly increasing')
            if type(deadman) is not bool:
                raise ValueError('Deadman must be an exact bool')
            if type(axes) not in (list, tuple) or len(axes) != 3:
                raise ValueError('Axes must be a 3-vector')
            parsed_axes = []
            for axis in axes:
                if type(axis) not in (int, float) or not math.isfinite(float(axis)):
                    raise ValueError('Axes components must be finite numbers')
                parsed_axis = float(axis)
                if not -1.0 <= parsed_axis <= 1.0:
                    raise ValueError('Axes components must be within [-1, 1]')
                parsed_axes.append(parsed_axis)
            if type(device) is not str or not device.strip() or len(device) > 64:
                raise ValueError('Device must be a bounded non-empty string')
            device_name = device.strip()
            neutral_release = (not deadman and all(axis == 0.0 for axis in parsed_axes))



            now = self._clock_now_locked()
            if now >= record['expires_at']:
                del self.tickets[ticket]
                raise ValueError('Ticket expired')

            record['used'] = True
            self.last_seq = seq
            if self.current_device is not None and device_name != self.current_device:
                self.current_device = device_name
                self._latch_stop_locked('Input device changed; Stop and re-check required')
                return
            self.current_device = device_name

            is_moving = any(abs(axis) > 1e-4 for axis in parsed_axes)
            if deadman or is_moving:
                self.deadman_active = True
                self.target_axes = parsed_axes
                self.require_neutral = False
                self.intent = {
                    'session': session, 'ticket': ticket, 'seq': seq,
                    'device': device_name, 'generation': self.generation,
                }
                self.intent_expires_at = record['expires_at']
                self.last_ingress_time = now
            else:
                self.deadman_active = False
                self.target_axes = [0.0, 0.0, 0.0]
                self.require_neutral = False
                self.intent = {
                    'session': session, 'ticket': ticket, 'seq': seq,
                    'device': device_name, 'generation': self.generation,
                }
                self.intent_expires_at = record['expires_at']
                self.last_ingress_time = now
            self.wake.set()

    def stop(self, session: str, reason: str) -> None:
        if type(reason) is not str or not reason.strip() or len(reason) > 240:
            raise ValueError('Stop reason must be a bounded non-empty string')
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')
            self._latch_stop_locked(reason.strip())

    def close(self) -> bool:
        with self.lock:
            if self.closed.is_set():
                return self.bus is None and self.port_owner is None
            self._close_requests += 1
            self.closing.set()
            self._latch_stop_locked('Session closing')
            self.close_requested.set()
            self.wake.set()
        if threading.current_thread() is self.thread:
            return False
        self.thread.join(0.75)
        return self.closed.is_set() and self.bus is None and self.port_owner is None

    def snapshot(self) -> dict:
        with self.lock:
            now = None
            if self.data['connected']:
                try:
                    now = self._clock_now_locked()
                except ValueError:
                    now = None
            age = None
            if now is not None and self.last_reply_time is not None:
                age = max(0.0, now - self.last_reply_time)

            latest = None
            if self.latest is not None and self.data['connected']:
                latest = {
                    key: self.latest[key]
                    for key in (
                        'id', 'mode', 'timeout', 'voltage', 'driver_c', 'winding_c',
                        'errors', 'warnings', 'torque_nm', 'rpm', 'position_rad', 'raw',
                    )
                }
                latest['source'] = 'demo' if self.data['demo'] else 'physical'

            calibrated = bool(self.calibrated and self.data['connected'])
            if calibrated and latest is not None:
                direction = self.settings['direction']
                j1_position = direction * (latest['position_rad'] - self.q_zero)
                j1_velocity = direction * latest['rpm'] * math.tau / 60
            else:
                j1_position = None
                j1_velocity = None
            source = ('demo' if self.data['demo'] else 'physical') if self.data['connected'] else 'unknown'
            j1 = {
                'name': 'J1', 'source': source,
                'position_rad': j1_position, 'velocity_rad_s': j1_velocity,
                'mode': latest['mode'] if latest else None,
                'timeout': latest['timeout'] if latest else None,
                'voltage': latest['voltage'] if latest else None,
                'driver_temp': latest['driver_c'] if latest else None,
                'winding_temp': latest['winding_c'] if latest else None,
                'errors': latest['errors'] if latest else None,
                'warnings': latest['warnings'] if latest else None,
                'torque_nm': latest['torque_nm'] if latest else None,
            }
            joints = [
                j1,
                {'name': 'J2', 'source': 'virtual', 'position_rad': self.q_j2,
                 'velocity_rad_s': self.v_j2},
                {'name': 'J3', 'source': 'virtual', 'position_rad': self.q_j3,
                 'velocity_rad_s': self.v_j3},
            ]
            return {
                'state': self.data['state'], 'reason': self.data['reason'],
                'connected': bool(self.data['connected']), 'demo': bool(self.data['demo']),
                'calibrated': calibrated, 'armed': bool(self.armed),
                'stop_confirmed': bool(self.stop_confirmed and self.data['connected']),
                'feedback_age_s': age, 'settings': dict(self.settings), 'joints': joints,
                'latest': latest, 'events': list(self.events),
            }

    def _assert_owner_thread(self):
        if threading.current_thread() is not self.thread:
            raise RuntimeError('Serial operation must run on the owner thread')

    def _sanitize_feedback(self, feedback):
        fields = (
            'id', 'mode', 'timeout', 'voltage', 'driver_c', 'winding_c',
            'errors', 'warnings', 'torque_nm', 'rpm', 'position_rad', 'raw',
        )
        return {name: feedback[name] for name in fields if name in feedback}

    def _transact_raw(self, packet, motor_id=0, before_write=None, on_write=None):
        self._assert_owner_thread()
        if self.bus is None or not getattr(self.bus, 'is_open', False):
            raise RuntimeError('Not connected')
        source = 'demo' if self.data['demo'] else 'physical'
        identity = 'demo-simulator' if source == 'demo' else self.data['port']
        result = capture_exchange(
            self.packet_capture, exchange, self.bus, packet, motor_id,
            before_write=before_write, on_write=on_write,
            source=source, identity=identity,
            connection=self.data['connection'],
            port='' if source == 'demo' else self.data['port'],
            baud=self.data['baud'],
        )
        with self.lock:
            self.latest = self._sanitize_feedback(result)
            try:
                self.last_reply_time = self._clock_now_locked()
            except ValueError:


                self.last_reply_time = None
        return result

    def _record_stop_result(self, confirmed):
        with self.lock:
            self.stop_confirmed = bool(confirmed and self.data['connected'])
            self.data['stop_confirmed'] = self.stop_confirmed

    def _stop(self):
        """Owner-thread mode-0 attempts; independent of the authority clock."""
        self._assert_owner_thread()
        with self.lock:
            self.armed = False
            self.data['armed'] = False
            self.deadman_active = False
            self.require_neutral = True
            self.target_axes = [0.0, 0.0, 0.0]
            self.intent = None
            self.intent_expires_at = None
        if self.bus is None or not getattr(self.bus, 'is_open', False):
            self._record_stop_result(False)
            return False
        if not self.data['demo'] and not (
            self.port_owner is not None and getattr(self.port_owner, 'verified', False)
        ):
            self._record_stop_result(False)
            return False
        packet = command(0, 0, 0, 0, 0, True)
        last_error = None
        for _ in range(3):
            try:
                reply = self._transact_raw(packet, motor_id=0)
                if reply['mode'] == 0:
                    self._record_stop_result(True)
                    self.last_poll_host_s = time.monotonic()
                    return True
                last_error = RuntimeError('Motor has not confirmed Stop')
            except Exception as error:
                last_error = error
        self._record_stop_result(False)
        self._event('STOP NOT CONFIRMED: ' + (str(last_error) or type(last_error).__name__))
        self.last_poll_host_s = time.monotonic()
        return False

    def _close_resource(self):
        self._assert_owner_thread()
        owner = self.port_owner
        bus = self.bus
        if owner is not None:
            try:
                confirmed = owner.close()
            except Exception:
                confirmed = False
        elif bus is not None:
            try:
                bus.close()
                confirmed = getattr(bus, 'is_open', None) is False
            except Exception:
                confirmed = False
        else:
            confirmed = True
        if not confirmed:
            self._event('SERIAL CLOSE NOT CONFIRMED: connection and lease retained.')
            return False
        self.bus = None
        self.port_owner = None
        return True

    def _mark_disconnected(self, reason):
        with self.lock:
            self.generation += 1
            self.tickets.clear()
            self.owner = None
            self.calibrated = False
            self.q_zero = 0.0
            self.armed = False
            self.deadman_active = False
            self.require_neutral = True
            self.target_axes = [0.0, 0.0, 0.0]
            self.intent = None
            self.intent_expires_at = None
            self.current_device = None
            self.last_seq = -1
            self.v_j2 = self.v_j3 = 0.0
            self.last_reply_time = None
            self.latest = None
            self.stop_confirmed = False
            self.data.update(
                connected=False, demo=False, calibrated=False, armed=False,
                stop_confirmed=False, state='Disconnected', reason=str(reason)[:240],
                port='',
            )
            self.last_reference_raw = None
            self.q_ref = self.q_ideal = self.v_ref = 0.0

    def _disconnect(self):
        self._assert_owner_thread()
        if self.bus is None and self.port_owner is None:
            self._mark_disconnected('Disconnected. Reconnect requires a new check and arm action.')
            return
        with self.lock:
            self._latch_stop_locked('Disconnect requested')
        stop_ok = self._stop()
        if not self._close_resource():
            with self.lock:
                self.data.update(
                    state='Fault — serial close unconfirmed',
                    reason='Serial close could not be confirmed; adapter and owner remain held.',
                    connected=True,
                )
            raise RuntimeError('Serial close unconfirmed; owner and bus retained for retry')
        reason = 'Stop confirmed before adapter close. Reconnect requires a new check and arm action.'
        if not stop_ok:
            reason = 'Adapter closed, but a mode-0 response was not confirmed.'
        self._mark_disconnected(reason)
        self._event('Disconnected.')

    def _validate_feedback(self, reply, mode, *, allow_timeout=False):
        if reply.get('id') != 0 or reply.get('mode') != mode:
            raise RuntimeError(f'Unexpected motor ID or mode in feedback: {reply.get("mode")}')
        if reply.get('errors') or reply.get('warnings'):
            self._record_overload('Motor reported errors or warnings')
            raise RuntimeError('Motor reports errors or warnings')
        if reply.get('timeout') and not allow_timeout:
            raise RuntimeError('Motor watchdog timeout is active')
        if not 24 <= reply.get('voltage', -math.inf) <= 72:
            raise RuntimeError('Motor voltage is outside the allowed range')
        if not -10 <= reply.get('driver_c', math.inf) <= 70:
            self._record_overload('Driver temperature is outside the allowed range')
            raise RuntimeError('Driver temperature is outside the allowed range')
        if reply.get('winding_c', math.inf) > 90:
            self._record_overload('Winding temperature exceeds 90 C')
            raise RuntimeError('Winding temperature exceeds 90 C')
        if abs(reply.get('torque_nm', math.inf)) > self.settings['effort_limit_nm']:
            self._record_overload('Measured motor effort exceeds the configured ceiling')
            raise RuntimeError('Measured torque exceeds the configured effort ceiling')
        healthy(reply, mode, allow_timeout=allow_timeout)
        return True

    def _fresh_feedback_age(self, now=None):
        if now is None:
            with self.lock:
                now = self._clock_now_locked()
        if self.last_reply_time is None:
            return None
        age = now - self.last_reply_time
        if not math.isfinite(age) or age < 0:
            return None
        return age

    def _require_fresh_stop(self):
        if not self.data['connected'] or self.bus is None:
            raise ValueError('Connect first')
        if self.armed or not self.stop_confirmed:
            raise ValueError('Requires confirmed Stop and disarmed state')
        now = self._clock_now_locked()
        age = self._fresh_feedback_age(now)
        if age is None or age > FEEDBACK_MAX_AGE_S:
            raise ValueError('Feedback stale; check connection again')
        if self.latest is None or self.latest['mode'] != 0:
            raise ValueError('Motor must be in mode 0')
        self._validate_feedback(self.latest, 0)

    def _invalidate_settings(self, reason):
        with self.lock:
            self.settings_revision += 1
            self.checked_revision = -1
            self._latch_stop_locked(reason + '; Check and Arm required')

    def _check(self):
        self._assert_owner_thread()
        if self.bus is None:
            raise ValueError('Connect first')
        with self.lock:
            self._latch_stop_locked('Check requested')
            check_generation = self.generation
        if not self._stop():
            raise RuntimeError('Cannot verify Stop; check adapter, ID, baud and motor power')
        reply = self._transact_raw(command(0, 0, 0, 0, 0, True), motor_id=0)
        self._validate_feedback(reply, 0, allow_timeout=True)
        now = self._clock_now_locked()
        age = self._fresh_feedback_age(now)
        if age is None or age > FEEDBACK_MAX_AGE_S:
            raise RuntimeError('Fresh mode-0 feedback was not available')
        with self.lock:
            if self.generation != check_generation:
                raise WriteCancelledError('Stop arrived while Check was in progress')
            self.stop_confirmed = True
            self.data['stop_confirmed'] = True
            self.stop_latched = False
            self.stop_requested.clear()
            self.stop_work.clear()
            self.checked_revision = self.settings_revision
            if self.protection_fault is not None:
                self.data['state'] = 'Fault — retained protection fault'
                self.data['reason'] = 'Retained protection fault: ' + self.protection_fault
                raise RuntimeError(self.data['reason'])
            if reply['timeout']:
                self.data['state'] = 'Timeout — disarmed'
                self.data['reason'] = 'Click Acknowledge timeout while the motor is supported.'
            else:
                self.data['state'] = 'Ready · Stop'
                self.data['reason'] = 'Healthy feedback. Motor stopped. Arm before jogging.'

    def _ack(self, supported):
        self._assert_owner_thread()
        if supported is not True:
            raise ValueError('Supported acknowledgment required')
        if self.bus is None or not self.stop_confirmed or self.latest is None:
            raise ValueError('Check a connected Stop state first')
        if self.stop_latched or self.stop_requested.is_set():
            raise ValueError('Check again after Stop before acknowledging the watchdog')
        if self.armed or self.latest.get('mode') != 0:
            raise ValueError('Acknowledge requires actual mode-0 Stop')
        self._validate_feedback(self.latest, 0, allow_timeout=True)
        now = self._clock_now_locked()
        age = self._fresh_feedback_age(now)
        if age is None or age > FEEDBACK_MAX_AGE_S:
            raise ValueError('Feedback stale; check connection again')
        if self.latest.get('timeout') is not True and self.latest.get('timeout') != 1:
            raise ValueError('There is no active startup timeout to acknowledge')

        start_generation = self.generation
        no_watchdog = command(0, 0, 0, 0, 0, False)
        watchdog = command(0, 0, 0, 0, 0, True)
        failure = None
        restored = None
        try:
            reply = self._transact_raw(no_watchdog, motor_id=0)
            self._validate_feedback(reply, 0, allow_timeout=True)
        except Exception as error:
            failure = error
        finally:
            try:
                restored = self._transact_raw(watchdog, motor_id=0)
                self._validate_feedback(restored, 0, allow_timeout=False)
            except Exception as error:
                if failure is None:
                    failure = error
        if failure is not None:
            raise RuntimeError('Watchdog acknowledgment failed; mode-0 watchdog restore attempted') from failure
        self._clock_now_locked()
        with self.lock:
            if self.generation != start_generation:
                raise WriteCancelledError('Stop arrived during watchdog acknowledgment')
            self.stop_confirmed = True
            self.data['stop_confirmed'] = True
            self.checked_revision = self.settings_revision
            self.stop_latched = False
            self.stop_requested.clear()
            self.stop_work.clear()
            self.data['state'] = 'Ready · Stop'
            self.data['reason'] = 'Startup timeout acknowledged in Stop; firmware watchdog restored.'
            self._event('Startup timeout acknowledged in Stop; firmware watchdog restored.')

    def _handle_action(self, session, action, values):
        self._assert_owner_thread()
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Session retired before dispatch')
            if session != self.owner:
                raise ValueError('Session is no longer the bus owner')

        if action == 'connect':
            if self.bus is not None or self.port_owner is not None:
                raise ValueError('Disconnect before changing adapter')
            demo = values['demo']
            port = values['port']
            baud = values['baud']
            with self.lock:
                self.data.update(
                    state='Connecting', reason='Opening adapter and verifying Stop.',
                    id=0, baud=baud, port=port, demo=demo,
                    connection=self.data['connection'] + 1, connected=False,
                    armed=False, calibrated=False, stop_confirmed=False,
                )
                self.calibrated = False
                self.q_zero = 0.0
                self.stop_confirmed = False
                self.latest = None
                self.last_reply_time = None
                self.current_device = None
                self.last_seq = -1
                self.tickets.clear()
                self.deadman_active = False
                self.require_neutral = True
            if demo:
                self.bus = DemoSerial(clock=self.clock)
            else:
                try:
                    config = PortConfig(port, baud)
                    inspect_report = inspect_port(config)
                except ValueError:
                    config = PortConfig(port, baud, expected_serial=None, expected_channel=None)
                    inspect_report = inspect_port(config)
                self._event(inspect_report)
                try:
                    self.port_owner = open_managed_port(config, serial_factory=self.serial_factory)
                except Exception as error:
                    failed_owner = getattr(error, 'managed_port', None)
                    if failed_owner is not None:
                        self.port_owner = failed_owner
                        self.bus = failed_owner.bus
                    raise
                self.bus = self.port_owner.bus
            with self.lock:
                self.data['connected'] = True
            self._event(f'{"DEMO" if demo else "REAL"} port open: {port} · ID 0 · {baud}')
            self._check()
            return

        if action == 'disconnect':
            self._disconnect()
            return
        if action == 'check':
            self._check()
            return
        if action == 'ack':
            self._ack(values['supported'])
            return
        if action == 'calibrate':
            self._require_fresh_stop()

            reply = self._transact_raw(command(0, 0, 0, 0, 0, True), motor_id=0)
            self._validate_feedback(reply, 0)
            known_rad = math.radians(values['known_angle_deg'])
            direction = values['direction']
            with self.lock:
                self.q_zero = reply['position_rad'] - direction * known_rad
                self.settings.update(
                    known_angle_deg=values['known_angle_deg'], direction=direction,
                    lower_deg=values['lower_deg'], upper_deg=values['upper_deg'],
                )
                self.calibrated = True
                self.data['calibrated'] = True
                self._invalidate_settings('Calibration changed')
                self._event(
                    f'Calibrated: known={values["known_angle_deg"]}°, dir={direction}, '
                    f'limits=[{values["lower_deg"]}°, {values["upper_deg"]}°]'
                )
            return
        if action == 'configure':
            self._require_fresh_stop()
            with self.lock:
                self.settings.update(values)
                self._invalidate_settings('Settings changed')
                self._event(
                    f'Configured: kp={values["kp"]}, kd={values["kd"]}, '
                    f'max_rpm={values["max_rpm"]}, effort_limit={values["effort_limit_nm"]} Nm'
                )
            return
        if action == 'arm':
            if values.get('supported') is not True:
                raise ValueError('Supported arm acknowledgment required')
            with self.lock:



                if self.stop_requested.is_set() or self.stop_latched:
                    raise ValueError('Stop is latched; Check again before arming')
                if self.bus is None or not self.data['connected']:
                    raise ValueError('Connect first')
                if self.protection_fault is not None:
                    raise ValueError('Arm rejected due to retained protection fault: ' + self.protection_fault)
                if self.data['state'] != 'Ready · Stop' or not self.stop_confirmed:
                    raise ValueError('Arm requires Ready · Stop state')
                if self.checked_revision != self.settings_revision:
                    raise ValueError('Arm requires Check after the latest settings or calibration change')
                if not self.calibrated:
                    raise ValueError('Arm requires calibration on the current connection')
                if self.latest is None or self.latest.get('mode') != 0:
                    raise ValueError('Motor must be in actual mode 0 before arming')
                now = self._clock_now_locked()
                age = self._fresh_feedback_age(now)
                if age is None or age > FEEDBACK_MAX_AGE_S:
                    raise ValueError('Feedback stale; check connection again')
                self._validate_feedback(self.latest, 0)
                self.armed = True
                self.data['armed'] = True
                self.deadman_active = False
                self.require_neutral = True
                self.intent = None
                self.intent_expires_at = None
                self.last_step_host_s = time.monotonic()
                self.q_ref = self.q_ideal = 0.0
                self.v_ref = 0.0
                self.last_reference_raw = None
                self.data['state'] = 'Armed · motor stopped'
                self.data['reason'] = 'Armed for manual jogging. Neutral release then press deadman.'
                self._event('Armed for manual jogging; motor remains stopped.')
            return
        if action == 'release':
            with self.lock:
                self._latch_stop_locked('Disarmed by operator')
            if not self._stop():
                raise RuntimeError('Disarm Stop was not confirmed')
            with self.lock:
                self.data['state'] = 'Stopped · re-check required'
                self.data['reason'] = 'Disarmed by operator. Check and Arm again before jogging.'
            return
        raise ValueError('Unknown action')

    def _mode1_gate(self, intent, expected_mode):
        with self.lock:
            if (self.closing.is_set() or self.close_requested.is_set()
                    or self.stop_requested.is_set() or self.stop_latched):
                return False
            if not self.armed or not self.data['connected'] or self.bus is None:
                return False
            if self.owner is None or intent is None or intent is not self.intent:
                return False
            if intent['session'] != self.owner or intent['generation'] != self.generation:
                return False
            if not self.calibrated or self.checked_revision != self.settings_revision:
                return False
            try:
                now = self._clock_now_locked()
            except ValueError:
                return False
            if self.intent_expires_at is None or now >= self.intent_expires_at:
                self._latch_stop_locked('Input ticket lease expired; Stop and re-check required')
                return False
            if self.last_reply_time is None or self.latest is None:
                self._latch_stop_locked('Fresh feedback unavailable; Stop and re-check required')
                return False
            age = now - self.last_reply_time
            if age < 0 or age > FEEDBACK_MAX_AGE_S:
                self._latch_stop_locked('Feedback stale; Stop and re-check required')
                return False
            if self.latest.get('mode') != expected_mode:
                self._latch_stop_locked('Motor feedback mode changed; Stop and re-check required')
                return False
            try:
                self._validate_feedback(self.latest, expected_mode)
            except Exception:
                self._latch_stop_locked('Feedback health gate failed; Stop and re-check required')
                return False
            return True

    def _send_mode1(self, packet, decoded, intent, expected_mode, kind):
        def before_write():
            return self._mode1_gate(intent, expected_mode)

        def on_write(result):
            if result is not WriteResult.COMPLETE:
                return
            host_time = time.monotonic()
            joint_position = self.settings['direction'] * (decoded['position_rad'] - self.q_zero)
            with self.lock:
                self.reference_history.append({
                    'time_s': host_time,
                    'position_rad': joint_position,
                    'raw_position': decoded['raw_position'],
                    'rpm': self.settings['direction'] * decoded['rpm'],
                    'kind': kind,
                })

        return self._transact_raw(
            packet, motor_id=0, before_write=before_write, on_write=on_write
        )

    def _enter_foc(self, intent):
        if self.latest is None or self.latest.get('mode') != 0:
            raise RuntimeError('FOC entry requires a current mode-0 reply')
        raw_position = position_to_raw(self.latest['position_rad'])
        packet = command_with_raw_position(
            0, raw_position, mode=1, rpm=0.0, kd=0.0, torque=0.0,
            watchdog=True, kp=0.0, raw_rpm=0,
        )
        decoded = decode_command(packet)
        first = self._send_mode1(packet, decoded, intent, 0, 'foc-entry')
        if first.get('mode') not in (0, 1):
            raise RuntimeError('FOC entry reply must show Stop or FOC')
        self._validate_feedback(first, first['mode'])
        if self.stop_requested.is_set():
            raise WriteCancelledError('Stop arrived during FOC entry')
        second = self._send_mode1(packet, decoded, intent, first['mode'], 'foc-entry')
        if second.get('mode') != 1:
            raise RuntimeError('FOC entry second reply must confirm mode 1')
        self._validate_feedback(second, 1)
        direction = self.settings['direction']
        joint_q = direction * (decoded['position_rad'] - self.q_zero)
        with self.lock:
            if self.stop_requested.is_set() or intent is not self.intent:
                raise WriteCancelledError('Stop arrived during FOC entry')
            self.q_ref = joint_q
            self.q_ideal = joint_q
            self.v_ref = 0.0
            self.last_reference_raw = decoded['raw_position']
            self.stop_confirmed = False
            self.data['stop_confirmed'] = False
            self.data['state'] = 'Jogging'
            self.data['reason'] = 'Manual jogging active'

    def _candidate_position(self, ideal_joint_q, previous_raw, previous_joint_q, step_limit):
        direction = self.settings['direction']
        q_min = math.radians(self.settings['lower_deg'])
        q_max = math.radians(self.settings['upper_deg'])
        motor_low, motor_high = (
            (self.q_zero + q_min, self.q_zero + q_max)
            if direction == 1 else
            (self.q_zero - q_max, self.q_zero - q_min)
        )
        motor_low = max(motor_low, POSITION_MIN)
        motor_high = min(motor_high, POSITION_MAX)
        prev_motor = raw_to_position(previous_raw)
        move_low = max(motor_low, prev_motor - step_limit)
        move_high = min(motor_high, prev_motor + step_limit)
        scale = 32768 * RATIO / math.tau
        raw_low = max(-2147483648, math.ceil(move_low * scale))
        raw_high = min(2147483647, math.floor(move_high * scale))
        if raw_low > raw_high:
            return previous_raw, previous_joint_q
        ideal_motor = self.q_zero + direction * ideal_joint_q
        desired = ideal_motor * scale
        central = round(desired)
        candidates = {
            raw_low, raw_high,
            min(raw_high, max(raw_low, central)),
            min(raw_high, max(raw_low, math.floor(desired))),
            min(raw_high, max(raw_low, math.ceil(desired))),
            min(raw_high, max(raw_low, previous_raw)),
        }
        possible = []
        for raw in candidates:
            motor_q = raw_to_position(raw)
            joint_q = direction * (motor_q - self.q_zero)
            if not q_min <= joint_q <= q_max:
                continue
            if abs(joint_q - previous_joint_q) > step_limit:
                continue
            possible.append((abs(joint_q - ideal_joint_q), raw, joint_q))
        if not possible:
            return previous_raw, previous_joint_q
        _, selected_raw, selected_q = min(possible)
        return int(selected_raw), selected_q

    def _build_motion_packet(self, dt):
        direction = self.settings['direction']
        max_rpm = self.settings['max_rpm']
        max_speed = max_rpm * math.tau / 60
        q_min = math.radians(self.settings['lower_deg'])
        q_max = math.radians(self.settings['upper_deg'])
        if self.last_reference_raw is None:
            self.last_reference_raw = position_to_raw(self.latest['position_rad'])
            self.q_ref = direction * (raw_to_position(self.last_reference_raw) - self.q_zero)
            self.q_ideal = self.q_ref
            self.v_ref = 0.0

        if self.deadman_active:
            v_target = self.target_axes[0] * max_speed
        else:
            v_target = 0.0
        a_max = MAX_ACCEL_RAD_S2
        if self.v_ref > 0:
            available = q_max - self.q_ref
            if self.v_ref * self.v_ref / (2 * a_max) >= available:
                v_target = min(v_target, math.sqrt(max(0.0, 2 * a_max * available)))
        elif self.v_ref < 0:
            available = self.q_ref - q_min
            if self.v_ref * self.v_ref / (2 * a_max) >= available:
                v_target = max(v_target, -math.sqrt(max(0.0, 2 * a_max * available)))
        delta_v = a_max * dt
        if v_target > self.v_ref:
            next_v = min(v_target, self.v_ref + delta_v)
        else:
            next_v = max(v_target, self.v_ref - delta_v)
        next_v = max(-max_speed, min(max_speed, next_v))
        ideal_q = max(q_min, min(q_max, self.q_ideal + next_v * dt))

        raw_rpm_limit = math.floor(max_rpm / 60 * RATIO * 64)
        wanted_rpm = direction * next_v * 60 / math.tau
        raw_rpm = math.trunc(wanted_rpm / 60 * RATIO * 64)
        raw_rpm = max(-raw_rpm_limit, min(raw_rpm_limit, raw_rpm))
        decoded_rpm = raw_rpm / 64 / RATIO * 60

        max_position_step = max_speed * dt
        accel_position_step = abs(next_v) * dt
        step_limit = min(max_position_step, accel_position_step)
        raw_position, decoded_joint_q = self._candidate_position(
            ideal_q, self.last_reference_raw, self.q_ref, step_limit
        )
        packet = command_with_raw_position(
            0, raw_position, mode=1, rpm=decoded_rpm,
            kd=self.settings['kd'], torque=0.0, watchdog=True,
            kp=self.settings['kp'], raw_rpm=raw_rpm,
        )
        decoded = decode_command(packet)
        decoded_joint_q = direction * (decoded['position_rad'] - self.q_zero)
        decoded_joint_v = direction * decoded['rpm'] * math.tau / 60
        if not q_min <= decoded_joint_q <= q_max:
            raise RuntimeError('Decoded position exceeds the inclusive travel limits')
        if abs(decoded_joint_q - self.q_ref) > step_limit:
            raise RuntimeError('Decoded packet step exceeds configured speed and capped dt')
        if abs(decoded['rpm']) > max_rpm:
            raise RuntimeError('Decoded packet velocity exceeds configured maximum')

        current_q = direction * (self.latest['position_rad'] - self.q_zero)
        current_v = direction * self.latest['rpm'] * math.tau / 60
        predicted_effort = abs(
            decoded['kp'] * (decoded_joint_q - current_q)
            + decoded['kd'] * (decoded_joint_v - current_v)
        )
        if predicted_effort > self.settings['effort_limit_nm']:
            reason = (
                f'Predicted effort {predicted_effort:.3f} Nm exceeds ceiling '
                f'{self.settings["effort_limit_nm"]:.3f} Nm'
            )
            self._record_overload(reason)
            raise RuntimeError(reason)
        return packet, decoded, next_v, ideal_q, decoded_joint_q

    def _check_motion_reply(self, reply, decoded_joint_q):
        self._validate_feedback(reply, 1)
        if abs(reply['rpm']) > MAX_MEASURED_SPEED_RPM:
            raise RuntimeError(
                f'Measured speed {reply["rpm"]:.3f} rpm exceeds {MAX_MEASURED_SPEED_RPM} rpm'
            )
        direction = self.settings['direction']
        measured_q = direction * (reply['position_rad'] - self.q_zero)
        q_min = math.radians(self.settings['lower_deg'])
        q_max = math.radians(self.settings['upper_deg'])
        if measured_q < q_min or measured_q > q_max:
            raise RuntimeError(f'Measured position {measured_q:.4f} rad exceeds travel limits')
        if abs(decoded_joint_q - measured_q) > MAX_MEASURED_LEAD_RAD:
            raise RuntimeError(
                f'Measured position lead {abs(decoded_joint_q - measured_q):.4f} rad '
                f'exceeds {MAX_MEASURED_LEAD_RAD} rad'
            )
        if abs(reply['torque_nm']) > self.settings['effort_limit_nm']:
            reason = (
                f'Measured torque {reply["torque_nm"]:.3f} Nm exceeds ceiling '
                f'{self.settings["effort_limit_nm"]:.3f} Nm'
            )
            self._record_overload(reason)
            raise RuntimeError(reason)

    def _trip_loss(self, reason):
        with self.lock:
            self._latch_stop_locked(reason)
        stop_ok = self._stop()
        with self.lock:
            if stop_ok:
                self.data['state'] = 'Stopped · re-check required'
                self.data['reason'] = str(reason)[:240]
            else:
                self.data['state'] = 'Fault — Stop unconfirmed'
                self.data['reason'] = 'No actual mode-0 response confirmed Stop after input loss.'
        self.stop_work.clear()

    def _trip_fault(self, error):
        message = str(error) or type(error).__name__
        with self.lock:
            self._latch_stop_locked(message)
            if self.data['state'] != 'Fault — serial close unconfirmed':
                self.data['state'] = 'Fault — disarmed'
                self.data['reason'] = message[:240]
        self._stop()
        if not self.data['connected'] and (self.bus is not None or self.port_owner is not None):
            if self._close_resource():
                self._mark_disconnected('Connection setup failed; resources closed.')
            else:
                with self.lock:
                    self.data['state'] = 'Fault — serial close unconfirmed'
                    self.data['reason'] = 'Connection setup failed; adapter ownership remains held.'
        with self.lock:
            if self.bus is not None and not getattr(self.bus, 'is_open', False):
                if self.port_owner is None:
                    self.bus = None
                    self.data['connected'] = False
            if self.data['state'] != 'Fault — serial close unconfirmed':
                self.data['state'] = 'Fault — disarmed'
                self.data['reason'] = message[:240]
            self.data['armed'] = False
            self.armed = False
        self.stop_work.clear()

    def _fresh_mode1_intent(self, now):
        with self.lock:
            intent = self.intent
            if not self.armed or not self.data['connected'] or self.bus is None:
                return None
            if intent is None or intent['session'] != self.owner:
                return None
            if intent['generation'] != self.generation:
                return None
            if self.intent_expires_at is None or now >= self.intent_expires_at:
                return None
            if self.stop_requested.is_set() or self.stop_latched or self.closing.is_set():
                return None
            if self.last_reply_time is None or self.latest is None:
                return None
            age = now - self.last_reply_time
            if age < 0 or age > FEEDBACK_MAX_AGE_S:
                return None
            return intent

    def _poll_stop(self):
        reply = self._transact_raw(command(0, 0, 0, 0, 0, True), motor_id=0)
        self.last_poll_host_s = time.monotonic()
        if reply.get('mode') != 0:
            with self.lock:
                self.stop_confirmed = False
                self.data['stop_confirmed'] = False
            raise RuntimeError('Idle mode-0 poll did not confirm Stop')
        self._record_stop_result(True)
        self._validate_feedback(reply, 0, allow_timeout=True)
        with self.lock:
            if self.data['state'].startswith(('Fault', 'Timeout', 'Stopped')):
                return
        if reply['timeout']:
            with self.lock:
                if self.armed:
                    self._latch_stop_locked('Watchdog timeout while armed; acknowledge in Stop')
                self.data['state'] = 'Timeout — disarmed'
                self.data['reason'] = 'Click Acknowledge timeout while the motor is supported.'

    def _tick(self):
        self._assert_owner_thread()
        if self.bus is None or not self.data['connected'] or not getattr(self.bus, 'is_open', False):
            return
        if self.closing.is_set():
            return
        host_now = time.monotonic()

        if self.armed:
            if self.last_step_host_s is None:
                self.last_step_host_s = host_now
            gap = host_now - self.last_step_host_s
            self.last_step_host_s = host_now
            if gap > SCHEDULE_GAP_MAX_S:
                self._trip_fault(RuntimeError(
                    f'Schedule gap exceeded {SCHEDULE_GAP_MAX_S * 1000:.0f}ms'
                ))
                return
            dt = min(PER_STEP_DT_MAX_S, max(0.0, gap))
            with self.lock:
                now = self._clock_now_locked()
                intent = self._fresh_mode1_intent(now)
                current = self.latest
            if current is None:
                self._trip_loss('Fresh feedback unavailable; Stop and re-check required')
                return
            age = self._fresh_feedback_age(now)
            if age is None or age > FEEDBACK_MAX_AGE_S:
                self._trip_fault(RuntimeError('Feedback stale; Stop and re-check required'))
                return
            if self.intent is not None and self.intent_expires_at is not None and now >= self.intent_expires_at:
                self._trip_loss('Input ticket lease expired; Stop and re-check required')
                return
            if current['mode'] == 0:
                if self.deadman_active and intent is not None:
                    self._enter_foc(intent)
                    self.last_step_host_s = time.monotonic()
                    return
                if host_now - self.last_poll_host_s >= 0.050:
                    self._poll_stop()
                return
            if current['mode'] != 1:
                raise RuntimeError('Unexpected feedback mode while armed')
            if intent is None:
                self._trip_loss('Valid live input intent unavailable; Stop and re-check required')
                return
            if dt <= 0:
                return

            packet, decoded, next_v, ideal_q, joint_q = self._build_motion_packet(dt)
            reply = self._send_mode1(packet, decoded, intent, 1, 'jog')
            if self.stop_requested.is_set() or intent is not self.intent:
                return
            self._check_motion_reply(reply, joint_q)
            direction = self.settings['direction']
            with self.lock:
                if self.stop_requested.is_set() or intent is not self.intent:
                    return
                self.q_ref = joint_q
                self.q_ideal = ideal_q
                self.v_ref = next_v
                self.last_reference_raw = decoded['raw_position']
                if self.deadman_active:
                    self.data['state'] = 'Jogging'
                    self.data['reason'] = 'Manual jogging active'
                elif abs(self.v_ref) > 1e-5:
                    self.data['state'] = 'Armed · braking'
                    self.data['reason'] = 'Deadman released; braking to zero'
                else:
                    self.data['state'] = 'Armed · holding'
                    self.data['reason'] = 'Deadman released; holding position'
                max_speed = self.settings['max_rpm'] * math.tau / 60
                v2 = self.target_axes[1] * max_speed if self.deadman_active else 0.0
                v3 = self.target_axes[2] * max_speed if self.deadman_active else 0.0
                self.q_j2 = max(-math.pi / 2, min(math.pi / 2, self.q_j2 + v2 * dt))
                self.q_j3 = max(-math.pi / 2, min(math.pi / 2, self.q_j3 + v3 * dt))
                self.v_j2 = v2
                self.v_j3 = v3
            return

        if host_now - self.last_poll_host_s >= 0.050:
            self._poll_stop()

    def _process_stop_work(self):
        with self.lock:
            if not self.stop_work.is_set():
                return
            self.stop_work.clear()
        result = self._stop()
        with self.lock:
            if self.data['connected'] and self.data['state'] != 'Fault — serial close unconfirmed':
                if result:
                    self.data['state'] = 'Stopped · re-check required'
                    if self.data['reason'] == 'Select an adapter and connect.':
                        self.data['reason'] = 'Click Check connection before re-arming.'
                else:
                    self.data['state'] = 'Fault — Stop unconfirmed'
                    self.data['reason'] = 'No actual mode-0 response confirmed Stop.'

    def _dispatch_job(self, job):
        if len(job) == 5:
            generation, session, op_id, action, values = job
            ticket_generation = None
            ticket_expiry = None
        elif len(job) == 6:
            generation, session, op_id, action, values, ticket_generation = job
            ticket_expiry = None
        else:
            generation, session, op_id, action, values, ticket_generation, ticket_expiry = job
        try:
            with self.lock:
                if session not in self.sessions:
                    return
                if generation != self.generation:
                    self._record_receipt(session, op_id, 'rejected', 'Cancelled by Stop or authority change')
                    if action == 'connect' and self.bus is None and self.port_owner is None and self.owner == session:
                        self.owner = None
                    return
                if ticket_generation is not None and ticket_generation != self.generation:
                    self._record_receipt(session, op_id, 'rejected', 'Action ticket generation is stale')
                    return
                if ticket_expiry is not None:
                    now = self._clock_now_locked()
                    if now >= ticket_expiry:
                        self._record_receipt(session, op_id, 'rejected', 'Action ticket expired before dispatch')
                        if action == 'connect' and self.bus is None and self.port_owner is None and self.owner == session:
                            self.owner = None
                        return
                if action != 'connect' and session != self.owner:
                    self._record_receipt(session, op_id, 'rejected', 'Session no longer owns the bus')
                    return
                if action == 'connect' and self.owner != session:
                    self._record_receipt(session, op_id, 'rejected', 'Session no longer owns the connection request')
                    return
            self._handle_action(session, action, values)
            self._record_receipt(session, op_id, 'complete', '')
        except (ValueError, TypeError, OverflowError) as error:
            self._record_receipt(session, op_id, 'rejected', str(error))
            self._event(f'Request rejected: {error}')
            if action == 'connect' and self.bus is None and self.port_owner is None:
                with self.lock:
                    if self.owner == session:
                        self.owner = None
            with self.lock:
                if self.data['connected'] and not self.armed:
                    self.data['reason'] = str(error)[:240]
        except WriteCancelledError as error:
            self._record_receipt(session, op_id, 'rejected', str(error))
        except Exception as error:
            self._record_receipt(session, op_id, 'rejected', str(error))
            self._trip_fault(error)
        finally:
            with self.lock:
                if session in self._pending_by_session:
                    self._pending_by_session[session] = max(
                        0, self._pending_by_session[session] - 1
                    )

    def _drain_rejected_jobs(self, reason):
        while True:
            with self.lock:
                try:
                    job = self.jobs.get_nowait()
                except queue.Empty:
                    return
            if len(job) >= 3:
                _, session, op_id = job[:3]
                self._record_receipt(session, op_id, 'rejected', reason)
                with self.lock:
                    if session in self._pending_by_session:
                        self._pending_by_session[session] = max(
                            0, self._pending_by_session[session] - 1
                        )
            self.jobs.task_done()

    def _service_close(self):
        with self.lock:
            if not self.close_requested.is_set():
                return False
            request_count = self._close_requests
            self.close_requested.clear()
            self.stop_work.clear()
            self.wake.clear()
        self._drain_rejected_jobs('Session closing')
        if self.bus is not None or self.port_owner is not None:
            self._stop()
            close_ok = self._close_resource()
        else:
            close_ok = True
        if close_ok:
            self._mark_disconnected('Session closed.')
            self.closed.set()
            return True
        with self.lock:
            self.data.update(
                state='Fault — serial close unconfirmed',
                reason='Serial close failed; bus and managed ownership remain for retry.',
                connected=True,
            )
            if self._close_requests == request_count:
                self.close_requested.clear()
        self.wake.wait()
        return False

    def _loop(self):
        try:
            while True:
                try:
                    if self.close_requested.is_set():
                        if self._service_close():
                            break
                        continue
                    self._process_stop_work()
                    with self.lock:
                        try:
                            job = self.jobs.get_nowait()
                        except queue.Empty:
                            job = None
                    if job is not None:
                        self._dispatch_job(job)
                        self.jobs.task_done()
                    try:
                        self._tick()
                    except WriteCancelledError:


                        pass
                    except Exception as error:
                        self._trip_fault(error)
                    self.wake.wait(0.010)
                    self.wake.clear()
                except BaseException as error:


                    self._trip_fault(error)
                    self.wake.wait(0.010)
                    self.wake.clear()
        finally:


            try:
                if self.bus is not None or self.port_owner is not None:
                    self._stop()
                    self._close_resource()
            except BaseException:
                pass


__all__ = [
    'ArmManualSession', 'DemoSerial', 'DEFAULT_SETTINGS', 'FEEDBACK_MAX_AGE_S',
    'INGRESS_LEASE_S', 'MAX_ACCEL_RAD_S2', 'MAX_MEASURED_LEAD_RAD',
    'MAX_MEASURED_SPEED_RPM', 'MAX_QUEUE', 'MAX_RATE_RPM',
    'MAX_RECEIPTS_PER_SESSION', 'MAX_SESSIONS', 'SCHEDULE_GAP_MAX_S',
    'TICKET_EXPIRY_S', 'MAX_TICKETS', 'PORT',
]
