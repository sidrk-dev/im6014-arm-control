"""Single-owner manual authority for one fixed physical arm profile.

The owner thread is the only code that opens, writes, reads, or closes the
serial connection. HTTP and input threads may only validate bounded requests,
update authority under the lock, and wake that thread.
"""

from collections import OrderedDict, deque
from dataclasses import dataclass, field
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
from spin_5rpm import MAX_CONFIGURED_EFFORT_NM, PORT, healthy

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
MAX_RATE_RPM = 100.0
MIN_MEASURED_SPEED_MARGIN_RPM = 5.0
MEASURED_SPEED_MARGIN_RATIO = 0.2
MAX_MEASURED_SPEED_RPM = MAX_RATE_RPM * (1 + MEASURED_SPEED_MARGIN_RATIO)
MAX_MEASURED_LEAD_RAD = 0.05
DEFAULT_EFFORT_CEILING_NM = MAX_CONFIGURED_EFFORT_NM

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


@dataclass(frozen=True)
class ManualJointSpec:
    name: str
    motor_id: int


@dataclass(frozen=True)
class ManualProfile:
    profile_id: str
    joints: tuple[ManualJointSpec, ...]


@dataclass
class ManualJointRuntime:
    spec: ManualJointSpec
    settings: dict = field(default_factory=lambda: dict(DEFAULT_SETTINGS))
    calibrated: bool = False
    q_zero: float = 0.0
    latest: dict | None = None
    received_at: float | None = None
    stop_confirmed: bool = False
    stop_result: str = 'not attempted'
    stop_error: str | None = None
    q_ref: float = 0.0
    q_ideal: float = 0.0
    v_ref: float = 0.0
    last_reference_raw: int | None = None
    last_step_host_s: float | None = None
    last_poll_host_s: float = 0.0
    reference_history: deque = field(default_factory=lambda: deque(maxlen=MAX_REFERENCE_HISTORY))
    protection_fault: str | None = None
    foc_entered: bool = False


_JOINT_SPECS = (
    ManualJointSpec('J1', 0), ManualJointSpec('J2', 1), ManualJointSpec('J3', 2),
)
MANUAL_PROFILES = {
    'arm_j1': ManualProfile('arm_j1', (_JOINT_SPECS[0],)),
    'arm_j1_j2_j3': ManualProfile('arm_j1_j2_j3', _JOINT_SPECS),
}

_ACTIONS = frozenset((
    'select_profile', 'connect', 'disconnect', 'check', 'ack', 'calibrate', 'configure', 'arm', 'release'
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


def _measured_speed_limit(max_rpm):
    margin = max(MIN_MEASURED_SPEED_MARGIN_RPM, max_rpm * MEASURED_SPEED_MARGIN_RATIO)
    return min(MAX_MEASURED_SPEED_RPM, max_rpm + margin)


class DemoSerial:
    """Packet-level simulator; all calls still run through the owner thread."""

    is_open = True
    in_waiting = 26
    timeout = 0.005
    write_timeout = 0.05

    def __init__(self, initial_pos=0.0, clock=time.monotonic, motor_ids=(0,)):
        self.clock = clock
        self.data = b''
        self.states = {
            motor_id: dict(
                mode=0, timeout=1, rpm=0.0, pos=float(initial_pos), torque=0.0,
                voltage=48.0, driver_c=35, winding_c=31, errors=0, warnings=0,
            )
            for motor_id in motor_ids
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
    """One serialized authority for the selected fixed IM6014 joint profile."""

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

        self.profile_id = 'arm_j1'
        self.profile = MANUAL_PROFILES[self.profile_id]
        self.joint_states = {
            spec.name: ManualJointRuntime(spec) for spec in _JOINT_SPECS
        }
        self.armed = False
        self.stop_confirmed = False
        self.stop_latched = True
        self._stop_cause = None
        self.deadman_active = False
        self.require_neutral = True
        self.target_axes = [0.0, 0.0, 0.0]
        self.v_j2 = self.v_j3 = 0.0
        self.intent = None
        self.intent_expires_at = None

        self.settings_revision = 0
        self.checked_revision = -1
        self.q_j2 = 0.0
        self.v_j2 = 0.0
        self.q_j3 = 0.0
        self.v_j3 = 0.0
        self._last_clock_value = None
        self.clock_fault = None
        self.last_ingress_time = None
        self.last_poll_host_s = 0.0
        self.last_step_host_s = None

        self.protection_fault = None
        self.protection_history = []
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

    @property
    def configured_joints(self):
        return tuple(self.joint_states[spec.name] for spec in self.profile.joints)

    @property
    def settings(self):
        return self.joint_states['J1'].settings

    @property
    def calibrated(self):
        return self.joint_states['J1'].calibrated

    @calibrated.setter
    def calibrated(self, value):
        self.joint_states['J1'].calibrated = bool(value)

    @property
    def q_zero(self):
        return self.joint_states['J1'].q_zero

    @q_zero.setter
    def q_zero(self, value):
        self.joint_states['J1'].q_zero = float(value)

    @property
    def latest(self):
        return self.joint_states['J1'].latest

    @latest.setter
    def latest(self, value):
        self.joint_states['J1'].latest = value

    @property
    def last_reply_time(self):
        return self.joint_states['J1'].received_at

    @last_reply_time.setter
    def last_reply_time(self, value):
        self.joint_states['J1'].received_at = value

    @property
    def q_ref(self):
        return self.joint_states['J1'].q_ref

    @q_ref.setter
    def q_ref(self, value):
        self.joint_states['J1'].q_ref = value

    @property
    def q_ideal(self):
        return self.joint_states['J1'].q_ideal

    @q_ideal.setter
    def q_ideal(self, value):
        self.joint_states['J1'].q_ideal = value

    @property
    def v_ref(self):
        return self.joint_states['J1'].v_ref

    @v_ref.setter
    def v_ref(self, value):
        self.joint_states['J1'].v_ref = value

    @property
    def last_reference_raw(self):
        return self.joint_states['J1'].last_reference_raw

    @last_reference_raw.setter
    def last_reference_raw(self, value):
        self.joint_states['J1'].last_reference_raw = value

    @property
    def reference_history(self):
        return self.joint_states['J1'].reference_history

    def _select_profile(self, profile_id):
        if profile_id not in MANUAL_PROFILES:
            raise ValueError('Unknown arm profile')
        with self.lock:
            if self.bus is not None or self.port_owner is not None or self.data['connected']:
                raise ValueError('Disconnect before changing arm profile')
            self.profile_id = profile_id
            self.profile = MANUAL_PROFILES[profile_id]
            self._reset_joint_authority(clear_settings=False)

    def _reset_joint_authority(self, *, clear_settings):
        for joint in self.joint_states.values():
            if clear_settings:
                joint.settings = dict(DEFAULT_SETTINGS)
            joint.calibrated = False
            joint.q_zero = 0.0
            joint.latest = None
            joint.received_at = None
            joint.stop_confirmed = False
            joint.stop_result = 'not attempted'
            joint.stop_error = None
            joint.q_ref = joint.q_ideal = joint.v_ref = 0.0
            joint.last_reference_raw = None
            joint.last_step_host_s = None
            joint.last_poll_host_s = 0.0
            joint.reference_history.clear()
            joint.foc_entered = False

    def _joint(self, name=None):
        name = 'J1' if name is None else name
        if type(name) is not str or name not in self.joint_states:
            raise ValueError('Joint must be J1, J2, or J3')
        if name not in {spec.name for spec in self.profile.joints}:
            raise ValueError(f'{name} is not configured in {self.profile_id}')
        return self.joint_states[name]

    def _joint_for_id(self, motor_id):
        for joint in self.configured_joints:
            if joint.spec.motor_id == motor_id:
                return joint
        raise RuntimeError(f'Motor ID {motor_id} is not in the selected profile')

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
        if self._stop_cause is None:
            self._stop_cause = str(reason)[:240]
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
        for joint in self.joint_states.values():
            joint.v_ref = 0.0
            joint.foc_entered = False
        for joint in self.configured_joints:
            joint.stop_confirmed = False
            joint.stop_result = 'pending'
            joint.stop_error = None
        self.intent = None
        self.intent_expires_at = None
        self.tickets.clear()
        if self.data['connected'] and self.data['state'] != 'Fault — serial close unconfirmed':
            if not self.data['state'].startswith('Fault —'):
                self.data['state'] = 'Stopped · re-check required'
            self.data['reason'] = self._stop_cause
        self._event('STOP LATCHED: ' + str(reason)[:180])
        self.wake.set()

    def _record_overload(self, reason, joint=None):
        with self.lock:
            self.protection_fault = str(reason)[:240]
            if joint is not None:
                joint.protection_fault = self.protection_fault
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

    def edit_parameters(self, session, values, joint='J1'):
        """Save finite drafts without changing applied settings or motion authority."""
        if not isinstance(values, dict) or not values or not set(values) <= set(DEFAULT_SETTINGS):
            raise ValueError('Unknown parameter draft fields')
        draft = {key: _finite_number(value, key) for key, value in values.items()}
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Unknown session')
            if type(joint) is not str or joint not in self.joint_states:
                raise ValueError('Joint must be J1, J2, or J3')
            drafts = self.parameter_drafts.setdefault(session, {})
            drafts[joint] = draft
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
        if action == 'select_profile':
            profile_id = v.get('profile_id')
            if set(v) != {'profile_id'} or type(profile_id) is not str or profile_id not in MANUAL_PROFILES:
                raise ValueError('Select a known fixed arm profile')
            return {'profile_id': profile_id}
        if action == 'connect':
            demo = v.get('demo')
            port = v.get('port', '')
            baud = v.get('baud', 6000000)
            motor_id = v.get('id', 0)
            profile_id = v.get('profile_id', 'arm_j1')
            if type(demo) is not bool:
                raise ValueError('demo must be an exact bool')
            if type(motor_id) is not int or motor_id != 0:
                raise ValueError('The legacy id field accepts ID 0 only; select a fixed profile for more joints')
            if type(profile_id) is not str or profile_id not in MANUAL_PROFILES:
                raise ValueError('Unknown arm profile')
            support_confirmed = v.get('support_confirmed') is True
            if not demo and profile_id == 'arm_j1_j2_j3' and not support_confirmed:
                raise ValueError('Physical three-joint Connect requires confirmation that the arm is supported')
            if type(baud) is not int or baud not in (4000000, 6000000):
                raise ValueError('Baud must be integer 4000000 or 6000000')
            if type(port) is not str:
                raise ValueError('Port must be a string')
            if demo:
                return {'demo': True, 'port': '', 'baud': baud, 'id': 0,
                        'profile_id': profile_id, 'support_confirmed': support_confirmed}
            if not self.physical_opt_in:
                raise ValueError('Physical mode requires physical_opt_in=True')
            if baud != 6000000:
                raise ValueError('Physical mode restricted to baud 6000000')
            if not port.strip():
                raise ValueError('Physical port must be a non-empty string')
            if len(port) > 240:
                raise ValueError('Physical port path is too long')
            return {'demo': False, 'port': port.strip(), 'baud': 6000000, 'id': 0,
                    'profile_id': profile_id, 'support_confirmed': support_confirmed}
        if action in ('disconnect', 'check', 'release'):
            if v:
                raise ValueError(f'{action} does not accept values')
            return {}
        if action in ('ack', 'arm'):
            if v.get('supported') is not True:
                raise ValueError('Supported acknowledgment must be the exact bool true')
            return {'supported': True}
        if action == 'calibrate':
            joint_name = v.get('joint', 'J1')
            selected = self._joint(joint_name)
            known = _finite_number(v.get('known_angle_deg'), 'known_angle_deg')
            direction = v.get('direction')
            lower = _finite_number(v.get('lower_deg'), 'lower_deg')
            upper = _finite_number(v.get('upper_deg'), 'upper_deg')
            if type(direction) is not int or direction not in (-1, 1):
                raise ValueError('Direction must be the integer +1 or -1')
            if selected.spec.name == 'J1' and (lower < -90.0 or upper > 90.0):
                raise ValueError('J1 travel must remain within the existing -90 to +90 degree wire limit')
            if lower >= upper:
                raise ValueError('Travel limits must be ordered with lower < upper')
            if not lower <= known <= upper:
                raise ValueError('known_angle_deg must be within the travel limits')
            return {
                'joint': selected.spec.name,
                'known_angle_deg': known, 'direction': direction,
                'lower_deg': lower, 'upper_deg': upper,
            }
        if action == 'configure':
            selected = self._joint(v.get('joint', 'J1'))
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
            return {
                'joint': selected.spec.name,
                'kp': kp, 'kd': kd, 'max_rpm': max_rpm, 'effort_limit_nm': effort,
            }
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
                if action == 'connect' and (self.bus is not None or self.port_owner is not None):
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
                raise ValueError('Motion-intent flag must be an exact bool')
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
            moving = any(abs(axis) > 1e-4 for axis in parsed_axes)
            if deadman != moving:
                raise ValueError('Motion intent must match nonzero axes')
            neutral_release = not deadman and not moving


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

            if self.require_neutral and not neutral_release:
                raise ValueError('Neutral input required after arming')
            if neutral_release:
                self.require_neutral = False

            if moving:
                self.deadman_active = True
                self.target_axes = parsed_axes
                self.intent = {
                    'session': session, 'ticket': ticket, 'seq': seq,
                    'device': device_name, 'generation': self.generation,
                }
                self.intent_expires_at = record['expires_at']
                self.last_ingress_time = now
            else:
                self.deadman_active = False
                self.target_axes = [0.0, 0.0, 0.0]
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
            source = ('demo' if self.data['demo'] else 'physical') if self.data['connected'] else 'unknown'
            joints = []
            ages = []
            for name in ('J1', 'J2', 'J3'):
                runtime = self.joint_states[name]
                configured = name in {spec.name for spec in self.profile.joints}
                if not configured:
                    joints.append({
                        'name': name, 'source': 'virtual',
                        'position_rad': self.q_j2 if name == 'J2' else self.q_j3 if name == 'J3' else None,
                        'velocity_rad_s': self.v_j2 if name == 'J2' else self.v_j3 if name == 'J3' else None,
                        'configured': False, 'calibrated': False,
                    })
                    continue
                age = None
                if now is not None and runtime.received_at is not None:
                    age = max(0.0, now - runtime.received_at)
                ages.append(age)
                latest = runtime.latest if self.data['connected'] else None
                if latest is not None:
                    latest_view = self._sanitize_feedback(latest)
                    latest_view['source'] = source
                else:
                    latest_view = None
                calibrated = bool(runtime.calibrated and self.data['connected'])
                position = (runtime.settings['direction'] * (latest['position_rad'] - runtime.q_zero)
                            if calibrated and latest is not None else None)
                velocity = (runtime.settings['direction'] * latest['rpm'] * math.tau / 60
                            if calibrated and latest is not None else None)
                unhealthy = bool(latest is not None and (
                    latest['errors'] or latest['warnings'] or latest['timeout']
                    or not 24 <= latest['voltage'] <= 72
                    or not -10 <= latest['driver_c'] <= 70
                    or latest['winding_c'] > 90
                    or abs(latest['torque_nm']) > runtime.settings['effort_limit_nm']
                ))
                joints.append({
                    'name': name, 'id': runtime.spec.motor_id, 'source': source,
                    'configured': True, 'calibrated': calibrated,
                    'position_rad': position, 'velocity_rad_s': velocity,
                    'mode': latest['mode'] if latest else None,
                    'timeout': latest['timeout'] if latest else None,
                    'voltage': latest['voltage'] if latest else None,
                    'driver_temp': latest['driver_c'] if latest else None,
                    'winding_temp': latest['winding_c'] if latest else None,
                    'errors': latest['errors'] if latest else None,
                    'warnings': latest['warnings'] if latest else None,
                    'torque_nm': latest['torque_nm'] if latest else None,
                    'feedback_age_s': age,
                    'stop_confirmed': runtime.stop_confirmed,
                    'stop_result': runtime.stop_result,
                    'stop_error': runtime.stop_error,
                    'protection_fault': runtime.protection_fault,
                    'health': ('missing' if latest is None else
                               'stale' if age is None or age > FEEDBACK_MAX_AGE_S else
                               'fault' if unhealthy else
                               'healthy'),
                    'settings': dict(runtime.settings),
                    'latest': latest_view,
                })
            configured = [joint for joint in joints if joint.get('configured')]
            all_fresh = bool(configured) and all(joint['feedback_age_s'] is not None for joint in configured)
            age = max(ages) if ages and all_fresh else None
            calibrated = bool(self.data['connected'] and configured
                              and all(joint['calibrated'] for joint in configured))
            latest = self._sanitize_feedback(self.latest) if self.latest is not None and self.data['connected'] else None
            if latest is not None:
                latest['source'] = source
            j1 = next(joint for joint in joints if joint['name'] == 'J1')
            return {
                'state': self.data['state'], 'reason': self.data['reason'],
                'connected': bool(self.data['connected']), 'demo': bool(self.data['demo']),
                'profile_id': self.profile_id,
                'profiles': [{'id': profile.profile_id, 'joints': [spec.name for spec in profile.joints]}
                             for profile in MANUAL_PROFILES.values()],
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
        joint = self._joint_for_id(motor_id)
        with self.lock:
            joint.latest = self._sanitize_feedback(result)
            try:
                joint.received_at = self._clock_now_locked()
            except ValueError:
                joint.received_at = None
        return result

    def _record_stop_result(self, confirmed):
        with self.lock:
            self.stop_confirmed = bool(confirmed and all(joint.stop_confirmed for joint in self.configured_joints)
                                       and self.data['connected'])
            self.data['stop_confirmed'] = self.stop_confirmed

    def _stop(self):
        self._assert_owner_thread()
        with self.lock:
            self.armed = False
            self.data['armed'] = False
            self.deadman_active = False
            self.require_neutral = True
            self.target_axes = [0.0, 0.0, 0.0]
            self.intent = None
            self.intent_expires_at = None
        for joint in self.configured_joints:
            joint.stop_confirmed = False
            joint.stop_result = 'pending'
            joint.stop_error = None
        if self.bus is None or not getattr(self.bus, 'is_open', False):
            self._record_stop_result(False)
            return False
        if not self.data['demo'] and not (
            self.port_owner is not None and getattr(self.port_owner, 'verified', False)
        ):
            self._record_stop_result(False)
            return False
        failures = {}
        for _ in range(3):
            for joint in self.configured_joints:
                if joint.stop_confirmed:
                    continue
                try:
                    reply = self._transact_raw(
                        command(joint.spec.motor_id, 0, 0, 0, 0, True),
                        motor_id=joint.spec.motor_id,
                    )
                    if reply['mode'] != 0:
                        raise RuntimeError('Motor has not confirmed mode-0 Stop')
                    joint.stop_confirmed = True
                    joint.stop_result = 'confirmed'
                    joint.stop_error = None
                    failures.pop(joint.spec.name, None)
                    joint.last_poll_host_s = time.monotonic()
                except Exception as error:
                    failures[joint.spec.name] = error
                    joint.stop_confirmed = False
                    joint.stop_result = 'unconfirmed'
                    joint.stop_error = (str(error) or type(error).__name__)[:160]
        self.last_poll_host_s = time.monotonic()
        result = all(joint.stop_confirmed for joint in self.configured_joints)
        with self.lock:
            self.stop_confirmed = bool(result and self.data['connected'])
            self.data['stop_confirmed'] = self.stop_confirmed
        if not result:
            summary = '; '.join(f'{name}: {error}' for name, error in failures.items())
            self._event('STOP NOT CONFIRMED: ' + summary[:200])
        return result

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
            stop_outcomes = {
                name: (joint.stop_confirmed, joint.stop_result, joint.stop_error)
                for name, joint in self.joint_states.items()
            }
            self._reset_joint_authority(clear_settings=False)
            for name, (confirmed, result, error) in stop_outcomes.items():
                joint = self.joint_states[name]
                joint.stop_confirmed = confirmed
                joint.stop_result = result
                joint.stop_error = error
            self.armed = False
            self.deadman_active = False
            self.require_neutral = True
            self.target_axes = [0.0, 0.0, 0.0]
            self.intent = None
            self.intent_expires_at = None
            self.current_device = None
            self.last_seq = -1
            self.v_j2 = self.v_j3 = 0.0
            self.last_step_host_s = None
            self.stop_confirmed = False
            self.data.update(
                connected=False, demo=False, calibrated=False, armed=False,
                stop_confirmed=False, state='Disconnected', reason=str(reason)[:240],
                port='',
            )

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

    def _validate_feedback(self, reply, mode, *, allow_timeout=False, joint=None):
        joint = self._joint('J1') if joint is None else joint
        if reply.get('id') != joint.spec.motor_id or reply.get('mode') != mode:
            raise RuntimeError(f'Unexpected motor ID or mode in feedback: {reply.get("mode")}')
        if reply.get('errors') or reply.get('warnings'):
            self._record_overload(f'{joint.spec.name} reported errors or warnings', joint)
            raise RuntimeError('Motor reports errors or warnings')
        if reply.get('timeout') and not allow_timeout:
            raise RuntimeError('Motor watchdog timeout is active')
        if not 24 <= reply.get('voltage', -math.inf) <= 72:
            raise RuntimeError('Motor voltage is outside the allowed range')
        if not -10 <= reply.get('driver_c', math.inf) <= 70:
            self._record_overload(f'{joint.spec.name} driver temperature is outside the allowed range', joint)
            raise RuntimeError('Driver temperature is outside the allowed range')
        if reply.get('winding_c', math.inf) > 90:
            self._record_overload(f'{joint.spec.name} winding temperature exceeds 90 C', joint)
            raise RuntimeError('Winding temperature exceeds 90 C')
        if abs(reply.get('torque_nm', math.inf)) > joint.settings['effort_limit_nm']:
            self._record_overload(f'{joint.spec.name} measured motor effort exceeds its configured ceiling', joint)
            raise RuntimeError('Measured torque exceeds the configured effort ceiling')
        healthy(
            reply, mode, allow_timeout=allow_timeout,
            torque_limit_nm=joint.settings['effort_limit_nm'],
        )
        return True

    def _fresh_feedback_age(self, now=None, joint=None):
        joint = self._joint('J1') if joint is None else joint
        if now is None:
            with self.lock:
                now = self._clock_now_locked()
        if joint.received_at is None:
            return None
        age = now - joint.received_at
        if not math.isfinite(age) or age < 0:
            return None
        return age

    def _joint_position_in_limits(self, joint, reply):
        if not joint.calibrated or reply is None:
            return False
        measured = joint.settings['direction'] * (reply['position_rad'] - joint.q_zero)
        return math.radians(joint.settings['lower_deg']) <= measured <= math.radians(joint.settings['upper_deg'])

    def _require_fresh_stop(self):
        if not self.data['connected'] or self.bus is None:
            raise ValueError('Connect first')
        if self.armed or not self.stop_confirmed:
            raise ValueError('Requires confirmed Stop and disarmed state')
        now = self._clock_now_locked()
        for joint in self.configured_joints:
            age = self._fresh_feedback_age(now, joint)
            if age is None or age > FEEDBACK_MAX_AGE_S:
                raise ValueError(f'{joint.spec.name} feedback stale; check connection again')
            if not joint.stop_confirmed or joint.latest is None or joint.latest['mode'] != 0:
                raise ValueError(f'{joint.spec.name} must have a confirmed mode-0 Stop')
            self._validate_feedback(joint.latest, 0, joint=joint)

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
            missing = ', '.join(f'{joint.spec.name} (ID {joint.spec.motor_id})'
                                for joint in self.configured_joints if not joint.stop_confirmed)
            raise RuntimeError(f'Cannot verify Stop for {missing}; check unique motor IDs, wiring, 6 Mbaud and motor power')
        replies = {}
        for joint in self.configured_joints:
            reply = self._transact_raw(
                command(joint.spec.motor_id, 0, 0, 0, 0, True),
                motor_id=joint.spec.motor_id,
            )
            self._validate_feedback(reply, 0, allow_timeout=True, joint=joint)
            replies[joint.spec.name] = reply
        now = self._clock_now_locked()
        for joint in self.configured_joints:
            age = self._fresh_feedback_age(now, joint)
            if age is None or age > FEEDBACK_MAX_AGE_S:
                raise RuntimeError(f'Fresh {joint.spec.name} mode-0 feedback was not available')
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
            self._stop_cause = None
            self.stop_confirmed = all(joint.stop_confirmed for joint in self.configured_joints)
            self.data['stop_confirmed'] = self.stop_confirmed
            timeouts = [joint.spec.name for joint in self.configured_joints if replies[joint.spec.name]['timeout']]
            if timeouts:
                self.data['state'] = 'Timeout — disarmed'
                self.data['reason'] = 'Startup timeout on ' + ', '.join(timeouts) + '; acknowledge only while each is in Stop.'
            else:
                self.data['state'] = 'Ready · Stop'
                self.data['reason'] = 'All configured motors returned fresh healthy mode-0 feedback.'

    def _ack(self, supported):
        self._assert_owner_thread()
        if supported is not True:
            raise ValueError('Supported acknowledgment required')
        if self.bus is None or not self.stop_confirmed:
            raise ValueError('Check a connected Stop state first')
        if self.stop_latched or self.stop_requested.is_set():
            raise ValueError('Check again after Stop before acknowledging the watchdog')
        if self.armed:
            raise ValueError('Acknowledge requires actual mode-0 Stop')
        now = self._clock_now_locked()
        timeout_joints = []
        for joint in self.configured_joints:
            age = self._fresh_feedback_age(now, joint)
            if age is None or age > FEEDBACK_MAX_AGE_S or joint.latest is None:
                raise ValueError(f'{joint.spec.name} feedback stale; check connection again')
            if joint.latest.get('mode') != 0 or not joint.stop_confirmed:
                raise ValueError(f'{joint.spec.name} must have actual mode-0 Stop')
            self._validate_feedback(joint.latest, 0, allow_timeout=True, joint=joint)
            if joint.latest.get('timeout') in (True, 1):
                timeout_joints.append(joint)
        if not timeout_joints:
            raise ValueError('There is no active startup timeout to acknowledge')

        start_generation = self.generation
        failures = []
        for joint in timeout_joints:
            no_watchdog = command(joint.spec.motor_id, 0, 0, 0, 0, False)
            watchdog = command(joint.spec.motor_id, 0, 0, 0, 0, True)
            try:
                reply = self._transact_raw(no_watchdog, motor_id=joint.spec.motor_id)
                self._validate_feedback(reply, 0, allow_timeout=True, joint=joint)
            except Exception as error:
                failures.append(f'{joint.spec.name} acknowledge: {str(error) or type(error).__name__}')
            finally:
                try:
                    restored = self._transact_raw(watchdog, motor_id=joint.spec.motor_id)
                    self._validate_feedback(restored, 0, allow_timeout=False, joint=joint)
                    joint.stop_confirmed = True
                    joint.stop_result = 'confirmed'
                except Exception as error:
                    failures.append(f'{joint.spec.name} watchdog restore: {str(error) or type(error).__name__}')
                    joint.stop_confirmed = False
                    joint.stop_result = 'unconfirmed'
                    joint.stop_error = str(error)[:160]
        for joint in self.configured_joints:
            age = self._fresh_feedback_age(now=None, joint=joint)
            if joint.latest is None or joint.latest.get('mode') != 0 or joint.latest.get('timeout') or age is None or age > FEEDBACK_MAX_AGE_S:
                failures.append(f'{joint.spec.name} did not return fresh mode-0 feedback with watchdog active')
                joint.stop_confirmed = False
        if failures:
            self._record_stop_result(False)
            raise RuntimeError('Watchdog acknowledgment failed; mode-0 watchdog restore attempted: '
                               + '; '.join(failures)[:240])
        self._clock_now_locked()
        with self.lock:
            if self.generation != start_generation:
                raise WriteCancelledError('Stop arrived during watchdog acknowledgment')
            self.stop_confirmed = all(joint.stop_confirmed for joint in self.configured_joints)
            self.data['stop_confirmed'] = self.stop_confirmed
            if not self.stop_confirmed:
                raise RuntimeError('Not every configured motor has confirmed Stop')
            self.checked_revision = self.settings_revision
            self.stop_latched = False
            self.stop_requested.clear()
            self.stop_work.clear()
            self._stop_cause = None
            self.data['state'] = 'Ready · Stop'
            self.data['reason'] = 'Startup timeout acknowledged in Stop; firmware watchdog restored.'
            self._event('Startup timeout acknowledged in Stop; firmware watchdog restored.')

    def _handle_action(self, session, action, values):
        self._assert_owner_thread()
        with self.lock:
            if session not in self.sessions:
                raise ValueError('Session retired before dispatch')
            if session != self.owner and not (action == 'select_profile' and self.owner is None):
                raise ValueError('Session is no longer the bus owner')

        if action == 'select_profile':
            self._select_profile(values['profile_id'])
            self._event(f'Arm profile selected: {self.profile_id}')
            return

        if action == 'connect':
            if self.bus is not None or self.port_owner is not None:
                raise ValueError('Disconnect before changing adapter')
            self._select_profile(values['profile_id'])
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
                self.stop_confirmed = False
                self.current_device = None
                self.last_seq = -1
                self.tickets.clear()
                self.deadman_active = False
                self.require_neutral = True
            if demo:
                self.bus = DemoSerial(
                    clock=self.clock,
                    motor_ids=tuple(joint.spec.motor_id for joint in self.configured_joints),
                )
            else:
                config = PortConfig(port, baud)
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
            ids = ','.join(str(joint.spec.motor_id) for joint in self.configured_joints)
            self._event(f'{"DEMO" if demo else "REAL"} port open: {port} · IDs {ids} · {baud}')
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
            joint = self._joint(values.get('joint', 'J1'))
            reply = self._transact_raw(
                command(joint.spec.motor_id, 0, 0, 0, 0, True),
                motor_id=joint.spec.motor_id,
            )
            self._validate_feedback(reply, 0, joint=joint)
            known_rad = math.radians(values['known_angle_deg'])
            direction = values['direction']
            q_zero = reply['position_rad'] - direction * known_rad
            lower_motor = q_zero + direction * math.radians(values['lower_deg'])
            upper_motor = q_zero + direction * math.radians(values['upper_deg'])
            if min(lower_motor, upper_motor) < POSITION_MIN or max(lower_motor, upper_motor) > POSITION_MAX:
                raise ValueError('Applied travel exceeds the actuator position codec envelope')
            with self.lock:
                joint.q_zero = q_zero
                joint.settings.update(
                    known_angle_deg=values['known_angle_deg'], direction=direction,
                    lower_deg=values['lower_deg'], upper_deg=values['upper_deg'],
                )
                joint.calibrated = True
                self._invalidate_settings('Calibration changed')
                self._event(
                    f'{joint.spec.name} calibrated: known={values["known_angle_deg"]}°, dir={direction}, '
                    f'limits=[{values["lower_deg"]}°, {values["upper_deg"]}°]'
                )
            return
        if action == 'configure':
            self._require_fresh_stop()
            joint = self._joint(values.get('joint', 'J1'))
            with self.lock:
                joint.settings.update({key: value for key, value in values.items() if key != 'joint'})
                self._invalidate_settings('Settings changed')
                self._event(
                    f'{joint.spec.name} configured: kp={values["kp"]}, kd={values["kd"]}, '
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
                uncalibrated = [joint.spec.name for joint in self.configured_joints if not joint.calibrated]
                if uncalibrated:
                    raise ValueError('Arm requires current zero and applied travel for: ' + ', '.join(uncalibrated))
                now = self._clock_now_locked()
                for joint in self.configured_joints:
                    age = self._fresh_feedback_age(now, joint)
                    if age is None or age > FEEDBACK_MAX_AGE_S:
                        raise ValueError(f'{joint.spec.name} feedback stale; check connection again')
                    if not joint.stop_confirmed or joint.latest is None or joint.latest.get('mode') != 0:
                        raise ValueError(f'{joint.spec.name} must confirm actual mode-0 Stop before Enable')
                    self._validate_feedback(joint.latest, 0, joint=joint)
                    if not self._joint_position_in_limits(joint, joint.latest):
                        raise ValueError(f'{joint.spec.name} measured position is outside its applied travel')
                self.armed = True
                self.data['armed'] = True
                self.current_device = None
                self.deadman_active = False
                self.require_neutral = True
                self.intent = None
                self.intent_expires_at = None
                self.last_step_host_s = time.monotonic()
                for joint in self.configured_joints:
                    joint.q_ref = joint.q_ideal = joint.v_ref = 0.0
                    joint.last_reference_raw = None
                    joint.last_step_host_s = time.monotonic()
                    joint.foc_entered = False
                self.data['state'] = 'Armed · motor stopped'
                self.data['reason'] = 'Armed for manual jogging. Release controls to neutral once, then move an input.'
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

    def _mode1_authority_matches_locked(self, intent):
        current = self.intent
        return (
            intent is not None and current is not None
            and intent['session'] == current['session'] == self.owner
            and intent['session'] in self.sessions
            and intent['generation'] == current['generation'] == self.generation
            and intent['device'] == current['device'] == self.current_device
        )

    def _mode1_gate(self, joint, intent, expected_mode, kind, expected_axis):
        with self.lock:
            if (self.closing.is_set() or self.close_requested.is_set()
                    or self.stop_requested.is_set() or self.stop_latched):
                return False
            if not self.armed or not self.data['connected'] or self.bus is None:
                return False
            if not self._mode1_authority_matches_locked(intent):
                self._latch_stop_locked('Manual input authority changed; Stop and re-check required')
                return False
            if (any(not item.calibrated for item in self.configured_joints)
                    or self.checked_revision != self.settings_revision):
                return False
            try:
                now = self._clock_now_locked()
            except ValueError:
                return False
            if self.intent_expires_at is None or now >= self.intent_expires_at:
                self._latch_stop_locked('Input ticket lease expired; Stop and re-check required')
                return False
            if kind == 'foc-entry' and not self.deadman_active:
                self._latch_stop_locked('Neutral input interrupted FOC entry; Stop and re-check required')
                return False
            if expected_axis is not None:
                axis_index = next(
                    (index for index, item in enumerate(self.configured_joints) if item is joint),
                    None,
                )
                if axis_index is None or self.target_axes[axis_index] != expected_axis:
                    return False
            for item in self.configured_joints:
                if item.latest is None or item.received_at is None:
                    self._latch_stop_locked(f'{item.spec.name} feedback unavailable; Stop and re-check required')
                    return False
                age = now - item.received_at
                if age < 0 or age > FEEDBACK_MAX_AGE_S:
                    self._latch_stop_locked(f'{item.spec.name} feedback stale; Stop and re-check required')
                    return False
                item_mode = expected_mode if item is joint else (1 if item.foc_entered else 0)
                if item_mode not in (0, 1) or item.latest.get('mode') != item_mode:
                    self._latch_stop_locked(f'{item.spec.name} feedback mode changed; Stop and re-check required')
                    return False
                try:
                    self._validate_feedback(item.latest, item_mode, joint=item)
                except Exception:
                    self._latch_stop_locked(f'{item.spec.name} feedback health gate failed; Stop and re-check required')
                    return False
                if not self._joint_position_in_limits(item, item.latest):
                    self._latch_stop_locked(f'{item.spec.name} position is outside applied travel; Stop and re-check required')
                    return False
            return True

    def _send_mode1(self, joint, packet, decoded, intent, expected_mode, kind, expected_axis=None):
        gate_lock_held = False

        def before_write():
            nonlocal gate_lock_held
            self.lock.acquire()
            gate_lock_held = True
            if self._mode1_gate(joint, intent, expected_mode, kind, expected_axis):
                return True
            self.lock.release()
            gate_lock_held = False
            return False

        def on_write(result):
            nonlocal gate_lock_held
            try:
                if result is WriteResult.COMPLETE:
                    host_time = time.monotonic()
                    joint_position = joint.settings['direction'] * (decoded['position_rad'] - joint.q_zero)
                    joint.reference_history.append({
                        'time_s': host_time,
                        'position_rad': joint_position,
                        'raw_position': decoded['raw_position'],
                        'rpm': joint.settings['direction'] * decoded['rpm'],
                        'kind': kind,
                    })
            finally:
                if gate_lock_held:
                    self.lock.release()
                    gate_lock_held = False

        try:
            return self._transact_raw(
                packet, motor_id=joint.spec.motor_id, before_write=before_write, on_write=on_write
            )
        finally:
            if gate_lock_held:
                self.lock.release()
                gate_lock_held = False

    def _enter_foc(self, joint, intent):
        if joint.latest is None or joint.latest.get('mode') != 0:
            raise RuntimeError('FOC entry requires a current mode-0 reply')
        raw_position = position_to_raw(joint.latest['position_rad'])
        packet = command_with_raw_position(
            joint.spec.motor_id, raw_position, mode=1, rpm=0.0, kd=0.0, torque=0.0,
            watchdog=True, kp=0.0, raw_rpm=0,
        )
        decoded = decode_command(packet)
        first = self._send_mode1(joint, packet, decoded, intent, 0, 'foc-entry')
        if first.get('mode') not in (0, 1):
            raise RuntimeError('FOC entry reply must show Stop or FOC')
        self._validate_feedback(first, first['mode'], joint=joint)
        if self.stop_requested.is_set():
            raise WriteCancelledError('Stop arrived during FOC entry')
        second = self._send_mode1(joint, packet, decoded, intent, first['mode'], 'foc-entry')
        if second.get('mode') != 1:
            raise RuntimeError('FOC entry second reply must confirm mode 1')
        self._validate_feedback(second, 1, joint=joint)
        direction = joint.settings['direction']
        joint_q = direction * (decoded['position_rad'] - joint.q_zero)
        with self.lock:
            joint.q_ref = joint_q
            joint.q_ideal = joint_q
            joint.v_ref = 0.0
            joint.last_reference_raw = decoded['raw_position']
            if self.stop_requested.is_set():
                return
            if not self._mode1_authority_matches_locked(intent):
                self._latch_stop_locked('Manual input authority changed during FOC entry; Stop required')
                return
            try:
                now = self._clock_now_locked()
            except ValueError:
                return
            if self.intent_expires_at is None or now >= self.intent_expires_at:
                self._latch_stop_locked('Input ticket lease expired during FOC entry; Stop required')
                return
            if (not self.deadman_active and any(
                    item is not joint and not item.foc_entered for item in self.configured_joints
            )):
                self._latch_stop_locked('Neutral input interrupted FOC entry; Stop and re-check required')
                return
            joint.foc_entered = True
            joint.stop_confirmed = False
            self.stop_confirmed = False
            self.data['stop_confirmed'] = False
            self.data['state'] = 'Jogging'
            self.data['reason'] = 'Manual jogging active'

    def _candidate_position(self, joint, ideal_joint_q, previous_raw, previous_joint_q, step_limit):
        direction = joint.settings['direction']
        q_min = math.radians(joint.settings['lower_deg'])
        q_max = math.radians(joint.settings['upper_deg'])
        motor_low, motor_high = (
            (joint.q_zero + q_min, joint.q_zero + q_max)
            if direction == 1 else
            (joint.q_zero - q_max, joint.q_zero - q_min)
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
        ideal_motor = joint.q_zero + direction * ideal_joint_q
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
            joint_q = direction * (motor_q - joint.q_zero)
            if not q_min <= joint_q <= q_max:
                continue
            if abs(joint_q - previous_joint_q) > step_limit:
                continue
            possible.append((abs(joint_q - ideal_joint_q), raw, joint_q))
        if not possible:
            return previous_raw, previous_joint_q
        _, selected_raw, selected_q = min(possible)
        return int(selected_raw), selected_q

    def _build_motion_packet(self, joint, axis, dt):
        settings = joint.settings
        direction = settings['direction']
        max_rpm = settings['max_rpm']
        max_speed = max_rpm * math.tau / 60
        q_min = math.radians(settings['lower_deg'])
        q_max = math.radians(settings['upper_deg'])
        if joint.last_reference_raw is None:
            joint.last_reference_raw = position_to_raw(joint.latest['position_rad'])
            joint.q_ref = direction * (raw_to_position(joint.last_reference_raw) - joint.q_zero)
            joint.q_ideal = joint.q_ref
            joint.v_ref = 0.0

        if self.deadman_active:
            v_target = axis * max_speed
        else:
            v_target = 0.0
        a_max = MAX_ACCEL_RAD_S2
        if joint.v_ref > 0:
            available = q_max - joint.q_ref
            if joint.v_ref * joint.v_ref / (2 * a_max) >= available:
                v_target = min(v_target, math.sqrt(max(0.0, 2 * a_max * available)))
        elif joint.v_ref < 0:
            available = joint.q_ref - q_min
            if joint.v_ref * joint.v_ref / (2 * a_max) >= available:
                v_target = max(v_target, -math.sqrt(max(0.0, 2 * a_max * available)))
        delta_v = a_max * dt
        if v_target > joint.v_ref:
            next_v = min(v_target, joint.v_ref + delta_v)
        else:
            next_v = max(v_target, joint.v_ref - delta_v)
        next_v = max(-max_speed, min(max_speed, next_v))
        ideal_q = max(q_min, min(q_max, joint.q_ideal + next_v * dt))

        raw_rpm_limit = math.floor(max_rpm / 60 * RATIO * 64)
        wanted_rpm = direction * next_v * 60 / math.tau
        raw_rpm = math.trunc(wanted_rpm / 60 * RATIO * 64)
        raw_rpm = max(-raw_rpm_limit, min(raw_rpm_limit, raw_rpm))
        decoded_rpm = raw_rpm / 64 / RATIO * 60

        max_position_step = max_speed * dt
        accel_position_step = abs(next_v) * dt
        step_limit = min(max_position_step, accel_position_step)
        raw_position, decoded_joint_q = self._candidate_position(
            joint, ideal_q, joint.last_reference_raw, joint.q_ref, step_limit
        )
        packet = command_with_raw_position(
            joint.spec.motor_id, raw_position, mode=1, rpm=decoded_rpm,
            kd=settings['kd'], torque=0.0, watchdog=True,
            kp=settings['kp'], raw_rpm=raw_rpm,
        )
        decoded = decode_command(packet)
        decoded_joint_q = direction * (decoded['position_rad'] - joint.q_zero)
        decoded_joint_v = direction * decoded['rpm'] * math.tau / 60
        if not q_min <= decoded_joint_q <= q_max:
            raise RuntimeError('Decoded position exceeds the inclusive travel limits')
        if abs(decoded_joint_q - joint.q_ref) > step_limit:
            raise RuntimeError('Decoded packet step exceeds configured speed and capped dt')
        if abs(decoded['rpm']) > max_rpm:
            raise RuntimeError('Decoded packet velocity exceeds configured maximum')

        current_q = direction * (joint.latest['position_rad'] - joint.q_zero)
        current_v = direction * joint.latest['rpm'] * math.tau / 60
        predicted_effort = abs(
            decoded['kp'] * (decoded_joint_q - current_q)
            + decoded['kd'] * (decoded_joint_v - current_v)
        )
        if predicted_effort > settings['effort_limit_nm']:
            reason = (
                f'Predicted effort {predicted_effort:.3f} Nm exceeds ceiling '
                f'{settings["effort_limit_nm"]:.3f} Nm'
            )
            self._record_overload(f'{joint.spec.name}: {reason}', joint)
            raise RuntimeError(reason)
        return packet, decoded, next_v, ideal_q, decoded_joint_q

    def _check_motion_reply(self, joint, reply, decoded_joint_q):
        self._validate_feedback(reply, 1, joint=joint)
        speed_limit = _measured_speed_limit(joint.settings['max_rpm'])
        if abs(reply['rpm']) > speed_limit:
            raise RuntimeError(
                f'Measured speed {reply["rpm"]:.3f} rpm exceeds configured trip {speed_limit:.3f} rpm '
                f'({joint.settings["max_rpm"]:.3f} rpm command limit plus servo margin)'
            )
        direction = joint.settings['direction']
        measured_q = direction * (reply['position_rad'] - joint.q_zero)
        q_min = math.radians(joint.settings['lower_deg'])
        q_max = math.radians(joint.settings['upper_deg'])
        if measured_q < q_min or measured_q > q_max:
            raise RuntimeError(f'Measured position {measured_q:.4f} rad exceeds travel limits')
        if abs(decoded_joint_q - measured_q) > MAX_MEASURED_LEAD_RAD:
            raise RuntimeError(
                f'Measured position lead {abs(decoded_joint_q - measured_q):.4f} rad '
                f'exceeds {MAX_MEASURED_LEAD_RAD} rad'
            )
        if abs(reply['torque_nm']) > joint.settings['effort_limit_nm']:
            reason = (
                f'Measured torque {reply["torque_nm"]:.3f} Nm exceeds ceiling '
                f'{joint.settings["effort_limit_nm"]:.3f} Nm'
            )
            self._record_overload(f'{joint.spec.name}: {reason}', joint)
            raise RuntimeError(reason)

    def _trip_loss(self, reason):
        with self.lock:
            self._latch_stop_locked(reason)
        stop_ok = self._stop()
        with self.lock:
            if stop_ok:
                if not self.data['state'].startswith('Fault —'):
                    self.data['state'] = 'Stopped · re-check required'
                self.data['reason'] = self._stop_cause or str(reason)[:240]
            else:
                self.data['state'] = 'Fault — Stop unconfirmed'
                self.data['reason'] = self._stop_cause or 'No actual mode-0 response confirmed Stop after input loss.'
        self.stop_work.clear()

    def _trip_fault(self, error):
        message = str(error) or type(error).__name__
        with self.lock:
            if not self.data['state'].startswith('Fault —'):
                self._stop_cause = message[:240]
            self._latch_stop_locked(message)
            if self.data['state'] != 'Fault — serial close unconfirmed':
                self.data['state'] = 'Fault — disarmed'
                self.data['reason'] = self._stop_cause or message[:240]
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
                self.data['reason'] = self._stop_cause or message[:240]
            self.data['armed'] = False
            self.armed = False
        self.stop_work.clear()

    def _fresh_mode1_intent(self, now):
        with self.lock:
            intent = self.intent
            if not self.armed or not self.data['connected'] or self.bus is None:
                return None
            if not self._mode1_authority_matches_locked(intent):
                return None
            if self.intent_expires_at is None or now >= self.intent_expires_at:
                return None
            if self.stop_requested.is_set() or self.stop_latched or self.closing.is_set():
                return None
            for joint in self.configured_joints:
                if joint.received_at is None or joint.latest is None:
                    return None
                age = now - joint.received_at
                if age < 0 or age > FEEDBACK_MAX_AGE_S:
                    return None
            return intent

    def _poll_stop(self):
        joints = self.configured_joints
        if self.stop_requested.is_set() or self.stop_work.is_set():
            return
        startup_timeouts = []
        for joint in joints:
            if self.stop_requested.is_set() or self.stop_work.is_set():
                return
            reply = self._transact_raw(
                command(joint.spec.motor_id, 0, 0, 0, 0, True),
                motor_id=joint.spec.motor_id,
            )
            joint.last_poll_host_s = time.monotonic()
            if reply.get('mode') != 0:
                joint.stop_confirmed = False
                joint.stop_result = 'unconfirmed'
                joint.stop_error = 'Idle mode-0 poll did not confirm Stop'
                with self.lock:
                    self._record_stop_result(False)
                raise RuntimeError(f'{joint.spec.name} idle mode-0 poll did not confirm Stop')
            joint.stop_confirmed = True
            joint.stop_result = 'confirmed'
            joint.stop_error = None
            self._validate_feedback(reply, 0, allow_timeout=True, joint=joint)
            if reply['timeout']:
                startup_timeouts.append(joint.spec.name)

        self.last_poll_host_s = time.monotonic()
        with self.lock:
            self.stop_confirmed = bool(all(item.stop_confirmed for item in joints) and self.data['connected'])
            self.data['stop_confirmed'] = self.stop_confirmed
            if self.data['state'].startswith(('Fault', 'Timeout', 'Stopped')):
                return
            if startup_timeouts:
                if self.armed:
                    self._latch_stop_locked('Watchdog timeout while armed; acknowledge in Stop')
                self.data['state'] = 'Timeout — disarmed'
                self.data['reason'] = (
                    ', '.join(startup_timeouts)
                    + ' startup timeout; acknowledge only while each motor is supported.'
                )

    def _advance_virtual_joints(self, axes, moving, dt):
        if self.profile_id != 'arm_j1':
            return
        max_speed = self.settings['max_rpm'] * math.tau / 60
        v2 = axes[1] * max_speed if moving else 0.0
        v3 = axes[2] * max_speed if moving else 0.0
        self.q_j2 = max(-math.pi / 2, min(math.pi / 2, self.q_j2 + v2 * dt))
        self.q_j3 = max(-math.pi / 2, min(math.pi / 2, self.q_j3 + v3 * dt))
        self.v_j2, self.v_j3 = v2, v3

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
            cycle_gap = host_now - self.last_step_host_s
            self.last_step_host_s = host_now
            if cycle_gap > SCHEDULE_GAP_MAX_S:
                self._trip_fault(RuntimeError(
                    f'Schedule gap exceeded {SCHEDULE_GAP_MAX_S * 1000:.0f}ms'
                ))
                return
            cycle_dt = min(PER_STEP_DT_MAX_S, max(0.0, cycle_gap))
            with self.lock:
                now = self._clock_now_locked()
                intent = self._fresh_mode1_intent(now)
                axes = tuple(self.target_axes)
                deadman = self.deadman_active
            for joint in self.configured_joints:
                age = self._fresh_feedback_age(now, joint)
                if joint.latest is None or age is None or age > FEEDBACK_MAX_AGE_S:
                    self._trip_fault(RuntimeError(f'Feedback stale for {joint.spec.name}; Stop and re-check required'))
                    return
                if joint.latest.get('mode') not in (0, 1):
                    self._trip_fault(RuntimeError(f'{joint.spec.name} reported unsupported feedback mode'))
                    return
                if joint.foc_entered and joint.latest['mode'] != 1:
                    self._trip_fault(RuntimeError(f'{joint.spec.name} left FOC; Stop and re-check required'))
                    return
            if self.intent is not None and self.intent_expires_at is not None and now >= self.intent_expires_at:
                self._trip_loss('Input ticket lease expired; Stop and re-check required')
                return

            modes = [joint.latest['mode'] for joint in self.configured_joints]
            if any(mode == 0 for mode in modes):
                if any(mode == 1 for mode in modes) and not deadman:
                    self._trip_loss('FOC entry was interrupted before every selected joint entered mode 1')
                    return
                physical_moving = deadman and any(abs(axes[index]) > 1e-4
                                                  for index in range(len(self.configured_joints)))
                if physical_moving and intent is not None:
                    for joint in self.configured_joints:
                        if joint.latest['mode'] == 0:
                            self._enter_foc(joint, intent)
                            joint.last_step_host_s = time.monotonic()
                            if self.stop_requested.is_set():
                                return
                    return
                if intent is not None:
                    self._advance_virtual_joints(axes, deadman, cycle_dt)
                if all(mode == 0 for mode in modes) and host_now - self.last_poll_host_s >= 0.050:
                    self._poll_stop()
                return

            if intent is None:
                self._trip_loss('Valid live input intent unavailable; Stop and re-check required')
                return
            for index, joint in enumerate(self.configured_joints):
                last_step = joint.last_step_host_s if joint.last_step_host_s is not None else host_now
                gap = host_now - last_step
                if gap > SCHEDULE_GAP_MAX_S:
                    self._trip_fault(RuntimeError(
                        f'{joint.spec.name} schedule gap exceeded {SCHEDULE_GAP_MAX_S * 1000:.0f}ms'
                    ))
                    return
                dt = min(PER_STEP_DT_MAX_S, max(0.0, gap))
                if dt <= 0:
                    continue
                packet, decoded, next_v, ideal_q, joint_q = self._build_motion_packet(
                    joint, axes[index], dt,
                )
                reply = self._send_mode1(
                    joint, packet, decoded, intent, 1, 'jog', expected_axis=axes[index]
                )
                self._check_motion_reply(joint, reply, joint_q)
                with self.lock:
                    joint.q_ref = joint_q
                    joint.q_ideal = ideal_q
                    joint.v_ref = next_v
                    joint.last_reference_raw = decoded['raw_position']
                    joint.last_step_host_s = time.monotonic()
                    if self.stop_requested.is_set():
                        return
            self._advance_virtual_joints(axes, deadman, cycle_dt)
            with self.lock:
                moving = any(abs(axis) > 1e-4 for axis in self.target_axes)
                braking = any(abs(joint.v_ref) > 1e-5 for joint in self.configured_joints)
                self.data['state'] = 'Jogging' if moving else 'Armed · braking' if braking else 'Armed · holding'
                self.data['reason'] = ('Manual jogging active' if moving else
                                       'Motion input released; braking to zero' if braking else
                                       'Motion input released; holding position')
            return

        poll_interval = 0.050
        if host_now - self.last_poll_host_s >= poll_interval:
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
                    self.data['state'] = (
                        'Fault — disarmed' if self.data['state'].startswith('Fault —')
                        else 'Stopped · re-check required'
                    )
                    if self.data['reason'] == 'Select an adapter and connect.':
                        self.data['reason'] = 'Click Check connection before re-arming.'
                else:
                    self.data['state'] = 'Fault — Stop unconfirmed'
                    if not self.data['reason']:
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
                if action == 'select_profile' and self.owner not in (None, session):
                    self._record_receipt(session, op_id, 'rejected', 'Another session owns the bus')
                    return
                if action not in ('connect', 'select_profile') and session != self.owner:
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
