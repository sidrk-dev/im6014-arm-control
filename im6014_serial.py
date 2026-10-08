from collections import deque
from dataclasses import dataclass
import math
import threading
import time

from im6014_codec import feedback
from joint_contract import WriteResult

_monotonic = time.monotonic

_REQUEST_BYTES = 20
_REPLY_BYTES = 26
_CAPTURE_CAPACITY = 256
_IDENTITY_CHARS = 96
_PORT_CHARS = 160
_TAIL_BYTES = 25
_OUTCOMES = frozenset(('OK', 'TIMEOUT', 'INVALID_FEEDBACK', 'DRIVER_FAILURE'))
_DECODED_INTEGER_LIMITS = {
    'id': (0, 14), 'mode': (0, 1), 'timeout': (0, 1),
    'driver_c': (-128, 127), 'winding_c': (0, 255),
    'errors': (0, 0xFFFFFFFF), 'warnings': (0, 7),
}
_DECODED_FLOAT_FIELDS = ('voltage', 'torque_nm', 'rpm', 'position_rad')


def _bounded_count(value):
    if type(value) is not int or value < 0:
        return 0
    return min(value, 0x7FFFFFFF)


def _bounded_tail(value):
    if isinstance(value, bytes):
        return value[-_TAIL_BYTES:]
    if isinstance(value, bytearray):
        return bytes(value[-_TAIL_BYTES:])
    if isinstance(value, memoryview):
        try:
            return value.cast('B')[-_TAIL_BYTES:].tobytes()
        except (TypeError, ValueError):
            return b''
    return b''


class TransactionTimeoutError(RuntimeError):
    """The bounded transaction expired; diagnostics contain no exception text."""

    def __init__(self, message, *, rejected=0, crc_rejected=0, tail=b''):
        super().__init__(message)
        self.rejected_frames = _bounded_count(rejected)
        self.crc_rejected_frames = min(_bounded_count(crc_rejected), self.rejected_frames)
        self.tail = _bounded_tail(tail)


class InvalidFeedbackError(TransactionTimeoutError):
    """The transaction expired after rejecting one or more candidate frames."""


class WriteCancelledError(RuntimeError):
    """The serial write was cancelled before transmission by the pre-write hook."""


@dataclass(frozen=True)
class _PacketRecord:
    source: str
    identity: str
    connection: int
    port: str
    baud: int
    requested_motor_id: int
    started_s: float
    ended_s: float | None
    request_hex: str
    reply_hex: str
    decoded_fields: tuple
    outcome: str
    rejected_frames: int
    crc_rejected_frames: int
    tail_hex: str


class PacketCapture:
    """A copy-out, fixed-capacity ring of allowlisted packet diagnostics."""

    def __init__(self, capacity=_CAPTURE_CAPACITY, *, clock=None):
        if type(capacity) is not int or not 1 <= capacity <= _CAPTURE_CAPACITY:
            raise ValueError('Capture capacity must be an integer from 1 to 256')
        if clock is not None and not callable(clock):
            raise ValueError('Capture clock must be callable')
        self.capacity = capacity
        self._clock = time.monotonic if clock is None else clock
        self._records = deque(maxlen=capacity)
        self._dropped = 0
        self._lock = threading.Lock()

    def _now(self):
        value = self._clock()
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('Capture clock must return a finite monotonic number')
        try:
            value = float(value)
        except (OverflowError, TypeError, ValueError) as error:
            raise ValueError('Capture clock must return a finite monotonic number') from error
        if not math.isfinite(value) or not 0 <= value <= 1e12:
            raise ValueError('Capture clock must return a finite monotonic number')
        return value

    def _append(self, record):
        with self._lock:
            if len(self._records) == self.capacity:
                self._dropped = min(self._dropped + 1, 0x7FFFFFFFFFFFFFFF)
            self._records.append(record)

    def snapshot(self):
        with self._lock:
            records = self._records.copy()
            dropped = self._dropped
        return {
            'capacity': self.capacity,
            'dropped': dropped,
            'records': [
                {
                    'source': item.source,
                    'identity': item.identity,
                    'connection': item.connection,
                    'port': item.port,
                    'baud': item.baud,
                    'requested_motor_id': item.requested_motor_id,
                    'started_s': item.started_s,
                    'ended_s': item.ended_s,
                    'request_hex': item.request_hex,
                    'reply_hex': item.reply_hex,
                    'decoded_fields': dict(item.decoded_fields),
                    'outcome': item.outcome,
                    'rejected_frames': item.rejected_frames,
                    'crc_rejected_frames': item.crc_rejected_frames,
                    'tail_hex': item.tail_hex,
                }
                for item in records
            ],
        }


def _capture_metadata(source, identity, connection, port, baud, motor_id):
    if type(source) is not str or source not in ('physical', 'demo', 'fixture'):
        raise ValueError('Capture source must be physical, demo, or fixture')
    if type(identity) is not str or not identity:
        raise ValueError('Capture identity must be a nonempty string')
    if type(port) is not str:
        raise ValueError('Capture port must be a string')
    if source == 'physical' and not port:
        raise ValueError('Physical capture requires an explicit port')
    if type(connection) is not int or not 0 <= connection <= 0x7FFFFFFF:
        raise ValueError('Capture connection must be a nonnegative integer')
    if type(baud) is not int or baud not in (4000000, 6000000):
        raise ValueError('Capture baud must be 4000000 or 6000000')
    if type(motor_id) is not int or not 0 <= motor_id <= 14:
        raise ValueError('Capture motor ID must be an integer from 0 to 14')
    return source, identity[:_IDENTITY_CHARS], connection, port[:_PORT_CHARS], baud, motor_id


def _packet_prefix(value, limit):
    if isinstance(value, bytes):
        return value[:limit]
    if isinstance(value, bytearray):
        return bytes(value[:limit])
    if isinstance(value, memoryview):
        try:
            return value.cast('B')[:limit].tobytes()
        except (TypeError, ValueError):
            return b''
    return b''


def _reply_prefix(result):
    if type(result) is not dict:
        return b''
    raw = result.get('raw')
    if isinstance(raw, bytes):
        return raw[:_REPLY_BYTES]
    if isinstance(raw, bytearray):
        return bytes(raw[:_REPLY_BYTES])
    if isinstance(raw, memoryview):
        try:
            return raw.cast('B')[:_REPLY_BYTES].tobytes()
        except (TypeError, ValueError):
            return b''
    if type(raw) is str:
        raw = raw[:_REPLY_BYTES * 2]
        raw = raw[:len(raw) - len(raw) % 2]
        try:
            return bytes.fromhex(raw)[:_REPLY_BYTES]
        except ValueError:
            return b''
    return b''


def _decoded_fields(result):
    if type(result) is not dict:
        return ()
    fields = []
    for name, (minimum, maximum) in _DECODED_INTEGER_LIMITS.items():
        value = result.get(name)
        if type(value) is int and minimum <= value <= maximum:
            fields.append((name, value))
    for name in _DECODED_FLOAT_FIELDS:
        value = result.get(name)
        if type(value) in (int, float):
            try:
                value = float(value)
            except (OverflowError, TypeError, ValueError):
                continue
            if math.isfinite(value) and abs(value) <= 1e6:
                fields.append((name, value))
    return tuple(fields)


def _outcome(error):
    if isinstance(error, InvalidFeedbackError):
        return 'INVALID_FEEDBACK'
    if isinstance(error, (TransactionTimeoutError, TimeoutError)):
        return 'TIMEOUT'
    return 'DRIVER_FAILURE'


def capture_exchange(capture, exchange_fn, bus, packet, motor_id, *,
                     source, identity, connection, port, baud,
                     before_write=None, on_write=None):
    """Call the current caller's three-argument exchange and retain one safe record."""
    if not isinstance(capture, PacketCapture):
        raise ValueError('Capture must be a PacketCapture')
    if not callable(exchange_fn):
        raise ValueError('Capture exchange function must be callable')
    context = _capture_metadata(source, identity, connection, port, baud, motor_id)
    source, identity, connection, port, baud, motor_id = context
    request = _packet_prefix(packet, _REQUEST_BYTES)
    started = capture._now()
    reply = b''
    decoded = ()
    outcome = 'DRIVER_FAILURE'
    rejected = 0
    crc_rejected = 0
    tail = b''
    kwargs = {}
    if before_write is not None:
        kwargs['before_write'] = before_write
    if on_write is not None:
        kwargs['on_write'] = on_write
    try:
        if kwargs:
            result = exchange_fn(bus, packet, motor_id, **kwargs)
        else:
            result = exchange_fn(bus, packet, motor_id)
        reply = _reply_prefix(result)
        decoded = _decoded_fields(result)
        outcome = 'OK'
        return result
    except Exception as error:
        outcome = _outcome(error)
        if isinstance(error, TransactionTimeoutError):
            rejected = _bounded_count(error.rejected_frames)
            crc_rejected = min(_bounded_count(error.crc_rejected_frames), rejected)
            tail = _bounded_tail(error.tail)
        raise
    finally:
        try:
            ended = capture._now()
        except Exception:
            ended = None
        if ended is not None and ended < started:
            ended = None
        if outcome not in _OUTCOMES:
            outcome = 'DRIVER_FAILURE'
        capture._append(_PacketRecord(
            source, identity, connection, port, baud, motor_id, started, ended,
            request.hex(), reply.hex(), decoded, outcome, rejected, crc_rejected, tail.hex(),
        ))


def _remaining(deadline):
    remaining = deadline - _monotonic()
    if remaining <= 0:
        raise TransactionTimeoutError('No validated motor feedback within deadline')
    return remaining


def _bounded_timeout(original, remaining, writing=False):
    if original is None or writing and original == 0:
        return remaining
    return min(original, remaining)


def _receive(bus, motor_id, deadline, original_timeout):
    data = bytearray()
    rejected = 0
    crc_rejected = 0
    last_error = ''
    try:
        while True:
            bus.timeout = _bounded_timeout(original_timeout, _remaining(deadline))
            _remaining(deadline)
            size = min(26, max(1, bus.in_waiting))
            _remaining(deadline)
            data.extend(bus.read(size))
            _remaining(deadline)
            while True:
                _remaining(deadline)
                start = data.find(b'\xfc\xee')
                if start < 0:
                    data[:] = b'\xfc' if data.endswith(b'\xfc') else b''
                    break
                del data[:start]
                if len(data) < 26:
                    break
                try:
                    result = feedback(bytes(data[:26]), motor_id)
                except RuntimeError as error:
                    rejected += 1
                    last_error = str(error)
                    crc_rejected += int("CRC" in last_error)
                    del data[:1]
                else:
                    _remaining(deadline)
                    return result
    except TransactionTimeoutError as error:
        crc_detail = f'CRC rejected {crc_rejected}; ' if crc_rejected else ''
        message = (f'No validated motor feedback within deadline; rejected {rejected}; '
                   f'{crc_detail}{last_error}; tail {data[-25:].hex()}')
        error_type = InvalidFeedbackError if rejected else TransactionTimeoutError
        raise error_type(message, rejected=rejected, crc_rejected=crc_rejected,
                         tail=data[-25:]) from error


def exchange(bus, packet, motor_id, deadline_s=0.05, *, before_write=None, on_write=None):
    """Reset once, write once, and return requested feedback before expiry.

    The bus has one owner. Its timeout setters, reset and I/O must obey their
    driver contract. Objects without write_timeout must write immediately.
    Synchronous driver calls and cleanup cannot be forcibly interrupted.
    Read/write settings are restored before any validated result is returned.
    If the final deadline check expires after before_write allows dispatch,
    on_write receives PREWRITE_DEADLINE even though bus.write was never called.
    """
    if type(motor_id) is not int or not 0 <= motor_id <= 14:
        raise ValueError('Expected motor ID must be an integer from 0 to 14')
    if isinstance(deadline_s, bool) or not isinstance(deadline_s, (int, float)):
        raise ValueError('Transaction deadline must be a finite positive int or float')
    try:
        deadline_s = float(deadline_s)
    except (OverflowError, TypeError, ValueError) as error:
        raise ValueError('Transaction deadline must be finite and positive') from error
    if not math.isfinite(deadline_s) or deadline_s <= 0:
        raise ValueError('Transaction deadline must be finite and positive')
    if before_write is not None and not callable(before_write):
        raise ValueError('before_write hook must be callable')
    if on_write is not None and not callable(on_write):
        raise ValueError('on_write hook must be callable')
    deadline = _monotonic() + deadline_s
    if not math.isfinite(deadline):
        raise ValueError('Transaction deadline is not representable')
    original_timeout = bus.timeout
    has_write_timeout = hasattr(bus, 'write_timeout')
    original_write_timeout = bus.write_timeout if has_write_timeout else None
    failure = None
    try:
        _remaining(deadline)
        bus.reset_input_buffer()
        _remaining(deadline)
        if has_write_timeout:
            bus.write_timeout = _bounded_timeout(original_write_timeout, _remaining(deadline), writing=True)
        _remaining(deadline)
        if before_write is not None:
            allowed = before_write()
            if not allowed:
                raise WriteCancelledError('Serial write cancelled by pre-write hook')
        try:
            _remaining(deadline)
        except TransactionTimeoutError:
            if on_write is not None:
                on_write(WriteResult.PREWRITE_DEADLINE)
            raise
        try:
            written = bus.write(packet)
        except Exception:
            if on_write is not None:
                try:
                    on_write(WriteResult.FAILED)
                except Exception:
                    pass
            raise
        if type(written) is int and written == len(packet):
            write_result = WriteResult.COMPLETE
        elif type(written) is int and 0 < written < len(packet):
            write_result = WriteResult.PARTIAL
        else:
            write_result = WriteResult.FAILED
        if on_write is not None:
            on_write(write_result)
        _remaining(deadline)
        if write_result is not WriteResult.COMPLETE:
            raise RuntimeError('Incomplete serial write')
        result = _receive(bus, motor_id, deadline, original_timeout)
    except ValueError as error:
        failure = error
        raise RuntimeError('Serial timeout configuration failed') from error
    except Exception as error:
        failure = error
        raise
    finally:
        restore_errors = []
        for name, value in [('timeout', original_timeout), *(
                [('write_timeout', original_write_timeout)] if has_write_timeout else [])]:
            try:
                setattr(bus, name, value)
            except Exception as error:
                restore_errors.append((name, error))
        if restore_errors:
            names = ', '.join(name for name, _ in restore_errors)
            detail = f'; transaction also failed: {failure}' if failure else ''
            raise RuntimeError(f'Cannot restore serial timeout settings: {names}{detail}') from restore_errors[0][1]
    _remaining(deadline)
    return result
