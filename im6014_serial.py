import math
import time

from im6014_codec import feedback

_monotonic = time.monotonic


def _remaining(deadline):
    remaining = deadline - _monotonic()
    if remaining <= 0:
        raise RuntimeError('No validated motor feedback within deadline')
    return remaining


def _bounded_timeout(original, remaining, writing=False):
    if original is None or writing and original == 0:
        return remaining
    return min(original, remaining)


def _receive(bus, motor_id, deadline, original_timeout):
    data = bytearray()
    rejected = 0
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
                    del data[:1]
                else:
                    _remaining(deadline)
                    return result
    except RuntimeError as error:
        if 'deadline' in str(error):
            raise RuntimeError(f'No validated motor feedback within deadline; rejected {rejected}; '
                               f'{last_error}; tail {data[-25:].hex()}') from error
        raise


def exchange(bus, packet, motor_id, deadline_s=0.05):
    """Reset once, write once, and return requested feedback before expiry.

    The bus has one owner. Its timeout setters, reset and I/O must obey their
    driver contract. Objects without write_timeout must write immediately.
    Synchronous driver calls and cleanup cannot be forcibly interrupted.
    Read/write settings are restored before any validated result is returned.
    """
    if not math.isfinite(deadline_s) or deadline_s <= 0:
        raise ValueError('Transaction deadline must be finite and positive')
    deadline = _monotonic() + deadline_s
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
        written = bus.write(packet)
        _remaining(deadline)
        if written != len(packet):
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
