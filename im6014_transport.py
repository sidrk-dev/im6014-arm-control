from enum import Enum
import errno
import math
import os
import select
import time


class WriteResult(str, Enum):
    COMPLETE = "complete"
    PARTIAL = "partial"
    FAILED = "failed"
    PREWRITE_DEADLINE = "prewrite_deadline"


class UnsupportedSerialDeadlineAdapter(RuntimeError):
    """The serial backend cannot provide bounded POSIX descriptor I/O."""


class PartialWriteTimeout(TimeoutError):
    """A bounded serial write expired after sending only part of its payload."""

    def __init__(self, bytes_written):
        super().__init__('Serial write deadline expired after a partial write')
        self.bytes_written = bytes_written


class WriteDeadlineExpired(TimeoutError):
    """The transaction deadline expired before any request byte was written."""


class PosixDeadlineSerial:
    """Keep pySerial port settings stable and bound transactions with select."""

    def __init__(self, serial_bus):
        if os.name != 'posix' or not hasattr(os, 'get_blocking'):
            raise UnsupportedSerialDeadlineAdapter('POSIX nonblocking descriptors are unavailable')
        try:
            fd = serial_bus.fileno()
            if not serial_bus.is_open or os.get_blocking(fd):
                raise UnsupportedSerialDeadlineAdapter('Serial descriptor must be open and nonblocking')
        except (AttributeError, OSError, ValueError) as error:
            raise UnsupportedSerialDeadlineAdapter('Cannot inspect serial descriptor') from error
        timeout = serial_bus.timeout
        write_timeout = serial_bus.write_timeout
        self._coerce_timeout(timeout)
        self._coerce_timeout(write_timeout)
        object.__setattr__(self, '_serial', serial_bus)
        object.__setattr__(self, '_timeout', timeout)
        object.__setattr__(self, '_write_timeout', write_timeout)

    def __getattr__(self, name):
        return getattr(self._serial, name)

    def __setattr__(self, name, value):
        if name == 'timeout':
            self._timeout = self._coerce_timeout(value)
        elif name == 'write_timeout':
            self._write_timeout = self._coerce_timeout(value)
        elif name in ('_serial', '_timeout', '_write_timeout'):
            object.__setattr__(self, name, value)
        else:
            setattr(self._serial, name, value)

    @staticmethod
    def _coerce_timeout(value):
        if value is None:
            return None
        try:
            value = float(value)
        except (OverflowError, TypeError, ValueError) as error:
            raise ValueError('Timeout must be a finite nonnegative number or None') from error
        if not math.isfinite(value) or value < 0:
            raise ValueError('Timeout must be a finite nonnegative number or None')
        return value

    @property
    def timeout(self):
        return self._timeout

    @property
    def write_timeout(self):
        return self._write_timeout

    def _cutoff(self, deadline, clock, timeout_cap):
        if timeout_cap is None or timeout_cap <= 0:
            return deadline
        return min(deadline, clock() + timeout_cap)

    def read_with_deadline(self, size, deadline, clock, timeout_cap=None, *, poll_once=False):
        """Read up to ``size`` bytes, returning partial data at either deadline."""
        if type(size) is not int or size < 0:
            raise ValueError('Read size must be a nonnegative integer')
        if size == 0:
            return b''
        fd = self._serial.fileno()
        if poll_once:
            try:
                ready, _, _ = select.select([fd], [], [], 0)
            except InterruptedError:
                return b''
            if not ready:
                return b''
            try:
                return os.read(fd, size)
            except OSError as error:
                if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                    return b''
                raise
        cutoff = self._cutoff(deadline, clock, timeout_cap)
        data = bytearray()
        while len(data) < size:
            remaining = cutoff - clock()
            if remaining <= 0:
                break
            try:
                ready, _, _ = select.select([fd], [], [], None if math.isinf(remaining) else remaining)
            except InterruptedError:
                continue
            if not ready:
                break
            if clock() >= cutoff:
                break
            try:
                chunk = os.read(fd, size - len(data))
            except OSError as error:
                if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                    continue
                raise
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data)

    @staticmethod
    def _payload_view(payload):
        try:
            return memoryview(payload).cast('B')
        except (TypeError, ValueError) as error:
            raise TypeError('Serial payload must be a contiguous bytes-like object') from error

    def write_with_deadline(self, payload, deadline, clock, timeout_cap=None, *, poll_once=False):
        """Write the whole payload before its absolute deadline or report failure."""
        view = self._payload_view(payload)
        if not view:
            return 0
        fd = self._serial.fileno()
        if poll_once:
            try:
                _, ready, _ = select.select([], [fd], [], 0)
            except InterruptedError:
                return 0
            if not ready:
                return 0
            try:
                return os.write(fd, view)
            except OSError as error:
                if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                    return 0
                raise
        cutoff = self._cutoff(deadline, clock, timeout_cap)
        offset = 0
        while offset < len(view):
            remaining = cutoff - clock()
            if remaining <= 0:
                if offset:
                    raise PartialWriteTimeout(offset)
                if clock() >= deadline:
                    raise WriteDeadlineExpired('Transaction deadline expired before serial write')
                raise TimeoutError('Serial write deadline expired')
            try:
                _, ready, _ = select.select([], [fd], [], None if math.isinf(remaining) else remaining)
            except InterruptedError:
                continue
            if not ready:
                if offset:
                    raise PartialWriteTimeout(offset)
                if clock() >= deadline:
                    raise WriteDeadlineExpired('Transaction deadline expired before serial write')
                raise TimeoutError('Serial write deadline expired')
            if clock() >= cutoff:
                if offset:
                    raise PartialWriteTimeout(offset)
                if clock() >= deadline:
                    raise WriteDeadlineExpired('Transaction deadline expired before serial write')
                raise TimeoutError('Serial write deadline expired')
            try:
                count = os.write(fd, view[offset:])
            except OSError as error:
                if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR):
                    continue
                if offset:
                    raise PartialWriteTimeout(offset) from error
                raise
            if count == 0:
                continue
            offset += count
        return offset

    def read(self, size):
        timeout = self.timeout
        now = time.monotonic()
        deadline = (now if timeout == 0 else math.inf if timeout is None else now + timeout)
        return self.read_with_deadline(size, deadline, time.monotonic, timeout,
                                       poll_once=timeout == 0)

    def write(self, payload):
        timeout = self.write_timeout
        now = time.monotonic()
        deadline = (now if timeout == 0 else math.inf if timeout is None else now + timeout)
        return self.write_with_deadline(payload, deadline, time.monotonic, timeout,
                                        poll_once=timeout == 0)
