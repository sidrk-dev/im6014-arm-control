from dataclasses import dataclass
import errno
import fcntl
import os
from pathlib import Path
import re
import stat

from im6014_transport import PosixDeadlineSerial, UnsupportedSerialDeadlineAdapter


SUPPORTED_BAUDS = (4_000_000, 6_000_000)
BENCH_SERIAL = 'FTC2RQ7H'
BENCH_CHANNEL = 3

_LOCK_DIRECTORY = f'/tmp/im6014-port-owner-{os.getuid()}'

_BY_ID_FTDI = re.compile(r'_(?P<serial>[^_]+)-if(?P<channel>\d{2})-port\d+$')
_LOCATION_CHANNEL = re.compile(r':\d+\.(\d+)$')
_INTERFACE_CHANNEL = re.compile(r'(?:if|interface\s*)(\d+)$', re.IGNORECASE)


@dataclass(frozen=True)
class PortConfig:
    """One explicit port route; ``None`` identity fields mean an alternate route."""

    path: str
    baud: int
    expected_serial: str | None = BENCH_SERIAL
    expected_channel: int | None = BENCH_CHANNEL

    def __post_init__(self):
        if not isinstance(self.path, str) or not self.path.strip():
            raise ValueError('Port path must be a non-empty string')
        if type(self.baud) is not int or self.baud not in SUPPORTED_BAUDS:
            raise ValueError('Baud must be the integer 4000000 or 6000000')
        if self.expected_serial is not None and (
                not isinstance(self.expected_serial, str) or not self.expected_serial.strip()):
            raise ValueError('Expected adapter serial must be a non-empty string or None')
        if self.expected_channel is not None and (
                type(self.expected_channel) is not int or self.expected_channel < 0):
            raise ValueError('Expected adapter channel must be a nonnegative integer or None')
        if (self.expected_serial is None) != (self.expected_channel is None):
            raise ValueError('Expected serial and channel must both be set or both be None')


class PortBusyError(RuntimeError):
    """Another cooperating workbench client owns this character device."""


class PortOwnershipError(RuntimeError):
    """The device path or the private lock namespace is unsafe or changed."""


class PortCloseError(RuntimeError):
    """Close could not be confirmed; managed_port retains the retryable lease."""

    def __init__(self, managed_port):
        super().__init__('Serial close was not confirmed; the bus lease remains held')
        self.managed_port = managed_port


class ManagedPort:
    """One serial handle and the device lease held until close is confirmed.

    The advisory lease coordinates cooperating workbench clients. A raw client
    can ignore it. ``bus`` is the serial object consumed by the existing packet
    code; ``verified`` becomes true only after the opened fd matches the device
    inspected before construction.
    """

    def __init__(self, bus, lock_fd):
        self.bus = bus
        self._lock_fd = lock_fd
        self.verified = False
        self.close_confirmed = False
        self.close_attempted = False

    @property
    def close_unconfirmed(self):
        return self.close_attempted and not self.close_confirmed

    def close(self):
        """Close the bus and release the lease only after confirmed closure.

        A failed close deliberately keeps the lock fd alive. Calling ``close``
        again is safe and can release the lease after the bus really closes.
        """
        if self.close_confirmed:
            return True
        self.close_attempted = True
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                return False
            try:
                if getattr(self.bus, 'is_open', None) is not False:
                    return False
            except Exception:
                return False
        try:
            os.close(self._lock_fd)
        except OSError:
            return False
        self._lock_fd = None
        self.close_confirmed = True
        return True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        if self.close():
            return False
        raise PortCloseError(self) from exc


def _validate_private_directory(path, uid=None):
    uid = os.getuid() if uid is None else uid
    try:
        before = os.lstat(path)
    except OSError as error:
        raise PortOwnershipError(f'Cannot inspect bus lock directory: {error.strerror}') from error
    if not stat.S_ISDIR(before.st_mode) or stat.S_ISLNK(before.st_mode):
        raise PortOwnershipError('Bus lock namespace must be a real directory')
    if before.st_uid != uid or stat.S_IMODE(before.st_mode) != 0o700:
        raise PortOwnershipError('Bus lock namespace must be owned by this user with mode 0700')
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | getattr(os, 'O_CLOEXEC', 0)
    try:
        directory_fd = os.open(path, flags)
    except OSError as error:
        raise PortOwnershipError(f'Cannot open bus lock directory: {error.strerror}') from error
    opened = os.fstat(directory_fd)
    try:
        current = os.lstat(path)
    except OSError:
        os.close(directory_fd)
        raise PortOwnershipError('Bus lock namespace changed while opening')
    if (not stat.S_ISDIR(opened.st_mode) or opened.st_uid != uid or
            stat.S_IMODE(opened.st_mode) != 0o700 or
            (opened.st_dev, opened.st_ino) != (current.st_dev, current.st_ino)):
        os.close(directory_fd)
        raise PortOwnershipError('Bus lock namespace changed while opening')
    return directory_fd


def _validate_tmp_root():
    path = '/tmp'
    try:
        before = os.lstat(path)
    except OSError as error:
        raise PortOwnershipError(f'Cannot inspect trusted /tmp: {error.strerror}') from error
    if (not stat.S_ISDIR(before.st_mode) or stat.S_ISLNK(before.st_mode) or
            before.st_uid != 0 or stat.S_IMODE(before.st_mode) != 0o1777):
        raise PortOwnershipError('Bus ownership requires root-owned sticky /tmp with mode 1777')
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | getattr(os, 'O_CLOEXEC', 0)
    try:
        directory_fd = os.open(path, flags)
    except OSError as error:
        raise PortOwnershipError(f'Cannot open trusted /tmp: {error.strerror}') from error
    try:
        opened = os.fstat(directory_fd)
        current = os.lstat(path)
        if ((opened.st_dev, opened.st_ino) != (current.st_dev, current.st_ino) or
                not stat.S_ISDIR(opened.st_mode) or opened.st_uid != 0 or
                stat.S_IMODE(opened.st_mode) != 0o1777):
            raise PortOwnershipError('Trusted /tmp changed while opening')
    except BaseException:
        os.close(directory_fd)
        raise
    os.close(directory_fd)


def _private_lock_fd(device_number):
    uid = os.getuid()
    _validate_tmp_root()
    try:
        os.mkdir(_LOCK_DIRECTORY, 0o700)
    except FileExistsError:
        pass
    except OSError as error:
        raise PortOwnershipError(f'Cannot create bus lock namespace: {error.strerror}') from error

    directory_fd = _validate_private_directory(_LOCK_DIRECTORY, uid)
    name = f'{os.major(device_number)}-{os.minor(device_number)}.lock'
    flags = os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | getattr(os, 'O_CLOEXEC', 0)
    lock_fd = None
    try:
        lock_fd = os.open(name, flags, 0o600, dir_fd=directory_fd)
        details = os.fstat(lock_fd)
        if (not stat.S_ISREG(details.st_mode) or details.st_uid != uid or
                stat.S_IMODE(details.st_mode) & 0o077):
            raise PortOwnershipError('Bus lock file must be a private regular file owned by this user')
        path_details = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        if ((path_details.st_dev, path_details.st_ino) != (details.st_dev, details.st_ino) or
                not stat.S_ISREG(path_details.st_mode)):
            raise PortOwnershipError('Bus lock file changed while opening')
    except OSError as error:
        if lock_fd is not None:
            os.close(lock_fd)
        raise PortOwnershipError(f'Cannot verify bus lock file: {error.strerror}') from error
    except BaseException:
        if lock_fd is not None:
            os.close(lock_fd)
        raise
    finally:
        os.close(directory_fd)
    # Never unlink lockfiles: all processes must keep locking the same inode.
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as error:
        os.close(lock_fd)
        if error.errno in (errno.EACCES, errno.EAGAIN):
            raise PortBusyError('This adapter is already owned by another workbench client') from error
        raise PortOwnershipError(f'Cannot acquire bus lock: {error.strerror}') from error
    return lock_fd


def _character_device(path):
    try:
        observed = os.stat(path)
    except OSError as error:
        raise PortOwnershipError(f'Cannot inspect character device {path}: {error.strerror}') from error
    if not stat.S_ISCHR(observed.st_mode):
        raise PortOwnershipError(f'Configured port is not a character device: {path}')
    return observed


def _same_device(path, device_number):
    observed = _character_device(path)
    if observed.st_rdev != device_number:
        raise PortOwnershipError(f'Character device changed while opening: {path}')
    return observed


def open_managed_port(config: PortConfig, serial_factory=None):
    """Open a serial device while holding a cross-process per-device lease.

    The lease is acquired before calling the injectable serial factory. The
    fixed per-user namespace ignores TMPDIR/XDG settings and has no caller
    override. This protects cooperating workbench entry points only.
    """
    if not isinstance(config, PortConfig):
        raise TypeError('open_managed_port requires a PortConfig')
    if serial_factory is None:
        import serial
        serial_factory = serial.Serial

    inspect_port(config)
    device = _character_device(config.path)
    lock_fd = _private_lock_fd(device.st_rdev)
    managed = ManagedPort(None, lock_fd)
    try:
        _same_device(config.path, device.st_rdev)
        bus = serial_factory(port=None, baudrate=config.baud, timeout=0.005,
                             write_timeout=0.05, exclusive=True)
        managed.bus = bus
        bus.port = config.path
        bus.dtr = False
        bus.rts = False
        _same_device(config.path, device.st_rdev)
        bus.open()

        path_after_open = _same_device(config.path, device.st_rdev)
        try:
            opened = os.fstat(bus.fileno())
        except (AttributeError, OSError, TypeError, ValueError) as error:
            raise PortOwnershipError('Cannot verify opened serial device identity') from error
        if not stat.S_ISCHR(opened.st_mode) or opened.st_rdev != device.st_rdev:
            raise PortOwnershipError('Serial file descriptor does not match the configured character device')
        if path_after_open.st_rdev != opened.st_rdev:
            raise PortOwnershipError('Configured path changed during serial open')
        try:
            from serial.serialutil import SerialBase
            if isinstance(bus, SerialBase):
                try:
                    managed.bus = PosixDeadlineSerial(bus)
                except UnsupportedSerialDeadlineAdapter:
                    pass
        except ImportError:
            pass
        managed.verified = True
        return managed
    except BaseException as error:
        if not managed.close():
            raise PortCloseError(managed) from error
        raise


def _value(item, name, default=None):
    if isinstance(item, dict):
        return item.get(name, default)
    return getattr(item, name, default)


def _channel(value):
    if type(value) is int and value >= 0:
        return value
    if isinstance(value, str):
        if re.fullmatch(r'[0-9]+', value.strip()):
            return int(value.strip())
        match = _INTERFACE_CHANNEL.search(value.strip())
        if match:
            return int(match.group(1))
    return None


def _metadata(item, source):
    if item is None:
        return None
    serial = _value(item, 'serial_number') or _value(item, 'serial')
    interface = _channel(_value(item, 'channel'))
    if interface is None:
        interface = _channel(_value(item, 'interface'))
    location = _value(item, 'location')
    if interface is None and isinstance(location, str):
        match = _LOCATION_CHANNEL.search(location)
        if match:
            interface = int(match.group(1))
    device = _value(item, 'device') or _value(item, 'path')
    if interface is None and isinstance(device, str):
        match = _BY_ID_FTDI.search(Path(device).name)
        if match:
            interface = int(match.group('channel'))
            serial = serial or match.group('serial')

    manufacturer = _value(item, 'manufacturer')
    product = _value(item, 'product')
    vid = _value(item, 'vid')
    is_ftdi = (isinstance(manufacturer, str) and 'ftdi' in manufacturer.lower())
    is_ftdi = is_ftdi or (isinstance(product, str) and 'ftdi' in product.lower())
    is_ftdi = is_ftdi or vid in (0x0403, '0403', '0x0403')
    if vid is not None and vid not in (0x0403, '0403', '0x0403'):
        adapter = f'non-FTDI USB VID {vid}'
    else:
        adapter = 'FTDI' if is_ftdi else manufacturer or product
    if serial is None and interface is None and adapter is None:
        return None
    return serial, interface, str(adapter) if adapter else None, source


def _by_id_metadata(path):
    name = Path(path).name
    if 'FTDI' not in name.upper():
        return None
    match = _BY_ID_FTDI.search(name)
    if not match:
        return None
    return match.group('serial'), int(match.group('channel')), 'FTDI', 'by-id basename'


def _merge_metadata(primary, hint):
    if primary is None:
        return hint
    if hint is None:
        return primary
    serial, channel, adapter, source = primary
    hint_serial, hint_channel, hint_adapter, hint_source = hint
    return (serial or hint_serial, channel if channel is not None else hint_channel,
            adapter or hint_adapter, f'{source} and {hint_source}')


def _same_path(left, right):
    return os.path.normcase(os.path.realpath(left)) == os.path.normcase(os.path.realpath(right))


def _enumerated_metadata(path, ports):
    for port in ports:
        device = _value(port, 'device')
        if device and _same_path(path, device):
            return _metadata(port, 'OS enumeration metadata')
    return None


def _system_ports(path):
    if not os.path.exists(path):
        return ()
    from serial.tools import list_ports
    return list_ports.comports()


def _observed_text(observed):
    if observed is None:
        return 'unknown'
    serial, channel, adapter, _ = observed
    parts = []
    if adapter:
        parts.append(adapter)
    if serial:
        parts.append(f'serial {serial}')
    if channel is not None:
        parts.append(f'interface {channel}')
    return ', '.join(parts) or 'unknown'


def inspect_port(config: PortConfig, *, metadata=None, ports=None):
    """Check baud and any observed identity without opening or talking to a port.

    Returns a short passive-inspection report. Unknown metadata is explicitly
    reported as unverified and remains usable for injected serial-constructor
    failures and explicitly configured alternate routes.
    """
    if not isinstance(config, PortConfig):
        raise TypeError('inspect_port requires a PortConfig')

    observed = _metadata(metadata, 'caller metadata') if metadata is not None else None
    if observed is None:
        candidates = _system_ports(config.path) if ports is None else ports
        observed = _enumerated_metadata(config.path, candidates)
    observed = _merge_metadata(observed, _by_id_metadata(config.path))

    if config.expected_serial is None:
        return (f'Alternate port route {config.path}; no FTDI serial/channel is asserted. '
                f'Observed metadata: {_observed_text(observed)}; identity is not verified.')

    serial, channel, adapter, source = observed or (None, None, None, 'none')
    expected = f'FTDI serial {config.expected_serial}, interface {config.expected_channel}'
    wrong_adapter = adapter is not None and adapter.upper() != 'FTDI'
    wrong_serial = serial is not None and serial != config.expected_serial
    wrong_channel = channel is not None and channel != config.expected_channel
    if wrong_adapter or wrong_serial or wrong_channel:
        observed_text = _observed_text(observed)
        raise ValueError(f'Port {config.path}: expected {expected}; observed {observed_text} '
                         f'from {source}. Select the intended FTDI interface.')

    if serial == config.expected_serial and channel == config.expected_channel and adapter in (None, 'FTDI'):
        return (f'Observed {expected} from {source}; passive metadata matches. '
                'Motor communication is not verified.')
    return (f'Port metadata unknown or incomplete for {config.path}; expected {expected}; '
            f'observed {_observed_text(observed)}. Adapter identity is not verified.')
