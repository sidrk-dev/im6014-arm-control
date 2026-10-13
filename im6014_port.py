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
DEFAULT_ADAPTER_PROFILE = 'u2d2'
MAX_PORT_RECORDS = 32

_LOCK_DIRECTORY = f'/tmp/im6014-port-owner-{os.getuid()}'

_BY_ID_FTDI = re.compile(r'_(?P<serial>[^_]+)-if(?P<channel>\d{2})-port\d+$')
_LOCATION_CHANNEL = re.compile(r':\d+\.(\d+)$')
_INTERFACE_CHANNEL = re.compile(r'(?:if|interface\s*)(\d+)$', re.IGNORECASE)
_BUILTIN_UART = re.compile(r'/dev/ttyS[0-9]+')
_USB_ID = re.compile(r'(?:0[xX])?([0-9a-fA-F]{1,4})\Z')


@dataclass(frozen=True)
class AdapterProfile:
    id: str
    label: str
    vid: int | None = None
    pid: int | None = None
    serial: str | None = None
    interface: int | None = None


ADAPTER_PROFILES = (
    AdapterProfile('u2d2', 'U2D2', 0x0403, 0x6014, None, 0),
    AdapterProfile('legacy-ft4232h', 'Legacy FT4232H', 0x0403, 0x6011, BENCH_SERIAL, BENCH_CHANNEL),
    AdapterProfile('custom', 'Custom (identity unverified)'),
)
ADAPTER_PROFILE_IDS = tuple(profile.id for profile in ADAPTER_PROFILES)
_PROFILES_BY_ID = {profile.id: profile for profile in ADAPTER_PROFILES}


@dataclass(frozen=True)
class PortConfig:
    """One explicit port route and its adapter identity policy."""

    path: str
    baud: int
    adapter_profile: str = DEFAULT_ADAPTER_PROFILE

    def __post_init__(self):
        if not isinstance(self.path, str) or not self.path.strip():
            raise ValueError('Port path must be a non-empty string')
        if type(self.baud) is not int or self.baud not in SUPPORTED_BAUDS:
            raise ValueError('Baud must be the integer 4000000 or 6000000')
        if type(self.adapter_profile) is not str or self.adapter_profile not in _PROFILES_BY_ID:
            raise ValueError('Adapter profile must be one of: ' + ', '.join(ADAPTER_PROFILE_IDS))


@dataclass(frozen=True)
class PortRecord:
    """A bounded, passive description of one enumerated serial device."""

    device: str
    path: str
    label: str
    description: str
    kind: str


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
    if type(value) is int and 0 <= value <= 255:
        return value
    if isinstance(value, str):
        text = value.strip()
        if len(text) > 16:
            return None
        if re.fullmatch(r'[0-9]+', text):
            parsed = int(text)
            return parsed if parsed <= 255 else None
        match = _INTERFACE_CHANNEL.search(text)
        if match:
            parsed = int(match.group(1))
            return parsed if parsed <= 255 else None
    return None


def _usb_id(value):
    if type(value) is int:
        return value if 0 <= value <= 0xFFFF else None
    if isinstance(value, str):
        match = _USB_ID.fullmatch(value.strip())
        if match:
            return int(match.group(1), 16)
    return None


@dataclass(frozen=True)
class _PortMetadata:
    serial: str | None
    interface: int | None
    vid: int | None
    pid: int | None
    source: str


def _short_text(value, limit=128):
    return value.strip()[:limit] if isinstance(value, str) and value.strip() else None


def _metadata(item, source):
    if item is None:
        return None
    serial = _short_text(_value(item, 'serial_number')) or _short_text(_value(item, 'serial'))
    interface = _channel(_value(item, 'channel'))
    if interface is None:
        interface = _channel(_value(item, 'interface'))
    location = _value(item, 'location')
    if interface is None and isinstance(location, str):
        match = _LOCATION_CHANNEL.search(location[:128])
        if match:
            interface = _channel(match.group(1))
    device = _value(item, 'device') or _value(item, 'path')
    if interface is None and isinstance(device, str):
        match = _BY_ID_FTDI.search(Path(device).name)
        if match:
            interface = int(match.group('channel'))
            serial = serial or match.group('serial')
    vid = _usb_id(_value(item, 'vid'))
    pid = _usb_id(_value(item, 'pid'))
    if serial is None and interface is None and vid is None and pid is None:
        return None
    return _PortMetadata(serial, interface, vid, pid, source)


def _by_id_metadata(path):
    name = Path(path).name
    match = _BY_ID_FTDI.search(name)
    if not match:
        return None
    return _PortMetadata(match.group('serial'), int(match.group('channel')), None, None,
                         'by-id basename')


def _merge_metadata(primary, hint):
    if primary is None:
        return hint
    if hint is None:
        return primary
    return _PortMetadata(
        primary.serial or hint.serial,
        primary.interface if primary.interface is not None else hint.interface,
        primary.vid if primary.vid is not None else hint.vid,
        primary.pid if primary.pid is not None else hint.pid,
        f'{primary.source} and {hint.source}',
    )


def _same_path(left, right):
    return os.path.normcase(os.path.realpath(left)) == os.path.normcase(os.path.realpath(right))


def _enumerated_metadata(path, ports):
    for port in ports:
        device = _value(port, 'device')
        if device and _same_path(path, device):
            return _metadata(port, 'OS enumeration metadata')
    return None


def _system_ports():
    from serial.tools import list_ports
    return list_ports.comports()


def _observed_text(observed):
    if observed is None:
        return 'unknown'
    parts = []
    if observed.vid is not None:
        parts.append(f'USB VID {observed.vid:04X}')
    if observed.pid is not None:
        parts.append(f'USB PID {observed.pid:04X}')
    if observed.serial:
        parts.append(f'serial {observed.serial}')
    if observed.interface is not None:
        parts.append(f'interface {observed.interface}')
    return ', '.join(parts) or 'unknown'


def inspect_port(config: PortConfig, *, metadata=None, ports=None):
    """Check one adapter profile against available metadata without opening it."""
    if not isinstance(config, PortConfig):
        raise TypeError('inspect_port requires a PortConfig')

    observed = _metadata(metadata, 'caller metadata') if metadata is not None else None
    if observed is None:
        candidates = _system_ports() if ports is None else ports
        observed = _enumerated_metadata(config.path, candidates)
    observed = _merge_metadata(observed, _by_id_metadata(config.path))

    profile = _PROFILES_BY_ID[config.adapter_profile]
    if profile.id == 'custom':
        if observed and observed.serial == BENCH_SERIAL and observed.interface in (0, 1, 2):
            raise ValueError(
                f'Port {config.path}: legacy FT4232H interface {observed.interface} is not wired; '
                'select its interface 3 or verify another adapter path.')
        return (f'Custom port route {config.path}; no adapter identity is asserted. '
                f'Observed metadata: {_observed_text(observed)}; identity is not verified.')

    fields = (('VID', profile.vid, observed.vid if observed else None),
              ('PID', profile.pid, observed.pid if observed else None),
              ('interface', profile.interface, observed.interface if observed else None))
    if profile.id == 'legacy-ft4232h':
        fields += (('serial', profile.serial, observed.serial if observed else None),)
    conflicts = [f'{name} {actual:04X}' if name in ('VID', 'PID') else f'{name} {actual}'
                 for name, expected, actual in fields
                 if expected is not None and actual is not None and actual != expected]
    if conflicts:
        expected_text = ('U2D2 FT232H VID:PID 0403:6014, interface 0'
                         if profile.id == 'u2d2'
                         else 'legacy FT4232H serial FTC2RQ7H, VID:PID 0403:6011, interface 3')
        raise ValueError(f'Port {config.path}: expected {expected_text}; observed '
                         f'{_observed_text(observed)} from {observed.source}. Select the intended adapter profile and path.')

    if profile.id == 'u2d2':
        complete = observed is not None and all(value is not None for value in
                                                 (observed.vid, observed.pid, observed.interface))
        if complete:
            return (f'Observed an FT232H-class U2D2 candidate at {config.path} from {observed.source}. '
                    'USB metadata does not prove ROBOTIS provenance or motor communication.')
        return (f'U2D2 profile selected for {config.path}; metadata is incomplete '
                f'({_observed_text(observed)}). Adapter identity and motor communication are not verified.')

    complete = (observed is not None and observed.serial == profile.serial
                and observed.interface == profile.interface
                and observed.vid == profile.vid and observed.pid == profile.pid)
    if complete:
        return (f'Observed the legacy FT4232H route at {config.path} from {observed.source}; '
                'motor communication is not verified.')
    return (f'Legacy FT4232H profile selected for {config.path}; metadata is unknown or incomplete '
            f'({_observed_text(observed)}) from {observed.source if observed else "no metadata"}. '
            'Adapter identity is not fully verified.')


def _classify_port(metadata):
    if metadata is None:
        return 'unknown'
    if metadata.serial == BENCH_SERIAL:
        if metadata.interface in (0, 1, 2):
            return 'unsupported-channel'
        if (metadata.interface == BENCH_CHANNEL
                and metadata.vid in (None, 0x0403) and metadata.pid in (None, 0x6011)):
            return 'legacy-ft4232h'
    if metadata.vid == 0x0403 and metadata.pid == 0x6014:
        if metadata.interface == 0:
            return 'ft232h-candidate'
        if metadata.interface is not None:
            return 'unsupported-channel'
    return 'unknown'


def _by_id_alias(device, directory='/dev/serial/by-id'):
    try:
        with os.scandir(directory) as entries:
            aliases = sorted(entries, key=lambda entry: entry.name)
    except OSError:
        return None
    for entry in aliases:
        alias = entry.path
        try:
            if len(alias) <= 240 and entry.is_symlink() and os.path.exists(alias) and _same_path(alias, device):
                return alias
        except OSError:
            continue
    return None


def enumerate_ports(*, ports=None, by_id_directory='/dev/serial/by-id'):
    """Return bounded passive serial records with no opens, probes, or writes."""
    candidates = _system_ports() if ports is None else ports
    devices = []
    for port in candidates:
        device = _short_text(_value(port, 'device'), 256)
        if device and not _BUILTIN_UART.fullmatch(device):
            devices.append((device, port))
    records = []
    for device, port in sorted(devices, key=lambda entry: entry[0])[:MAX_PORT_RECORDS]:
        alias = _by_id_alias(device, by_id_directory)
        path = alias or device
        metadata = _merge_metadata(_metadata(port, 'OS enumeration metadata'),
                                   _by_id_metadata(path))
        description = _short_text(_value(port, 'description'), 160) or device
        label = f'{path} · {description}'
        records.append(PortRecord(device, path, label[:256], description,
                                  _classify_port(metadata)))
    return tuple(records)


def default_port(records, adapter_profile=DEFAULT_ADAPTER_PROFILE):
    """Choose a path only when exactly one record matches the selected profile."""
    if type(adapter_profile) is not str or adapter_profile not in _PROFILES_BY_ID:
        raise ValueError('Unknown adapter profile')
    kind = {'u2d2': 'ft232h-candidate',
            'legacy-ft4232h': 'legacy-ft4232h',
            'custom': None}[adapter_profile]
    matches = [_value(record, 'path') for record in records
               if kind is not None and _value(record, 'kind') == kind
               and isinstance(_value(record, 'path'), str) and _value(record, 'path')]
    return matches[0] if len(matches) == 1 else ''
