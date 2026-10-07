"""Passive configuration checks for the existing IM6014 serial entry points."""
from dataclasses import dataclass
import os
from pathlib import Path
import re


SUPPORTED_BAUDS = (4_000_000, 6_000_000)
BENCH_SERIAL = 'FTC2RQ7H'
BENCH_CHANNEL = 3

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


def _value(item, name, default=None):
    if isinstance(item, dict):
        return item.get(name, default)
    return getattr(item, name, default)


def _channel(value):
    if type(value) is int and value >= 0:
        return value
    if isinstance(value, str):
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
