"""One-owner, single-request IM6014 stream transaction."""
import time

from im6014_codec import feedback

_monotonic = time.monotonic


def exchange(bus, packet, motor_id, deadline_s=0.05):
    """Reset once, write once, and find requested feedback before expiry."""
    bus.reset_input_buffer()
    deadline = _monotonic() + deadline_s
    if bus.write(packet) != len(packet):
        raise RuntimeError('Incomplete serial write')
    data = bytearray()
    rejected = 0
    last_error = ''
    while _monotonic() < deadline:
        data.extend(bus.read(min(26, max(1, bus.in_waiting))))
        while _monotonic() < deadline:
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
                if _monotonic() < deadline:
                    return result
                break
    raise RuntimeError(f'No validated motor feedback within deadline; rejected {rejected}; '
                       f'{last_error}; tail {data[-25:].hex()}')
