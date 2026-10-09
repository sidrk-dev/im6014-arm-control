"""Three-ID pyserial-shaped packet fixture and synthetic arm specification.

Simulates IDs 0, 1, 2 with canonical CRC feedback and recorded writes.
Deterministic time and packet behavior for unit and integration testing.
Never imports or opens physical serial devices.
"""

from collections import deque
from collections.abc import Callable
import json
import math
from pathlib import Path
import struct
import time

from im6014_codec import RATIO, crc32, decode_command
from joint_contract import JointSpec
from joint_mapping import JointMap
from joint_protection import ProtectionPolicy

__all__ = (
    "DEFAULT_SYNTHETIC_PROTECTION",
    "J1_MAX_RAD",
    "J1_MIN_RAD",
    "SYNTHETIC_A_MAX",
    "SYNTHETIC_KD",
    "SYNTHETIC_KP",
    "SYNTHETIC_MAX_EFFORT",
    "SYNTHETIC_MAX_LEAD",
    "SYNTHETIC_RATIO",
    "SYNTHETIC_J2_MAX_RAD",
    "SYNTHETIC_J2_MIN_RAD",
    "SYNTHETIC_J3_MAX_RAD",
    "SYNTHETIC_J3_MIN_RAD",
    "SYNTHETIC_V_MAX",
    "ArmPacketFixture",
    "create_synthetic_arm_specs",
    "make_feedback_packet",
)

SYNTHETIC_RATIO = RATIO


def _j1_model_bounds() -> tuple[float, float]:
    model = json.loads(
        Path(__file__).with_name("arm_assets").joinpath("model.json").read_text(
            encoding="utf-8"
        )
    )
    joint = next(
        item for item in model["joints"]
        if item.get("role") == "base" and item.get("motor_id") == 0
    )
    limits = joint["limits_deg"]
    if (
        type(limits) is not list
        or len(limits) != 2
        or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in limits)
        or not all(math.isfinite(float(value)) for value in limits)
        or limits[0] >= limits[1]
    ):
        raise ValueError("generated model must define finite ordered J1 limits")
    return math.radians(float(limits[0])), math.radians(float(limits[1]))


J1_MIN_RAD, J1_MAX_RAD = _j1_model_bounds()
SYNTHETIC_J2_MIN_RAD = -2.356194490192345
SYNTHETIC_J2_MAX_RAD = 2.356194490192345
SYNTHETIC_J3_MIN_RAD = -2.356194490192345
SYNTHETIC_J3_MAX_RAD = 2.356194490192345
SYNTHETIC_V_MAX = 2.0
SYNTHETIC_A_MAX = 5.0
SYNTHETIC_MAX_LEAD = 0.5
SYNTHETIC_MAX_EFFORT = 3.5
SYNTHETIC_KP = 3.0
SYNTHETIC_KD = 0.2

DEFAULT_SYNTHETIC_PROTECTION = ProtectionPolicy(
    driver_trip_c=85.0,
    driver_reset_c=70.0,
    winding_trip_c=105.0,
    winding_reset_c=90.0,
    continuous_effort_nm=1.0,
    continuous_dwell_s=1.0,
    recovery_dwell_s=0.5,
)


def create_synthetic_arm_specs() -> tuple[JointSpec, ...]:
    return (
        JointSpec(
            name="J1 base",
            motor_id=0,
            mapping=JointMap(reduction=1.0, direction=1, zero_output_rad=0.0),
            q_min_rad=J1_MIN_RAD,
            q_max_rad=J1_MAX_RAD,
            v_max_rad_s=SYNTHETIC_V_MAX,
            a_max_rad_s2=SYNTHETIC_A_MAX,
            max_lead_rad=SYNTHETIC_MAX_LEAD,
            max_effort_nm=SYNTHETIC_MAX_EFFORT,
            limits_provenance="synthetic fixture: J1 +/- pi/2 bound",
            protection=DEFAULT_SYNTHETIC_PROTECTION,
        ),
        JointSpec(
            name="J2 shoulder",
            motor_id=1,
            mapping=JointMap(reduction=1.0, direction=1, zero_output_rad=0.0),
            q_min_rad=SYNTHETIC_J2_MIN_RAD,
            q_max_rad=SYNTHETIC_J2_MAX_RAD,
            v_max_rad_s=SYNTHETIC_V_MAX,
            a_max_rad_s2=SYNTHETIC_A_MAX,
            max_lead_rad=SYNTHETIC_MAX_LEAD,
            max_effort_nm=SYNTHETIC_MAX_EFFORT,
            limits_provenance="synthetic fixture: shoulder finite bound",
            protection=DEFAULT_SYNTHETIC_PROTECTION,
        ),
        JointSpec(
            name="J3 elbow",
            motor_id=2,
            mapping=JointMap(reduction=1.0, direction=1, zero_output_rad=0.0),
            q_min_rad=SYNTHETIC_J3_MIN_RAD,
            q_max_rad=SYNTHETIC_J3_MAX_RAD,
            v_max_rad_s=SYNTHETIC_V_MAX,
            a_max_rad_s2=SYNTHETIC_A_MAX,
            max_lead_rad=SYNTHETIC_MAX_LEAD,
            max_effort_nm=SYNTHETIC_MAX_EFFORT,
            limits_provenance="synthetic fixture: elbow finite bound",
            protection=DEFAULT_SYNTHETIC_PROTECTION,
        ),
    )


def make_feedback_packet(
    motor_id: int,
    mode: int = 0,
    timeout: bool = False,
    driver_c: int = 25,
    winding_c: int = 30,
    voltage_v: float = 36.0,
    torque_nm: float = 0.0,
    rpm: float = 0.0,
    position_rad: float = 0.0,
    errors: int = 0,
    warnings: int = 0,
) -> bytes:
    """Generate a canonical 26-byte IM6014 feedback frame with valid CRC32."""
    ratio = RATIO
    m = (motor_id & 0x0F) | ((mode & 0x07) << 4) | ((1 if timeout else 0) << 7)
    td = max(-128, min(127, int(round(driver_c))))
    tw = max(0, min(255, int(round(winding_c))))
    v = max(0, min(255, int(round(voltage_v * 2))))
    raw_torque = max(-32768, min(32767, int(round(torque_nm * 2560.0 / ratio))))
    raw_rpm = max(-32768, min(32767, int(round(rpm * 64.0 * ratio / 60.0))))
    raw_pos = max(
        -2147483648,
        min(2147483647, int(round(position_rad * 32768.0 * ratio / math.tau))),
    )
    flags = (warnings & 0x07) << 13

    body = struct.pack(
        "<BbBBhhiIHBB",
        m,
        td,
        tw,
        v,
        raw_torque,
        raw_rpm,
        raw_pos,
        errors & 0xFFFFFFFF,
        flags,
        0,
        0,
    )
    crc = crc32(body)
    return b"\xfc\xee" + body + struct.pack("<I", crc)


class ArmPacketFixture:
    """Deterministic pyserial-shaped three-motor packet fixture.

    Never accesses physical hardware or serial ports. Decodes commands
    using canonical im6014_codec and generates canonical feedback frames.
    """

    def __init__(self, *, clock: Callable[[], float] | None = None) -> None:
        self.clock = time.monotonic if clock is None else clock
        self.is_open = True
        self.timeout = 0.05
        self.write_timeout = 0.05
        self._rx_buffer = bytearray()
        self._writes: deque[bytes] = deque(maxlen=256)
        self._states = {
            i: {
                "mode": 0,
                "position_rad": 0.0,
                "rpm": 0.0,
                "torque_nm": 0.0,
                "driver_c": 25,
                "winding_c": 30,
                "errors": 0,
                "warnings": 0,
                "timeout": False,
            }
            for i in (0, 1, 2)
        }
        self.fail_close: bool = False
        self.inject_partial_write: int | None = None
        self.inject_partial_write_ids: set[int] = set()
        self.inject_partial_write_bytes: int = 10
        self.inject_write_error: Exception | None = None
        self.inject_write_error_ids: set[int] = set()
        self.inject_timeout_ids: set[int] = set()
        self.inject_no_reply_ids: set[int] = set()
        self.inject_reply_modes: dict[int, int] = {}
        self.inject_corrupt_crc_ids: set[int] = set()
        self.inject_corrupt_feedback_ids: set[int] = set()
        self.inject_stop_failure_ids: set[int] = set()
        self.inject_errors: dict[int, int] = {}
        self.inject_warnings: dict[int, int] = {}
        self.inject_timeouts: dict[int, bool] = {}
        self.inject_temperatures: dict[int, tuple[int, int]] = {}

    @property
    def writes(self) -> tuple[bytes, ...]:
        return tuple(self._writes)

    @property
    def in_waiting(self) -> int:
        return len(self._rx_buffer)

    @property
    def motor_states(self) -> dict[int, dict]:
        return {i: dict(self._states[i]) for i in self._states}

    def reset_input_buffer(self) -> None:
        self._rx_buffer.clear()

    def write(self, data: bytes | bytearray | memoryview) -> int:
        if not self.is_open:
            raise RuntimeError("Fake serial port is closed")
        packet = bytes(data)
        decoded = decode_command(packet)
        motor_id = decoded["motor_id"]
        mode = decoded["mode"]

        self._writes.append(packet)
        if self.inject_write_error is not None:
            raise self.inject_write_error
        if motor_id in self.inject_write_error_ids:
            raise RuntimeError("Injected fixture write failure")
        if self.inject_partial_write is not None:
            return self.inject_partial_write
        if motor_id in self.inject_partial_write_ids:
            return self.inject_partial_write_bytes

        if motor_id not in self._states:
            return len(packet)

        state = self._states[motor_id]

        if mode == 0:
            if motor_id in self.inject_stop_failure_ids:
                state["mode"] = 1
            else:
                state["mode"] = 0
                state["rpm"] = 0.0
                state["torque_nm"] = 0.0
        elif mode == 1:
            state["mode"] = 1
            state["position_rad"] = decoded["position_rad"]
            state["rpm"] = decoded["rpm"]
            state["torque_nm"] = decoded["torque_nm"]

        if motor_id in self.inject_timeout_ids or motor_id in self.inject_no_reply_ids:
            return len(packet)

        temps = self.inject_temperatures.get(
            motor_id, (state["driver_c"], state["winding_c"])
        )
        err = self.inject_errors.get(motor_id, state["errors"])
        warn = self.inject_warnings.get(motor_id, state["warnings"])
        tout = self.inject_timeouts.get(motor_id, state["timeout"])

        frame = make_feedback_packet(
            motor_id=motor_id,
            mode=self.inject_reply_modes.get(motor_id, state["mode"]),
            timeout=tout,
            driver_c=temps[0],
            winding_c=temps[1],
            voltage_v=36.0,
            torque_nm=state["torque_nm"],
            rpm=state["rpm"],
            position_rad=state["position_rad"],
            errors=err,
            warnings=warn,
        )

        if motor_id in self.inject_corrupt_crc_ids:
            frame = frame[:-4] + b"\x00\x00\x00\x00"
        if motor_id in self.inject_corrupt_feedback_ids:
            frame = b"\x00\x00" + frame[2:]

        self._rx_buffer.extend(frame)
        return len(packet)

    def read(self, size: int = 1) -> bytes:
        if not self.is_open:
            raise RuntimeError("Fake serial port is closed")
        size = max(0, int(size))
        count = min(size, len(self._rx_buffer))
        chunk = bytes(self._rx_buffer[:count])
        del self._rx_buffer[:count]
        return chunk

    def close(self) -> None:
        if self.fail_close:
            return
        self.is_open = False
