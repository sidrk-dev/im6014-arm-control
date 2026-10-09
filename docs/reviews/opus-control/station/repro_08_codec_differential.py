"""Differential check: im6014_codec.command/feedback at 9e03d918 vs 244811b.
Golden vectors from test_protocol.py, boundary values, and 200k random inputs.
Same bytes for every accepted input; same accept/reject decision for every input.
"""
import math
import random

import common  # noqa: F401
import im6014_codec as new
import old_codec_244811b as old

GOLDEN = [
    ({"motor_id": 0}, "feee80000000000000000000000000001d99fb6e"),
    ({"motor_id": 14}, "feee8e000000000000000000000000007da3a05b"),
    ({"motor_id": 0, "watchdog": False}, "feee000000000000000000000000000080462079"),
    ({"motor_id": 0, "mode": 1, "rpm": 5, "kd": 6}, "feee9000000044000000000000007b071f113ea7"),
    ({"motor_id": 3, "mode": 1, "rpm": -5, "kd": 1, "torque": -.2,
      "position": -math.pi / 2, "kp": 3}, "feee9300d8ffbcffab6afeffef003f013c754064"),
    ({"motor_id": 1, "mode": 1, "rpm": 60}, "feee910000002b0300000000000000006bad05c6"),
    ({"motor_id": 1, "mode": 1, "rpm": -60}, "feee91000000d5fc0000000000000000b16eeecc"),
]
for kw, hexs in GOLDEN:
    assert new.command(**kw).hex() == hexs == old.command(**kw).hex(), kw
print(f"golden command vectors: {len(GOLDEN)}/{len(GOLDEN)} identical (old == new == expected)")


def outcome(fn, kw):
    try:
        return ("ok", fn(**kw))
    except ValueError as e:
        return ("ValueError", str(e))
    except Exception as e:  # noqa: BLE001
        return (type(e).__name__, str(e))


rng = random.Random(15)
edge_pos = [0.0, -0.0, 1e-12, -1e-12, new.POSITION_MIN, new.POSITION_MAX, new.POSITION_MAX * 1.0000001,
            math.inf, math.nan, 2 ** 31, True]
edge_rpm = [0, new.RPM_MIN, new.RPM_MAX, new.RPM_MAX + 1, 1e-9, math.nan]
mismatch_bytes = mismatch_kind = msg_only = accepted = rejected = 0
examples = []
N = 200_000
for i in range(N):
    kw = dict(
        motor_id=rng.choice([0, 1, 7, 14, 15, -1, True]),
        mode=rng.choice([0, 1, 1, 1, 2, False]),
        rpm=rng.choice([0, rng.uniform(-6000, 6000), rng.choice(edge_rpm)]),
        kd=rng.choice([0, rng.uniform(-1, 7)]),
        torque=rng.choice([0, rng.uniform(-4, 4)]),
        watchdog=rng.choice([True, True, False]),
        position=rng.choice([0, rng.uniform(-2e5, 2e5), rng.uniform(-1e3, 1e3), rng.choice(edge_pos)]),
        kp=rng.choice([0, rng.uniform(-1, 31)]),
    )
    a, b = outcome(old.command, kw), outcome(new.command, kw)
    if a[0] != b[0]:
        mismatch_kind += 1
        if len(examples) < 5:
            examples.append((kw, a, b))
    elif a[0] == "ok":
        accepted += 1
        if a[1] != b[1]:
            mismatch_bytes += 1
            if len(examples) < 5:
                examples.append((kw, a, b))
    else:
        rejected += 1
        msg_only += a[1] != b[1]
print(f"random command inputs: {N}; accepted by both: {accepted}; rejected by both: {rejected}; "
      f"byte mismatches: {mismatch_bytes}; accept/reject mismatches: {mismatch_kind}; "
      f"same rejection but different message: {msg_only}")
for ex in examples:
    print("   example:", ex)

# feedback(): unchanged? compare on random frames with valid CRC
fb_mismatch = 0
for i in range(50_000):
    body = bytes(rng.getrandbits(8) for _ in range(20))
    frame = b"\xfc\xee" + body + new.crc32(body).to_bytes(4, "little")
    mid = body[0] & 15 if rng.random() < 0.9 else rng.randrange(15)
    if mid > 14:
        continue
    fa = outcome(lambda raw, motor_id: old.feedback(raw, motor_id), {"raw": frame, "motor_id": mid})
    fb = outcome(lambda raw, motor_id: new.feedback(raw, motor_id), {"raw": frame, "motor_id": mid})
    fb_mismatch += fa != fb
print(f"feedback() on 50k random CRC-valid frames: mismatches={fb_mismatch}")
