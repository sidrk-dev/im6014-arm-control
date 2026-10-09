"""Attribute step() wall time for a 3-joint cycle on the in-memory bus (cProfile)."""
import cProfile, pstats, io, time
import common
from common import ScriptedBus, armed, submit

bus = ScriptedBus()
core, owner, tracker, coord = armed(bus, time.monotonic, nominal=0.005)
seq = [0]
def cycle():
    seq[0] += 1
    submit(core, owner, seq[0])
    return coord.step()
# warm-up
for _ in range(3): cycle()
t = []
for _ in range(30):
    a = time.perf_counter(); s0 = time.perf_counter(); submit(core, owner, seq[0] + 1); seq[0] += 1; s1 = time.perf_counter(); coord.step(); b = time.perf_counter()
    t.append((s1 - s0, b - s1))
print("3-joint: mean submit_target %.3f ms, mean step %.3f ms" % (sum(x for x, _ in t) / len(t) * 1e3, sum(y for _, y in t) / len(t) * 1e3))
pr = cProfile.Profile(); pr.enable()
for _ in range(20): cycle()
pr.disable()
s = io.StringIO(); pstats.Stats(pr, stream=s).sort_stats("cumulative").print_stats(18); print(s.getvalue()[:4000])
