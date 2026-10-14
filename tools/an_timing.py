"""Slot-duration statistics from strong transitions in a capture."""
import sys
import numpy as np
from scipy.signal import find_peaks
import proto

f0 = float(sys.argv[1])
raw = np.fromfile("/tmp/cap.cs8", dtype=np.int8).astype(np.float32).reshape(-1, 4, 2)
x = (raw[..., 0] + 1j * raw[..., 1]).T
z, fs = proto.ddc(x, f0, D=16)
e = proto.step_energy(z, W=6)
pk, _ = find_peaks(e, distance=int(9e-6 * fs))
h = e[pk]
ref = np.median(np.sort(h)[-len(h) // 2:])
strong = pk[h > 0.3 * ref]
d = np.diff(strong) / fs * 1e6
one = d[(d > 14) & (d < 30)]
print("single-slot intervals: n=%d mean %.2f std %.2f  p1 %.1f p50 %.1f p99 %.1f max %.1f us" % (
    len(one), one.mean(), one.std(), *np.percentile(one, [1, 50, 99]), one.max()))
print("fraction > 22 us: %.3f" % np.mean(one > 22))
