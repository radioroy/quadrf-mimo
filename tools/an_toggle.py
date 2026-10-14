"""Write instants from a `quadrf-mimo txtoggle` capture: one element flipping 180 deg on
each of its writes. The shifter passes near zero amplitude mid-flip, so every flip shows as a
sharp dip of |z| summed over the lanes.
an_toggle.py cap.cs8"""
import sys
import numpy as np
from capio import open_cap, ddc, RX_FS

FS = RX_FS / 16
z = ddc(open_cap(sys.argv[1]), 500020.5)
p = np.sqrt((np.abs(z) ** 2).sum(1))
lvl = np.percentile(p, 95)
on = np.flatnonzero(p > 0.7 * lvl)
p = p[on[0] + 200:on[-1] - 200]
ref = np.convolve(p, np.ones(64) / 64, mode="same")
dip = p < 0.6 * ref
idx = np.flatnonzero(dip[1:] & ~dip[:-1]) + 1
keep = [idx[0]]
for i in idx[1:]:
    if i - keep[-1] > 6e-6 * FS:
        keep.append(i)
t = np.array(keep) / FS * 1e6
d = np.diff(t)
print("dips %d over %.1f ms" % (len(t), (t[-1] - t[0]) / 1e3))
h, e = np.histogram(d, bins=np.arange(0, 200, 4))
for a, n in zip(e[:-1], h):
    if n:
        print("  %3.0f-%3.0f us: %d" % (a, a + 4, n))
dd = d[(d > 28) & (d < 56)]
print("flip interval: mean %.2f us  std %.2f us  n %d  -> %.2f us/write"
      % (dd.mean(), dd.std(), len(dd), dd.mean() / 2))
print("first 80:", " ".join("%.0f" % v for v in d[:80]))
# flip step per lane: consecutive slot means differ by 2 h_k (other elements static)
zz = z[on[0] + 200:on[-1] - 200]
ki = np.array(keep)
gd = 4
mm = np.array([zz[a + gd:b - gd].mean(0) for a, b in zip(ki[:-1], ki[1:]) if b - a > 3 * gd])
st = np.median(np.abs(np.diff(mm, axis=0)), 0) / 2
print("element |h| per lane %s  norm %.1f  (static level %s)" % (np.round(st, 1), np.linalg.norm(st),
      np.round(np.sqrt((np.abs(zz) ** 2).mean(0)), 1)))
