"""Flip step of one element from a `quadrf-mimo txtoggle` capture, with other elements on.
Two-sided window means either side of each candidate instant; flips are peaks of their
difference (every 2 writes, ~38 us). Step vector = (after - before) / 2 = h_k, per lane.
an_step.py cap.cs8"""
import sys
import numpy as np
from capio import open_cap, ddc, RX_FS

FS = RX_FS / 16
z = ddc(open_cap(sys.argv[1]), 500020.5).astype(complex)
p = np.sqrt((np.abs(z) ** 2).sum(1))
on = np.flatnonzero(p > 0.6 * np.percentile(p, 95))
z = z[on[0] + 2000:on[-1] - 2000]
# CFO from lag products at 2 flip periods (the BPSK sign cancels in pairs)
Lf = int(round(2 * 18.96e-6 * FS))
fe = np.angle(np.sum(z[2 * Lf:] * z[:-2 * Lf].conj())) / (2 * np.pi * 2 * Lf) * FS
z = z * np.exp(-2j * np.pi * fe * np.arange(len(z)) / FS)[:, None]
W, G = 14, 4
cs = np.vstack([np.zeros((1, 4)), np.cumsum(z, 0)])
n = np.arange(W + G, len(z) - W - G)
aft = (cs[n + G + W] - cs[n + G]) / W
bef = (cs[n - G] - cs[n - G - W]) / W
D = np.linalg.norm(aft - bef, axis=1)
pk = []
i = 0
thr = 0.5 * np.percentile(D, 99)
while i < len(D):
    if D[i] > thr:
        j = i + int(np.argmax(D[i:i + int(20e-6 * FS)]))
        pk.append(j)
        i = j + int(25e-6 * FS)
    else:
        i += 1
pk = np.array(pk)
st = (aft[pk] - bef[pk]) / 2
# sign-align to the first step (flips alternate sign)
ref = st[0]
sg = np.sign(np.real(st @ ref.conj()))
h = np.mean(st * sg[:, None], 0)
dt = np.diff(pk) / FS * 1e6
print("steps %d  median spacing %.1f us  cfo %.0f Hz  |h| per lane %s  norm %.1f  ang %s  level %s" % (
    len(pk), np.median(dt), fe, np.round(np.abs(h), 1), np.linalg.norm(h),
    np.round(np.degrees(np.angle(h * np.exp(-1j * np.angle(h[0]))))),
    np.round(np.sqrt((np.abs(z) ** 2).mean(0)), 1)))
