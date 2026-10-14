"""Phase-shifter code check from `quadrf-mimo txtoggle k nb mask base 0,64,0,128,0,192,64,192 32`:
one element cycles through codes, each held 32 writes. Dwell levels give |a(code)|; the step
magnitude at each change gives |a(new) - a(old)|, which with the ideal 90/180 deg spacing
predicts |0<->64| = |0<->192| = 0.707 |0<->128| and |64<->192| = |0<->128|.
an_codes2.py cap.cs8"""
import sys
import numpy as np
from capio import open_cap, ddc, RX_FS

CYC = [0, 64, 0, 128, 0, 192, 64, 192]
FS = RX_FS / 16
z = ddc(open_cap(sys.argv[1]), 500020.5).astype(complex)
p = np.convolve((np.abs(z) ** 2).sum(1), np.ones(2048) / 2048, "same")
lo, hi = np.percentile(p, 5), np.percentile(p, 99)
on = np.flatnonzero(p > 0.5 * (lo + hi))
z = z[on[0] + 100:on[-1] - 100]
W, G = 14, 4
cs = np.vstack([np.zeros((1, 4)), np.cumsum(z, 0)])
n = np.arange(W + G, len(z) - W - G)
aft = (cs[n + G + W] - cs[n + G]) / W
bef = (cs[n - G] - cs[n - G - W]) / W
D = np.linalg.norm(aft - bef, axis=1)
thr = 0.4 * np.percentile(D, 99.97)
pk = []
i = 0
while i < len(D):
    if D[i] > thr:
        j = i + int(np.argmax(D[i:i + int(20e-6 * FS)]))
        pk.append(j)
        i = j + int(300e-6 * FS)
    else:
        i += 1
pk = np.array(pk)
mag = D[pk]
seglen = np.diff(n[pk]) / FS * 1e6
print("changes %d  dwell median %.0f us" % (len(pk), np.median(seglen)))
# align the 8-step cycle: the largest-magnitude steps are 0<->128 and 64<->192 (2|h|)
types = [(CYC[i], CYC[(i + 1) % 8]) for i in range(8)]
ideal = np.array([abs(np.exp(1j * np.pi * b / 128) - np.exp(1j * np.pi * a / 128)) for a, b in types])
best = None
for o in range(8):
    t = (np.arange(len(mag)) + o) % 8
    pred = ideal[t]
    sc = np.sum(mag * pred) / np.sum(pred ** 2)
    err = np.sum((mag - sc * pred) ** 2)
    if best is None or err < best[0]:
        best = (err, o)
o = best[1]
t = (np.arange(len(mag)) + o) % 8
ref = np.median(mag[[i for i in range(len(mag)) if types[t[i]] in ((0, 128), (128, 0))]])
print("step magnitude relative to |0<->128|, measured vs ideal:")
for k in range(8):
    sel = mag[t == k] / ref
    if len(sel):
        print("  %3d -> %3d : %.3f +- %.3f   ideal %.3f   n %d" % (types[k][0], types[k][1], np.median(sel),
              np.std(sel), ideal[k] / 2, len(sel)))
# dwell levels per code
lev = {}
for i in range(len(pk) - 1):
    a, b = n[pk[i]] + 30, n[pk[i + 1]] - 30
    if b - a < 50:
        continue
    code = types[t[i]][1]
    lev.setdefault(code, []).append(np.linalg.norm(z[a:b].mean(0)) if False else np.sqrt((np.abs(z[a:b]) ** 2).mean(0).sum()))
l0 = np.median(lev.get(0, [1]))
print("dwell rms level relative to code 0:", " ".join("c%d %.3f" % (c, np.median(v) / l0) for c, v in sorted(lev.items())))
