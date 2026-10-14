"""Per-element phase-shifter response from a `quadrf-mimo calib` capture.

usage: an_calib.py cap.cs8 [f_if]
Prints, for each element, the measured phase (deg) and relative amplitude (dB) of every
code c = 0, 16, ..., 240, and the error against the ideal 360*c/256.
"""
import sys
import numpy as np
from capio import open_cap, ddc

RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0
FS = RX_FS / 16
f_if = float(sys.argv[2]) if len(sys.argv) > 2 else 500020.5

z = ddc(open_cap(sys.argv[1]), f_if)  # [samples, 4ch] at FS

# carrier on: first sample well above the noise
p = np.abs(z).sum(1)
on = np.flatnonzero(p > 0.5 * np.median(p[p > np.percentile(p, 60)]))
z = z[on[0] + int(5e-3 * FS):on[-1] - 100]

# frequency track: lag-16 phase per 1 ms block (the signal is piecewise constant, so the
# lag product only sees the carrier), smoothed over 5 blocks, integrated and removed
D, B = 16, int(1e-3 * FS)
nb = len(z) // B
fr = np.zeros(nb)
for i in range(nb):
    s = z[i * B:(i + 1) * B]
    fr[i] = np.angle(np.sum(np.conj(s[:-D]) * s[D:])) * FS / (2 * np.pi * D)
frs = np.convolve(fr, np.ones(5) / 5, mode="same")
fi = np.interp(np.arange(len(z)), (np.arange(nb) + 0.5) * B, frs)
z = z * np.exp(-2j * np.pi * np.cumsum(fi) / FS)[:, None]
print("carrier %.0f Hz +- %.0f Hz" % (frs.mean(), frs.std()))

# step detector: two-sided 64-sample means
W = 64
c = np.cumsum(np.vstack([np.zeros((1, 4)), z]), 0)
m1 = (c[W:-W] - c[:-2 * W]) / W
m2 = (c[2 * W:] - c[W:-W]) / W
e = np.abs(m2 - m1).sum(1)
thr = 6 * np.median(e)
pk = []
i = 0
while i < len(e):
    if e[i] > thr:
        j = i + np.argmax(e[i:i + 2 * W])
        pk.append(j + W)
        i = j + int(150e-6 * FS)
    else:
        i += 1
pk = np.array(pk)

half = 16 * 19.3e-6 * FS  # samples per half segment
seq = [(k, cc, h) for k in range(4) for cc in range(0, 256, 16) for h in (0, 1)]
segs = []
for a, b in zip(pk[:-1], pk[1:]):
    nh = max(1, int(round((b - a) / half)))
    L = (b - a) / nh
    for q in range(nh):
        s0, s1 = int(a + q * L + 0.2 * L), int(a + (q + 1) * L - 0.15 * L)
        segs.append(z[s0:s1].mean(0))
segs = np.array(segs)
# the first visible step is k=0, c=16 test half (index 3 in seq); align on it
nrep = len(segs) // len(seq)
print("steps %d, segments %d, full reps %d" % (len(pk), len(segs), nrep))
start = 3 - 1  # segment between first and second peak is seq[3]; the one before it seq[2]
resp = np.zeros((4, 16), complex)
cnt = 0
for r in range(nrep):
    o = start + r * len(seq) - 0
    blk = segs[o:o + len(seq)] if o >= 0 else None
    if blk is None or len(blk) < len(seq):
        continue
    ref = blk[0::2]  # [64, 4ch]
    tst = blk[1::2]
    d = (tst - ref).reshape(4, 16, 4)  # [k, code, ch] = H_k (x_k(c) - 1)
    for k in range(4):
        Dk = d[k].T  # [ch, code]
        u, sv, vh = np.linalg.svd(Dk)
        dk = vh[0] * sv[0]  # Dk[:, c] = u0 s0 vh[0, c]: d_c up to a common complex factor
        if r == 0:
            # for g(c) = exp(j 2 pi c/256), |d_c| = 2 |H_k| sin(pi c/256): 0 at c = 0, peak at 128
            print("el%d |d| rel  :" % k, " ".join("%5.2f" % v for v in np.abs(dk) / np.abs(dk).max()),
                  " rank1 %.0f%%" % (100 * sv[0] ** 2 / (sv ** 2).sum()))
        # x_c = 1 + beta d_c lies on the unit circle and x_0 = 1, so the d_c lie on a circle
        # through the origin with centre c0: |d|^2 = 2 Re(conj(c0) d), linear in c0
        A = np.column_stack([2 * dk.real, 2 * dk.imag])
        sol = np.linalg.lstsq(A, np.abs(dk) ** 2, rcond=None)[0]
        c0 = sol[0] + 1j * sol[1]
        best = (0, 1 - dk / c0)
        resp[k] += best[1]
    cnt += 1
resp /= max(cnt, 1)
ideal = np.arange(0, 256, 16) * 360 / 256
for k in range(4):
    ph = np.degrees(np.unwrap(np.angle(resp[k])))
    err = (ph - ideal + 180) % 360 - 180
    amp = 20 * np.log10(np.abs(resp[k]) / np.abs(resp[k]).mean())
    print("el%d phase err deg:" % k, " ".join("%+5.1f" % v for v in err))
    print("     amp dB      :", " ".join("%+5.1f" % v for v in amp))
