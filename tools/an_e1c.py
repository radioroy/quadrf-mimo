import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0
D = 37
dt = D / RX_FS * 1e6  # us per decimated sample
y = np.load("/tmp/e1_y.npy")
M = y.shape[1]

e = (np.abs(y[:, 2:] - y[:, :-2]) ** 2).sum(0)
e = np.concatenate([[0], e, [0]])
t = np.arange(M) * dt

# coarse search of slot period by folding
best = None
for T in np.arange(19.00, 19.40, 0.005):
    ph = (t % T) / T
    hb = np.bincount((ph * 64).astype(int), weights=e, minlength=64)
    score = hb.max() / hb.mean()
    if best is None or score > best[0]:
        best = (score, T, np.argmax(hb) / 64 * T)
print("fold: score %.2f T %.3f us, transition phase %.2f us" % best)
score, T, tphase = best

# Fold over short windows to see whether timing drifts / jumps (batch gaps)
W = int(2000 / dt)
for w0 in range(0, M - W, W * 3):
    sl = slice(w0, w0 + W)
    ph = (t[sl] % T) / T
    hb = np.bincount((ph * 64).astype(int), weights=e[sl], minlength=64)
    print("  t=%6.0f us  phase %.2f us  peak/mean %.1f" % (t[w0], np.argmax(hb) / 64 * T, hb.max() / hb.mean()))

fig, ax = plt.subplots(2, 1, figsize=(14, 6))
ax[0].plot(t[:3000], e[:3000], lw=0.5)
ax[1].plot(t, e, lw=0.3)
plt.savefig("/tmp/e1c.png", dpi=70)
