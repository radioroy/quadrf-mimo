import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0
D = 37
y = np.load("/tmp/e1_y.npy")  # [4, M] at D/RX_FS spacing (~0.99 us)
dt = D / RX_FS
M = y.shape[1]

# transition energy
d = np.abs(np.diff(y, axis=1)).sum(0)
thr = np.median(d) * 6
idx = np.where(d > thr)[0]
# merge adjacent hits
ev = []
for i in idx:
    if not ev or i - ev[-1][-1] > 2:
        ev.append([i])
    else:
        ev[-1].append(i)
tev = np.array([np.mean(e) for e in ev]) * dt * 1e6
gaps = np.diff(tev)
print("transitions", len(tev))
h, b = np.histogram(gaps, bins=np.arange(0, 200, 1))
for i in np.argsort(h)[::-1][:8]:
    print("  gap %3d-%3d us: %d" % (b[i], b[i + 1], h[i]))
big = gaps[gaps > 100]
print("long gaps (us):", np.round(big, 1))

# slot period from transitions within the first batch: fit t = t0 + k*T
g = gaps[(gaps > 10) & (gaps < 100)]
k = np.round(g / 19.17)
T = (g.sum() / k.sum())
print("slot period est %.3f us" % T)
