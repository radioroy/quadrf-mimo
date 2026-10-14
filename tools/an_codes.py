"""Code-to-phase map from a `quadrf-mimo txcodes` capture.
an_codes.py cap.cs8 [hold_ms]

The carrier wanders by kHz over tens of ms, so every code-c segment is referenced to the
mean of its two code-0 neighbours (2 ms either side), which cancels a linear phase ramp."""
import sys
import numpy as np
from capio import open_cap, ddc, RX_FS

FS = RX_FS / 16
hold = float(sys.argv[2]) * 1e-3 if len(sys.argv) > 2 else 2e-3
z = ddc(open_cap(sys.argv[1]), 500020.5)
p = (np.abs(z) ** 2).sum(1)
k = int(1e-4 * FS)
pb = p[:len(p) // k * k].reshape(-1, k).mean(1)
on = np.flatnonzero(pb > 0.25 * np.percentile(pb, 90))
t_on, t_off = on[0] * k, (on[-1] + 1) * k
# carrier from the 50 ms single-element marker (lag product over the middle 40 ms)
m = z[t_on + int(5e-3 * FS):t_on + int(45e-3 * FS)].sum(1)
fc = np.angle(np.sum(np.conj(m[:-8]) * m[8:])) * FS / (2 * np.pi * 8)
z = z * np.exp(-2j * np.pi * fc * np.arange(len(z)) / FS)[:, None]
t0 = t_on + int(50e-3 * FS)
seg = (t_off - t0) / 128
print("carrier %+.0f Hz  pattern %.1f ms  segment %.3f ms (nominal %.3f)" % (fc, (t_off - t0) / FS * 1e3, seg / FS * 1e3, hold * 1e3))
y = np.array([z[int(t0 + (i + 0.25) * seg):int(t0 + (i + 0.85) * seg)].mean(0) for i in range(128)])  # [128, 4ch]
for el in range(4):
    Y = y[32 * el:32 * (el + 1)]
    ref = Y[0::2]
    tst = Y[1::2]
    nxt = np.vstack([ref[1:], ref[-1:]])
    r = 0.5 * (ref + nxt)
    # MRC over the four RX lanes against the code-0 reference
    g = (np.conj(r) * tst).sum(1) / (np.abs(r) ** 2).sum(1)
    ph = np.degrees(np.angle(g))
    ideal = np.arange(16) * 22.5
    err = (ph - ideal + 180) % 360 - 180
    print("el%d phase deg:" % el, " ".join("%6.1f" % (v % 360) for v in ph))
    print("    err deg  :", " ".join("%+6.1f" % v for v in err))
    print("    amp dB   :", " ".join("%+6.1f" % (20 * np.log10(abs(v))) for v in g))
