"""Carrier stability of a CW capture: frequency per 0.1 ms block, and phase deviation from
a straight line over 0.2 / 1 / 4 ms windows (deg rms), MRC over the four lanes.
an_freq.py cap.cs8 [t0_ms t1_ms] [lp_khz]   lp_khz: brick-wall low-pass on the DDC output first"""
import sys
import numpy as np
from capio import open_cap, ddc, ms, RX_FS

FS = RX_FS / 16
raw = open_cap(sys.argv[1])
a = ms(sys.argv[2]) if len(sys.argv) > 3 else 0
b = ms(sys.argv[3]) if len(sys.argv) > 3 else raw.size // 8
z = ddc(raw, 500020.5, a=a, b=b)
if len(sys.argv) > 4:
    Z = np.fft.fft(z, axis=0)
    f = np.fft.fftfreq(len(z), 1 / FS)
    Z[np.abs(f) > float(sys.argv[4]) * 1e3] = 0
    z = np.fft.ifft(Z, axis=0).astype(np.complex64)
p = (np.abs(z) ** 2).sum(1)
on = np.flatnonzero(p > 0.25 * np.percentile(p, 95))
z = z[on[0] + int(2e-3 * FS):on[-1] - int(2e-3 * FS)]
w = z[:-1].conj() * z[1:]
B = int(1e-4 * FS)
fb = np.angle(w[:len(w) // B * B].reshape(-1, B, 4).sum((1, 2))) * FS / (2 * np.pi)
print("carrier %+.0f Hz, 0.1 ms blocks: std %.0f Hz  p1/p99 %+.0f/%+.0f Hz  max step %.0f Hz" % (
    fb.mean(), fb.std(), np.percentile(fb, 1), np.percentile(fb, 99), np.abs(np.diff(fb)).max()))
f5 = fb[:len(fb) // 50 * 50].reshape(-1, 50).mean(1)
print("5 ms means:", " ".join("%+.0f" % v for v in f5[:40]))
# lane-combined phase: rotate each lane onto lane 0 with the long-term mean cross term
ref = (z * np.conj(z[:, :1])).mean(0)
c = (z * np.conj(ref / np.abs(ref))).sum(1)
ph = np.unwrap(np.angle(c))
for W in (0.2e-3, 1e-3, 4e-3):
    n = int(W * FS)
    x = np.arange(n)
    dev = []
    for i in range(0, len(ph) - n, n):
        seg = ph[i:i + n]
        dev.append(np.std(seg - np.polyval(np.polyfit(x, seg, 1), x)))
    print("phase dev from line over %.1f ms: %.1f deg rms (median %.1f)" % (W * 1e3, np.degrees(np.sqrt(np.mean(np.square(dev)))), np.degrees(np.median(dev))))
snr = np.abs(c).mean() ** 2 / np.var(np.abs(c))
print("|c| mean %.1f  amplitude SNR %.1f dB" % (np.abs(c).mean(), 10 * np.log10(snr)))
