import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import lfilter

RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0
f_if = float(sys.argv[1]) if len(sys.argv) > 1 else 5e5
raw = np.fromfile("/tmp/cap.cs8", dtype=np.int8).astype(np.float32).reshape(-1, 4, 2)
x = (raw[..., 0] + 1j * raw[..., 1]).T
N = x.shape[1]
seg = x[:, : 1 << 21].sum(0)
S = np.abs(np.fft.fft(seg * np.hanning(len(seg)))) ** 2
fr = np.fft.fftfreq(len(seg), 1 / RX_FS)
m = np.abs(fr - f_if) < 100e3
f0 = fr[m][np.argmax(S[m])]
print("tone at %.1f Hz, CFO %.1f Hz" % (f0, f0 - f_if))
D = 64
n = np.arange(N)
bb = x * np.exp(-2j * np.pi * f0 * n / RX_FS)
M = N // D
y = bb[:, : M * D].reshape(4, M, D).mean(2)
fs2 = RX_FS / D
t = np.arange(M) / fs2
ph = np.unwrap(np.angle(y), axis=1)
# instantaneous frequency smoothed over 100 us
k = int(100e-6 * fs2)
inst = np.diff(ph, axis=1) * fs2 / (2 * np.pi)
inst_s = lfilter(np.ones(k) / k, 1, inst, axis=1)
print("inst freq (100us avg) std per ch:", inst_s[:, k:].std(1).round(1), "Hz")
rel = np.unwrap(np.angle(y[1:] * np.conj(y[0])), axis=1)
print("relative phase rx1..3 vs rx0 std (rad):", rel.std(1).round(3))
mag = np.abs(y)
snr = 10 * np.log10(mag.mean(1) ** 2 / (mag.var(1) * 2))
print("approx SNR per %.2f us sample (dB):" % (1e6 / fs2), snr.round(1))
fig, ax = plt.subplots(3, 1, figsize=(14, 9))
for ch in range(4):
    ax[0].plot(t * 1e3, ph[ch] - ph[ch, 0], lw=0.6)
    ax[1].plot(t[1:] * 1e3, inst_s[ch], lw=0.6)
ax[0].set_ylabel("phase rad")
ax[1].set_ylabel("inst freq Hz")
ax[1].set_ylim(-5000, 5000)
for r in range(3):
    ax[2].plot(t * 1e3, rel[r], lw=0.6)
ax[2].set_ylabel("rel phase vs rx0")
ax[2].set_xlabel("ms")
plt.tight_layout()
plt.savefig("/tmp/cw.png", dpi=70)
