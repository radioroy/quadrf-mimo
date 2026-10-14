import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0
path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cap.cs8"
f_if = float(sys.argv[2]) if len(sys.argv) > 2 else 2e6
out = sys.argv[3] if len(sys.argv) > 3 else "/tmp/e1.png"

raw = np.fromfile(path, dtype=np.int8).astype(np.float32).reshape(-1, 4, 2)
x = (raw[..., 0] + 1j * raw[..., 1]).T  # [4, N]
N = x.shape[1]
print("N", N, "ms", 1e3 * N / RX_FS)

nfft = 1 << 20
seg = x[:, :nfft]
win = np.hanning(seg.shape[1])
S = np.fft.fftshift(np.abs(np.fft.fft(seg * win, axis=1)) ** 2, axes=1)
fr = np.fft.fftshift(np.fft.fftfreq(seg.shape[1], 1 / RX_FS))
P = 10 * np.log10(S.sum(0) + 1e-9)
band = (fr > f_if - 300e3) & (fr < f_if + 300e3)
fpk = fr[band][np.argmax(P[band])]
print("peak near IF at %.1f Hz (CFO %.1f Hz)" % (fpk, fpk - f_if))
for lo, hi in [(-1e3, 1e3), (f_if - 1e6, f_if + 1e6)]:
    pass

# DDC at the measured peak, decimate by block mean (~1 us)
D = 37
n = np.arange(N)
bb = x * np.exp(-2j * np.pi * fpk * n / RX_FS)
M = N // D
y = bb[:, : M * D].reshape(4, M, D).mean(2)
t_us = np.arange(M) * D / RX_FS * 1e6

fig, ax = plt.subplots(4, 1, figsize=(14, 12))
ax[0].plot(fr / 1e6, P, lw=0.5)
ax[0].set_xlabel("MHz")
ax[0].set_ylabel("dB")
ax[0].set_title("RX spectrum (sum of 4 ch)")
ax[1].plot(fr[band] / 1e6, P[band], lw=0.6)
ax[1].set_xlabel("MHz")
sl = slice(0, int(2000 / (D / RX_FS * 1e6)))
for ch in range(4):
    ax[2].plot(t_us[sl], np.abs(y[ch, sl]), lw=0.7, label="rx%d" % ch)
    ax[3].plot(t_us[sl], np.unwrap(np.angle(y[ch, sl])), lw=0.7)
ax[2].legend()
ax[2].set_ylabel("|y|")
ax[3].set_ylabel("phase rad")
ax[3].set_xlabel("us")
plt.tight_layout()
plt.savefig(out, dpi=80)
np.save("/tmp/e1_y.npy", y)
print("saved", out)
