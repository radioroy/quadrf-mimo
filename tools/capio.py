"""cs8 capture access for the analysis scripts. Nothing here
holds more than one FFT segment or decimation chunk of complex samples at a time."""
import numpy as np

RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0


def open_cap(path):
    return np.memmap(path, dtype=np.int8, mode="r")


def ms(t):
    return int(float(t) * 1e-3 * RX_FS)


def iq(raw, a, b):
    """Samples [a, b) as complex64 [n, 4]."""
    r = np.asarray(raw[8 * a:8 * b], np.float32).reshape(-1, 4, 2)
    return (r[..., 0] + 1j * r[..., 1]).astype(np.complex64)


def psd(raw, a, b, N=1 << 15):
    """Hann-windowed Welch PSD over [a, b), fftshifted, [N, 4]. Normalized so a full-scale
    (amplitude 127) complex tone reads 0 dBFS in its peak bin."""
    w = np.hanning(N).astype(np.float32)
    P = np.zeros((N, 4))
    ns = (b - a) // N
    for s in range(ns):
        x = iq(raw, a + s * N, a + (s + 1) * N)
        P += np.abs(np.fft.fft(x * w[:, None], axis=0)) ** 2
    f = np.fft.fftshift(np.fft.fftfreq(N, 1 / RX_FS))
    return f, np.fft.fftshift(P / max(ns, 1), axes=0) / (127 ** 2 * w.sum() ** 2), (w ** 2).sum() / w.sum() ** 2 * N


def ddc(raw, f_if, decim=16, a=0, b=None, chunk=1 << 20):
    """Mix by -f_if and boxcar-decimate, same as the receiver front end. [n/decim, 4]."""
    b = raw.size // 8 if b is None else b
    b = a + ((b - a) // decim) * decim
    z = np.empty(((b - a) // decim, 4), np.complex64)
    for s0 in range(a, b, chunk):
        s1 = min(b, s0 + chunk)
        lo = np.exp(-2j * np.pi * f_if * np.arange(s0, s1) / RX_FS).astype(np.complex64)
        z[(s0 - a) // decim:(s1 - a) // decim] = (iq(raw, s0, s1) * lo[:, None]).reshape(-1, decim, 4).mean(1)
    return z
