"""Phase-coded SDM prototype, second receiver.

Each JTAG write changes one element pair: slot 2c commits el0/el1 of chip c,
slot 2c+1 commits el2/el3. The preamble flips exactly one element of the pair
by 180 deg per write so every preamble step is 2*|H_k| and cannot cancel across
the RX array; that gives clean timing, the write period T and a full-rank H.
Data chips are pi/M-rotated M-PSK. The data tracker picks the boundary and the
new absolute pair state jointly from M^2 hypotheses against e^{j th} H x, with a
Gaussian prior around b0 + T so weak (cancelling) steps fall back to timing.
"""
import sys
import numpy as np

RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0
FRAME_CHIPS = 512
PRE_CHIPS = 24


def pre_flips(seed=11):
    rng = np.random.default_rng(seed)
    return rng.integers(0, 2, size=(PRE_CHIPS, 2))  # which element of pair 01 / 23 flips


def pre_states():
    """Element phase states (0 / 128) for preamble chips [PRE_CHIPS, 4]."""
    f = pre_flips()
    st = np.zeros(4, int)
    out = np.zeros((PRE_CHIPS, 4), int)
    for c in range(PRE_CHIPS):
        st[f[c, 0]] ^= 128
        st[2 + f[c, 1]] ^= 128
        out[c] = st
    return out


ROT = {2: 1, 4: 1, 8: 1, 16: 1}  # per-chip rotation in units of 128/M (pi/M)


def rot(c, mpsk):
    return ROT.get(mpsk, 0) * (128 // mpsk) * (c + 1)


def make_frame(data, mpsk):
    """data [4, nd] ints in 0..M-1 -> phase bytes [FRAME_CHIPS, 4]."""
    nd = FRAME_CHIPS - PRE_CHIPS - 1
    out = np.zeros((FRAME_CHIPS, 4), np.uint8)
    out[:PRE_CHIPS] = pre_states()
    c = np.arange(PRE_CHIPS, FRAME_CHIPS - 1)
    out[PRE_CHIPS:-1] = ((256 // mpsk) * data[:, :nd] + rot(c, mpsk)).T % 256
    out[-1] = 0
    return out


def data_of(ph, mpsk):
    c = np.arange(PRE_CHIPS, FRAME_CHIPS - 1)
    return ((ph[PRE_CHIPS:-1].astype(int).T - rot(c, mpsk)) % 256) // (256 // mpsk)


def slot_states(ph):
    """[2*chips, 4] phase-byte state held during each slot (slot j ends at write j+1)."""
    n = ph.shape[0]
    s = np.zeros((2 * n, 4), int)
    prev = np.zeros(4, int)
    for c in range(n):
        s[2 * c, :2] = ph[c, :2]
        s[2 * c, 2:] = prev[2:]
        s[2 * c + 1] = ph[c]
        prev = ph[c].astype(int)
    return s


def ph2x(p):
    return np.exp(2j * np.pi * np.asarray(p) / 256.0)


# ------------------------------------------------------------------ receiver
def ddc(x4, f_if, D=16):
    N = x4.shape[1] // D * D
    n = np.arange(N)
    bb = x4[:, :N] * np.exp(-2j * np.pi * f_if * n / RX_FS)
    return bb.reshape(4, -1, D).mean(2), RX_FS / D


def step_energy(z, W=4):
    c = np.cumsum(np.concatenate([np.zeros((4, 1)), z], axis=1), axis=1)
    n = np.arange(W, z.shape[1] - W)
    d = (c[:, n + W] - c[:, n]) - (c[:, n] - c[:, n - W])
    e = np.zeros(z.shape[1])
    e[n] = (np.abs(d / W) ** 2).sum(0)
    return e


def noise_var(z):
    """Per-channel complex noise variance from first differences (robust to steps)."""
    d = np.diff(z, axis=1)
    return (np.median(np.abs(d), axis=1) / 0.8326) ** 2 / 2  # Rayleigh median -> sigma^2


def step_energy_rotinv(z, W=8):
    """min_phi |post - e^{j phi} pre|^2: ignores the common LO rotation (kHz residual CFO)."""
    c = np.cumsum(np.concatenate([np.zeros((4, 1)), z], axis=1), axis=1)
    n = np.arange(W, z.shape[1] - W)
    pre = (c[:, n] - c[:, n - W]) / W
    post = (c[:, n + W] - c[:, n]) / W
    e = np.zeros(z.shape[1])
    e[n] = (np.abs(pre) ** 2 + np.abs(post) ** 2).sum(0) - 2 * np.abs(np.sum(np.conj(pre) * post, 0))
    return e


def find_frames(z, fs, gap_us=200, k=8.0, W=8):
    e = step_energy_rotinv(z, W)
    nv = noise_var(z).sum()
    hit = np.where(e > k * nv * 2 / W)[0]
    q = int(gap_us * 1e-6 * fs)
    out = []
    for a, b in zip(hit[:-1], hit[1:]):
        if b - a > q:
            out.append((a + 4, b))
    return out, e


def cumsums(z):
    S1 = np.concatenate([np.zeros((4, 1)), np.cumsum(z, axis=1)], axis=1)
    S2 = np.concatenate([[0.0], np.cumsum((np.abs(z) ** 2).sum(0))])
    return S1, S2


def seg_cost(S1, S2, a, b, v):
    """sum_{n in [a,b)} |z[n] - v|^2 ; a,b arrays or scalars, v [4] or [4,K]."""
    s1 = S1[:, b] - S1[:, a]
    return (S2[b] - S2[a]) - 2 * np.real(np.sum(np.conj(v) * s1, axis=0)) + (b - a) * np.sum(np.abs(v) ** 2, axis=0)


def preamble_sync(z, S1, S2, b, fs, T0, nslots, nv, sig_us=1.5):
    """Unstructured change points with a timing prior; every preamble write is a single-element step."""
    T = T0
    bnd = [b]
    L = int(0.35 * T0)
    xi = 1.0 / (2 * nv.sum())
    sig = sig_us * 1e-6 * fs
    for j in range(nslots):
        b0 = bnd[-1]
        n = np.arange(int(b0 + 0.65 * T), int(b0 + 1.4 * T))
        pre = (S1[:, n] - S1[:, n - L]) / L
        post = (S1[:, n + L] - S1[:, n]) / L
        # log-likelihood gain of a step at n over no step, two-sided windows of L samples
        score = xi * L / 2 * np.sum(np.abs(post - pre) ** 2, axis=0) - ((n - b0 - T) / sig) ** 2 / 2
        bnd.append(int(n[np.argmax(score)]))
        if j >= 3:
            d = np.diff(bnd)
            T = np.median(d) if j < 8 else (bnd[-1] - bnd[0]) / (len(bnd) - 1)
    return np.array(bnd), T


def refine_known(S1, S2, bnd, H, X, T, fs, nv, sig_us=1.5):
    """Re-place boundaries with known states: v_j = H x_j before, H x_{j+1} after write j+1."""
    xi = 1.0 / (2 * nv.sum())
    sig = sig_us * 1e-6 * fs
    L = int(0.35 * T)
    V = H @ X  # [4, slots+1], column 0 = quiet state
    out = [bnd[0]]
    for j in range(1, len(bnd)):
        b0 = out[-1]
        e0 = b0 + T
        n = np.arange(int(b0 + 0.65 * T), int(b0 + 1.4 * T))
        cost = seg_cost(S1, S2, n - L, n, V[:, j][:, None]) + seg_cost(S1, S2, n, n + L, V[:, j + 1][:, None])
        out.append(int(n[np.argmin(xi * cost + ((n - e0) / sig) ** 2 / 2)]))
    return np.array(out)


class Tracker:
    def __init__(self, H, th, T, mpsk, sigma_us=1.5, fs=1.0, kp=0.2, ki=0.01, mu=0.02, lam=1.0):
        self.H, self.th, self.T, self.M = H.copy(), th, T, mpsk
        self.sig = sigma_us * 1e-6 * fs
        self.kp, self.ki, self.mu, self.lam = kp, ki, mu, lam
        self.w = 0.0
        q = np.arange(mpsk) * (256 // mpsk)
        self.pairs = np.array([(a, b) for a in q for b in q])  # [M^2, 2]

    def run(self, z, S1, S2, b0, state, c0, nv):
        """Track data slots starting at boundary b0 (end of preamble), element state [4] bytes."""
        M, T = self.M, self.T
        L = int(0.4 * T)
        N = FRAME_CHIPS - 1 - c0  # remaining data chips (tail written separately)
        decided = np.zeros((N, 4), int)
        bnd = [b0]
        st = state.copy()
        xi = 1.0 / (2 * nv.mean())
        info = []
        for c in range(c0, FRAME_CHIPS - 1):
            for half in (0, 1):
                bp = bnd[-1]
                e0 = bp + T
                # every candidate is scored over the same window [A, B): current state on
                # [A, n), hypothesis on [n, B). JTAG stalls only stretch a write (up to ~1.5 T),
                # they never shorten it much, so the search is skewed late.
                A = int(bp + 0.3 * T)
                B = min(int(bp + 1.75 * T), z.shape[1] - 1)
                n = np.arange(int(bp + 0.8 * T), min(int(bp + 1.6 * T), B - 1))
                rotb = rot(c, M)
                hyp = (self.pairs + rotb) % 256  # [K, 2]
                Xh = np.repeat(st[None, :], len(hyp), 0)
                idx = [0, 1] if half == 0 else [2, 3]
                Xh[:, idx] = hyp
                rot_ = np.exp(1j * self.th)
                v_pre = rot_ * (self.H @ ph2x(st))
                v_h = rot_ * (self.H @ ph2x(Xh).T)  # [4, K]
                cpre = seg_cost(S1, S2, np.full_like(n, A), n, v_pre[:, None])  # [Nn]
                s1 = S1[:, B][:, None] - S1[:, n]  # [4, Nn]
                cpost = ((S2[B] - S2[n])[:, None] - 2 * np.real(s1.T @ np.conj(v_h))
                         + (B - n)[:, None] * np.sum(np.abs(v_h) ** 2, 0)[None, :])
                dn = n - e0
                prior = self.lam * (dn / np.where(dn < 0, self.sig, 3 * self.sig)) ** 2 / 2
                tot = xi * (cpre[:, None] + cpost) + prior[:, None]
                i, h = np.unravel_index(np.argmin(tot), tot.shape)
                nb = int(n[i])
                st = Xh[h].copy()
                bnd.append(nb)
                # slot mean of the previous slot [bp, nb) for PLL + LMS on H
                m = (S1[:, nb] - S1[:, bp]) / max(nb - bp, 1)
                info.append((nb - bp))
                self._update(m, ph2x(self._prev))
                self._prev = st.copy()
            decided[c - c0] = st
        return decided, np.array(bnd), info

    def _update(self, m, x):
        pred = self.H @ x
        err = np.angle(np.vdot(pred, m * np.exp(-1j * self.th)))
        self.w += self.ki * err
        self.th += self.kp * err + self.w
        r = m * np.exp(-1j * self.th) - pred
        self.H += self.mu * np.outer(r, np.conj(x))


def decode_frame(z, fs, fr, mpsk, T0=19.2e-6, verbose=True, **kw):
    a, b = fr
    # quiet tone level gives H @ 1 and the residual CFO
    seg = z[:, a:b - 2]
    ph = np.unwrap(np.angle(seg.sum(0)))
    k = np.polyfit(np.arange(len(ph)) / fs, ph, 1)
    f_res = k[0] / (2 * np.pi)
    n = np.arange(z.shape[1]) - b
    zr = z * np.exp(-2j * np.pi * f_res * n / fs)
    S1, S2 = cumsums(zr)
    nv = noise_var(zr[:, a:b])
    npre = 2 * PRE_CHIPS
    # boundaries of writes 0..npre-1; the tracker then finds write npre (first data write)
    bnd, T = preamble_sync(zr, S1, S2, b, fs, T0 * fs, npre - 1, nv)
    # slot j in [bnd[j], bnd[j+1]) holds the state *after* write j; column 0 is the quiet (all-zero) state
    X = np.concatenate([np.ones((4, 1)), ph2x(slot_states(pre_states())[:npre]).T], 1)
    yq = zr[:, a:b - 2].mean(1)
    for it in range(3):
        Yp = np.array([(S1[:, q] - S1[:, p]) / (q - p) for p, q in zip(bnd[:-1], bnd[1:])]).T
        Y = np.concatenate([yq[:, None], Yp], 1)
        Xs = X[:, :Y.shape[1]]
        H = Y @ np.linalg.pinv(Xs)
        if it < 2:
            bnd = refine_known(S1, S2, bnd, H, X, T, fs, nv)
            T = (bnd[-1] - bnd[0]) / (len(bnd) - 1)
    res = Y - H @ Xs
    if verbose:
        print("  f_res %+7.1f Hz  T %.3f us  pre resid %.1f dB  cond(H) %.1f  |H| %s" % (
            f_res, T / fs * 1e6, 10 * np.log10(np.sum(np.abs(res) ** 2) / np.sum(np.abs(Y) ** 2)),
            np.linalg.cond(H), np.round(np.linalg.norm(H, axis=0), 1)))
    tr = Tracker(H, 0.0, T, mpsk, fs=fs, **kw)
    tr._prev = pre_states()[-1]
    st0 = pre_states()[-1].copy()
    decided, bd, info = tr.run(zr, S1, S2, bnd[-1], st0, PRE_CHIPS, nv)
    c = np.arange(PRE_CHIPS, FRAME_CHIPS - 1)
    sym = ((decided.T - rot(c, mpsk)) % 256) // (256 // mpsk)
    return dict(sym=sym, H=H, T=T, bnd=np.concatenate([bnd, bd[1:]]), f_res=f_res, tr=tr)


def gen_pattern(path, nframes, mpsk, seed=1):
    rng = np.random.default_rng(seed)
    nd = FRAME_CHIPS - PRE_CHIPS - 1
    pat = np.concatenate([make_frame(rng.integers(0, mpsk, size=(4, nd)), mpsk) for _ in range(nframes)])
    pat.tofile(path)
    np.save(path + ".npy", pat)


def load_cap(path, f_if):
    raw = np.fromfile(path, dtype=np.int8).astype(np.float32).reshape(-1, 4, 2)
    x = (raw[..., 0] + 1j * raw[..., 1]).T
    seg = x[:, : 1 << 21].sum(0)
    S = np.abs(np.fft.fft(seg * np.hanning(len(seg)))) ** 2
    fr = np.fft.fftfreq(len(seg), 1 / RX_FS)
    m = np.abs(fr - f_if) < 60e3
    f0 = np.sum(fr[m] * S[m]) / np.sum(S[m])
    z, fs = ddc(x, f0)
    return z, fs, f0, np.abs(raw).max(axis=(0, 2))


if __name__ == "__main__":
    if sys.argv[1] == "gen":
        gen_pattern(sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
    elif sys.argv[1] == "rx":
        cap, patf, f_if, M = sys.argv[2], sys.argv[3], float(sys.argv[4]), int(sys.argv[5])
        pat = np.load(patf)
        nf = pat.shape[0] // FRAME_CHIPS
        z, fs, f0, pk = load_cap(cap, f_if)
        frames, _ = find_frames(z, fs)
        print("cfo %.0f Hz  raw peak %s  frames %d" % (f0 - f_if, pk, len(frames)))
        for fr in frames[:6]:
            if fr[1] + int(1.05 * 2 * FRAME_CHIPS * 19.5e-6 * fs) > z.shape[1]:
                continue
            r = decode_frame(z, fs, fr, M)
            errs = [(r["sym"] != data_of(pat[k * FRAME_CHIPS:(k + 1) * FRAME_CHIPS], M)).sum(1) for k in range(nf)]
            k = int(np.argmin([e.sum() for e in errs]))
            print("   pattern %d errs %s / %d" % (k, errs[k], r["sym"].shape[1]))
