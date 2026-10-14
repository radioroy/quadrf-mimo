"""Phase-coded MIMO waveform + receiver prototype (offline, numpy).

Physical model per JTAG slot j (one register write, ~19 us):
    y[j] = exp(1j*theta_j) * H @ x[j] + n       y: 4 RX, H: 4x4, x: 4 TX elements
Chip c writes 0x2C (el0, el1) then 0x2D (el2, el3), so slot 2c holds
el0/1 at chip c and el2/3 at chip c-1; slot 2c+1 holds all four at chip c.
"""
import sys
import numpy as np

RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0
FRAME_CHIPS = 512
PRE_CHIPS = 16
RNG = np.random.default_rng(12345)

HAD16 = np.array([[1]])
for _ in range(4):
    HAD16 = np.block([[HAD16, HAD16], [HAD16, -HAD16]])
PRE_ROWS = [1, 2, 4, 8]
PRE_SCR = np.where(np.random.default_rng(7).random(PRE_CHIPS) > 0.5, 1, -1)


def pre_symbols():
    return np.array([HAD16[r, :PRE_CHIPS] * PRE_SCR for r in PRE_ROWS])  # [4, P] +-1


def make_frame(data_bits_per_el, mpsk=2):
    """data_bits_per_el: int symbols [4, Nd] in 0..mpsk-1. Returns phase bytes [512,4]."""
    P = pre_symbols()
    nd = FRAME_CHIPS - PRE_CHIPS - 1
    sym = np.zeros((4, FRAME_CHIPS - 1), dtype=np.int64)
    sym[:, :PRE_CHIPS] = np.where(P > 0, 0, mpsk // 2)  # BPSK in the M-PSK alphabet
    sym[:, PRE_CHIPS:] = data_bits_per_el[:, :nd]
    c = np.arange(FRAME_CHIPS - 1)
    # pi/M rotation per chip guarantees every register write moves both of its elements
    ph = (256 * sym // mpsk + (128 // mpsk) * (c + 1)) % 256
    out = np.zeros((FRAME_CHIPS, 4), dtype=np.uint8)
    out[:-1] = ph.T
    out[-1] = 0  # tail chip: known state held during the inter-frame gap
    return out


def chip_x(phbytes):
    return np.exp(2j * np.pi * phbytes.astype(np.float64) / 256.0)  # [chips, 4]


def slot_x(phbytes, prev_tail=None):
    """Per-slot element state [2*chips, 4]."""
    x = chip_x(phbytes)
    n = x.shape[0]
    s = np.zeros((2 * n, 4), complex)
    prev = np.ones(4) if prev_tail is None else prev_tail
    for c in range(n):
        p23 = x[c - 1, 2:] if c > 0 else prev[2:]
        s[2 * c, :2] = x[c, :2]
        s[2 * c, 2:] = p23
        s[2 * c + 1] = x[c]
    return s


# ---------------------------------------------------------------- receiver
def ddc(x4, f_if, D=16):
    N = x4.shape[1] // D * D
    n = np.arange(N)
    bb = x4[:, :N] * np.exp(-2j * np.pi * f_if * n / RX_FS)
    return bb.reshape(4, -1, D).mean(2), RX_FS / D


def step_energy(z, W=4):
    """|mean(after) - mean(before)|^2 summed over RX: matched filter for a phase step."""
    c = np.cumsum(np.concatenate([np.zeros((4, 1)), z], axis=1), axis=1)
    n = np.arange(W, z.shape[1] - W)
    d = (c[:, n + W] - c[:, n]) - (c[:, n] - c[:, n - W])
    e = np.zeros(z.shape[1])
    e[n] = (np.abs(d / W) ** 2).sum(0)
    return e


def find_frames(z, fs, quiet_us=150):
    """Quiet = no transitions. Returns list of (quiet_start, quiet_end) in samples."""
    e = step_energy(z)
    lvl = np.median(e)
    hit = e > 12 * lvl
    q = int(quiet_us * 1e-6 * fs)
    frames = []
    last = -10**9
    idx = np.where(hit)[0]
    for a, b in zip(idx[:-1], idx[1:]):
        if b - a > q:
            frames.append((a + 3, b))
    return frames, e


def cfo_from_quiet(z, a, b, fs):
    seg = z[:, a:b - 2]
    ph = np.unwrap(np.angle(seg.sum(0)))
    t = np.arange(len(ph)) / fs
    k = np.polyfit(t, ph, 1)
    return k[0] / (2 * np.pi), ph[-1], k


def segment(e, start, nslots, fs, T0=19.1e-6, win=0.35):
    """Track transitions: boundaries[j] = sample index of transition j."""
    T = T0 * fs
    b = [start]
    for j in range(1, nslots + 1):
        exp = b[-1] + T
        lo, hi = int(exp - win * T), int(exp + win * T) + 1
        hi = min(hi, len(e) - 1)
        if lo >= hi:
            break
        k = lo + int(np.argmax(e[lo:hi]))
        b.append(k)
        T = 0.9 * T + 0.1 * (k - b[-2])
    return np.array(b)


def segment_cp(z, start, nslots, fs, T0=19.3e-6, lo=0.75, hi=1.6, G=2):
    """Sequential change-point boundary tracking (least squares, cumulative sums)."""
    S1 = np.concatenate([np.zeros((4, 1)), np.cumsum(z, axis=1)], axis=1)
    S2 = np.concatenate([[0.0], np.cumsum((np.abs(z) ** 2).sum(0))])
    T = T0 * fs
    b = [start]
    N = z.shape[1]
    for j in range(nslots):
        b0 = b[-1]
        nmin, nmax = int(b0 + lo * T), int(b0 + hi * T)
        if nmax >= N:
            break
        a = b0 + G
        ypre = (S1[:, nmin] - S1[:, a]) / (nmin - a)
        n = np.arange(nmin + 1, nmax - 2)
        # pre part [nmin, n) against the known level, post part [n, nmax) against its own mean
        s1p = S1[:, n] - S1[:, nmin][:, None]
        cpre = (S2[n] - S2[nmin]) - 2 * np.real(np.conj(ypre) @ s1p) + (n - nmin) * np.sum(np.abs(ypre) ** 2)
        s1q = S1[:, nmax][:, None] - S1[:, n]
        cpost = (S2[nmax] - S2[n]) - np.sum(np.abs(s1q) ** 2, axis=0) / (nmax - n)
        k = int(n[np.argmin(cpre + cpost)])
        b.append(k)
        T = 0.95 * T + 0.05 * (k - b0)
    return np.array(b)


def slot_means(z, bnd, guard=1):
    out = []
    for a, b in zip(bnd[:-1], bnd[1:]):
        out.append(z[:, a + guard:b - guard + 1].mean(1) if b - a > 2 * guard else z[:, a:b + 1].mean(1))
    return np.array(out).T  # [4, J]


def decode_frame(z, fs, fr, ref_ph, mpsk=2, verbose=True):
    a, b = fr
    f_res, ph_end, _ = cfo_from_quiet(z, a, b, fs)
    n0 = b  # first transition
    # remove the residual CFO measured in the quiet tone
    n = np.arange(z.shape[1]) - n0
    zr = z * np.exp(-2j * np.pi * f_res * n / fs)
    e = step_energy(zr)
    nslots = 2 * FRAME_CHIPS
    bnd = segment_cp(zr, n0, nslots - 1, fs)
    Y = slot_means(zr, bnd)  # [4, J]
    J = Y.shape[1]
    X = slot_x(ref_ph)[:J].T  # [4, J]
    # channel from preamble slots (LS == precomputed correlator bank)
    Jp = 2 * PRE_CHIPS
    Xp = X[:, :Jp]
    Wp = Xp.conj().T @ np.linalg.inv(Xp @ Xp.conj().T)
    H = Y[:, :Jp] @ Wp
    # data: per-slot ZF + decision-directed common-phase PLL
    Hinv = np.linalg.pinv(H)
    th, w = 0.0, 0.0
    Kp, Ki = 0.15, 0.004
    xh = np.zeros((4, J), complex)
    for j in range(J):
        v = Hinv @ Y[:, j] * np.exp(-1j * th)
        xh[:, j] = v
        # decision on the known-rotated alphabet
        c = j // 2 + 1
        rot = np.exp(1j * np.pi / mpsk * c)
        rot23 = np.exp(1j * np.pi / mpsk * (c - 1 if j % 2 == 0 else c))
        r = np.array([rot, rot, rot23, rot23])
        u = v / r
        dec = np.exp(1j * 2 * np.pi / mpsk * np.round(np.angle(u) / (2 * np.pi / mpsk)))
        err = np.angle(np.sum(u * np.conj(dec) * np.abs(u)))
        w += Ki * err
        th += Kp * err + w
    # chip decisions: el0/1 average slots 2c,2c+1; el2/3 average 2c+1,2c+2
    nc = J // 2 - 1
    est = np.zeros((4, nc), complex)
    for c in range(nc):
        est[:2, c] = 0.5 * (xh[:2, 2 * c] + xh[:2, 2 * c + 1])
        est[2:, c] = 0.5 * (xh[2:, 2 * c + 1] + xh[2:, 2 * c + 2])
    cc = np.arange(nc)
    est = est * np.exp(-1j * np.pi / mpsk * (cc + 1))
    sym = np.round(np.angle(est) / (2 * np.pi / mpsk)).astype(int) % mpsk
    tx_sym = (ref_ph[:nc].T.astype(int) - (128 // mpsk) * (cc + 1)) % 256 // (256 // mpsk)
    # genie: LS fit over all slots with the known element states
    Hg = Y @ X.conj().T @ np.linalg.inv(X @ X.conj().T)
    res = Y - Hg @ X
    if verbose:
        print("  genie fit: residual/total energy %.3f" % (np.sum(np.abs(res)**2) / np.sum(np.abs(Y)**2)))
    data = slice(PRE_CHIPS, nc)
    errs = (sym[:, data] != tx_sym[:, data]).sum(1)
    ref = np.exp(2j * np.pi * tx_sym[:, data] / mpsk)
    evm = np.sqrt(np.mean(np.abs(est[:, data] - ref) ** 2, axis=1))
    slot_T = np.diff(bnd) / fs * 1e6
    if verbose:
        print("  cfo_res %+7.1f Hz  slots %d  T %.2f+-%.2f us  sym errs %s  EVM %s dB  cond(H) %.1f" % (
            f_res, J, slot_T.mean(), slot_T.std(), errs, np.round(20 * np.log10(evm), 1), np.linalg.cond(H)))
    return dict(H=H, est=est, errs=errs, evm=evm, Y=Y, bnd=bnd, sym=sym, tx=tx_sym)


def gen_pattern(path, nframes=4, mpsk=2, seed=1):
    rng = np.random.default_rng(seed)
    frames = []
    for _ in range(nframes):
        d = rng.integers(0, mpsk, size=(4, FRAME_CHIPS))
        frames.append(make_frame(d, mpsk))
    pat = np.concatenate(frames)
    pat.tofile(path)
    np.save(path + ".npy", pat)
    return pat


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "gen":
        gen_pattern(sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
    elif cmd == "rx":
        cap, patf, f_if, mpsk = sys.argv[2], sys.argv[3], float(sys.argv[4]), int(sys.argv[5])
        pat = np.load(patf)
        raw = np.fromfile(cap, dtype=np.int8).astype(np.float32).reshape(-1, 4, 2)
        x = (raw[..., 0] + 1j * raw[..., 1]).T
        seg = x[:, : 1 << 21].sum(0)
        S = np.abs(np.fft.fft(seg * np.hanning(len(seg)))) ** 2
        fr = np.fft.fftfreq(len(seg), 1 / RX_FS)
        m = np.abs(fr - f_if) < 60e3
        f0 = np.sum(fr[m] * S[m]) / np.sum(S[m])
        print("coarse tone centroid %.1f Hz (CFO ~%.1f)" % (f0, f0 - f_if))
        z, fs = ddc(x, f0)
        frames, e = find_frames(z, fs)
        print("frames found:", len(frames))
        nf = pat.shape[0] // FRAME_CHIPS
        for fr_ in frames[:12]:
            # try every pattern frame, keep the best (prototype has no header yet)
            best = None
            for k in range(nf):
                r = decode_frame(z, fs, fr_, pat[k * FRAME_CHIPS:(k + 1) * FRAME_CHIPS], mpsk, verbose=False)
                if best is None or r["errs"].sum() < best[1]["errs"].sum():
                    best = (k, r)
            print(" frame @%.2f ms -> pattern %d" % (fr_[1] / fs * 1e3, best[0]))
            decode_frame(z, fs, fr_, pat[best[0] * FRAME_CHIPS:(best[0] + 1) * FRAME_CHIPS], mpsk)
