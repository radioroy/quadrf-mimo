"""Superposition check from a `quadrf-mimo txstates` capture. Dwells alternate
ref(all 0) / state s; each state mean is normalized by its neighbouring reference means (removes
CFO, LO wander and gain drift), then fitted with the linear model sum_k h_k exp(j theta_k) + c.
an_states.py cap.cs8 [out.npz]"""
import sys
import numpy as np
from capio import open_cap, ddc, RX_FS

FS = RX_FS / 16
z = ddc(open_cap(sys.argv[1]), 500020.5).astype(complex)
p = np.convolve((np.abs(z) ** 2).sum(1), np.ones(2048) / 2048, "same")
lo, hi = np.percentile(p, 5), np.percentile(p, 99)
on = np.flatnonzero(p > 0.5 * (lo + hi))
z = z[on[0] + 100:on[-1] - 100]
fe = np.angle(np.sum(z[16:] * z[:-16].conj())) / (2 * np.pi * 16) * FS
z = z * np.exp(-2j * np.pi * fe * np.arange(len(z)) / FS)[:, None]
W, G = 20, 6
cs = np.vstack([np.zeros((1, 4)), np.cumsum(z, 0)])
n = np.arange(W + G, len(z) - W - G)
D = np.linalg.norm((cs[n + G + W] - cs[n + G]) / W - (cs[n - G] - cs[n - G - W]) / W, axis=1)
thr = 0.25 * np.percentile(D, 99.5)
pk = []
i = 0
while i < len(D):
    if D[i] > thr:
        j = i + int(np.argmax(D[i:i + int(40e-6 * FS)]))
        pk.append(n[j])
        i = j + int(100e-6 * FS)
    else:
        i += 1
pk = np.array(pk)
# timing-based dwells: change detection misses states that only move the weak elements.
# Batches are 1024 writes (64 dwells of 16); the first batch starts at the change point that
# ends the long pre-pattern quiet, later ones at the strongest change just after the gap.
Tw = 18.96e-6 * FS
seglen = np.diff(pk)
# batch 0 opens with a ref dwell equal to the pre-pattern state: the change found is ref -> s1
st0 = pk[1:][np.argmax(seglen)] - 16 * Tw
starts = [st0]
for k in range(1, 8):
    a0 = int(starts[-1] + 1024 * Tw + 150e-6 * FS) if k > 1 else int(st0 + 1024 * Tw + 150e-6 * FS)
    a1 = int(a0 + 900e-6 * FS)
    j = np.argmax(D[a0 - n[0]:a1 - n[0]]) + a0
    starts.append(j)
bnd = []
for k, s0 in enumerate(starts):
    nd = min(64, 510 - 64 * k)
    bnd += [s0 + j * 16 * Tw for j in range(nd)] + ([s0 + nd * 16 * Tw] if k == 7 or nd < 64 else [])
bnd = np.array(bnd)
# per-dwell local frequency (the carrier wanders by ~1 kHz over ms), phase-referenced dwell
# means, then dwell-to-dwell phase chained from the local frequencies
g = 12
m, fk, nc = [], [], []
for k in range(510):
    a = int(bnd[k]) + g
    b = int(bnd[k] + 16 * Tw) - g
    x = z[a:b]
    Dl = 32
    f = np.angle(np.sum(x[Dl:] * x[:-Dl].conj())) / (2 * np.pi * Dl) * FS
    c = (a + b) / 2
    m.append(np.mean(x * np.exp(-2j * np.pi * f * (np.arange(a, b) - c) / FS)[:, None], 0))
    fk.append(f)
    nc.append(c)
m, fk, nc = np.array(m), np.array(fk), np.array(nc)
print("cfo %.0f Hz  batch starts (ms) %s" % (fe, np.round(np.array(starts) / FS * 1e3, 3)))
# each state against its two neighbouring refs, projected to the state's centre with the refs'
# own local frequencies (~10 rad of carrier per dwell, so no long-range chaining)
idx = [2 * i + 1 for i in range(255) if 2 * i + 2 < len(m)]
Y = m[idx]
Rp = np.array([0.5 * (m[k - 1] * np.exp(2j * np.pi * fk[k - 1] * (nc[k] - nc[k - 1]) / FS)
                      + m[k + 1] * np.exp(-2j * np.pi * fk[k + 1] * (nc[k + 1] - nc[k]) / FS)) for k in idx])
R0 = m[0]
r = (Rp @ R0.conj()) / np.vdot(R0, R0).real
fwd = np.array([m[k - 1] * np.exp(2j * np.pi * fk[k - 1] * (nc[k] - nc[k - 1]) / FS) for k in idx])
bwd = np.array([m[k + 1] * np.exp(-2j * np.pi * fk[k + 1] * (nc[k + 1] - nc[k]) / FS) for k in idx])
print("ref fwd/bwd prediction mismatch: median %.3f" % np.median(np.linalg.norm(fwd - bwd, axis=1) / np.linalg.norm(fwd, axis=1)))
Yn = np.vstack([R0[None, :], Y / r[:, None]])
ns = len(Yn)
codes = np.array([[(s >> (2 * k)) & 3 for k in range(4)] for s in range(ns)])
A = np.hstack([np.exp(1j * np.pi / 2 * codes), np.ones((ns, 1))])
Hf, *_ = np.linalg.lstsq(A, Yn, rcond=None)
res = Yn - A @ Hf
sig = np.sum(np.abs(A[:, :4] @ Hf[:4]) ** 2)
print("linear model fit over %d states: %.1f dB   |h| %s  |c| %.1f" % (
    ns, 10 * np.log10(sig / np.sum(np.abs(res) ** 2)), np.round(np.linalg.norm(Hf[:4], axis=1), 1), np.linalg.norm(Hf[4])))
# per-element amplitude vs the number of other elements in phase with it
e = np.linalg.norm(res, axis=1) / np.sqrt(sig / ns)
print("residual rms / signal rms per state: median %.3f  90%% %.3f  max %.3f" % (np.median(e), np.percentile(e, 90), e.max()))
tot = np.linalg.norm(Yn, axis=1)
print("total |y| over states: min %.1f  median %.1f  max %.1f  (linear prediction min %.1f max %.1f)" % (
    tot.min(), np.median(tot), tot.max(), np.linalg.norm(A @ Hf, axis=1).min(), np.linalg.norm(A @ Hf, axis=1).max()))
if len(sys.argv) > 2:
    np.savez(sys.argv[2], Y=Yn, codes=codes, H=Hf)
