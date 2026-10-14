"""Spatial signature of one TX element from a single-element capture: dominant RX lane vector
(CFO-invariant, via SVD of the 4-lane covariance) scaled by its rms amplitude.
an_sig.py cap.cs8 [out.npy]"""
import sys
import numpy as np
from capio import open_cap, ddc

z = ddc(open_cap(sys.argv[1]), 500020.5)
p = (np.abs(z) ** 2).sum(1)
on = np.flatnonzero(p > 0.5 * np.percentile(p, 95))
z = z[on[0] + 500:on[-1] - 500]
R = z.T @ z.conj() / len(z)
w, V = np.linalg.eigh(R)
v = V[:, -1] * np.sqrt(w[-1])
v *= np.exp(-1j * np.angle(v[0]))
print("lane |h| %s  ang %s deg  eig %s" % (np.round(np.abs(v), 1), np.round(np.degrees(np.angle(v))),
      np.round(10 * np.log10(w[::-1] / w[-1]), 1)))
if len(sys.argv) > 2:
    np.save(sys.argv[2], v)
