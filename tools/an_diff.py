"""Lines that appear between two windows (e.g. idle vs TX on), optionally in two captures.
an_diff.py cap.cs8 q0_ms q1_ms [cap2.cs8] t0_ms t1_ms [nlines]"""
import sys
import numpy as np
from capio import open_cap, ms, psd

A = sys.argv[1:]
raw = open_cap(A[0])
raw2 = open_cap(A.pop(3)) if len(A) >= 6 and not A[3].replace(".", "").isdigit() else raw
nl = int(A[5]) if len(A) > 5 else 10
f, q, _ = psd(raw, ms(A[1]), ms(A[2]))
_, t, _ = psd(raw2, ms(A[3]), ms(A[4]))
d = (t - q).sum(1)
for _ in range(nl):
    i = int(np.argmax(d))
    print("%+9.1f kHz  added %s dBFS  idle %s" % (
        f[i] / 1e3, " ".join("%6.1f" % (10 * np.log10(max(t[i, c] - q[i, c], 1e-12))) for c in range(4)),
        " ".join("%6.1f" % (10 * np.log10(q[i, c])) for c in range(4))))
    d[max(0, i - 30):i + 30] = -1
