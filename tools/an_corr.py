"""Lane-to-lane correlation of an idle cs8 capture: |rho| of wideband noise and of the
DDC output. Independent RX chains give |rho| near 0 on noise.
an_corr.py cap.cs8"""
import sys
import numpy as np
from capio import open_cap, iq, ddc

raw = open_cap(sys.argv[1])
n = min(raw.size // 8, 1 << 21)
for name, x in (("raw", iq(raw, 0, n)), ("ddc", ddc(raw, 500020.5, b=n))):
    x = x - x.mean(0)
    C = (x.conj().T @ x) / len(x)
    d = np.sqrt(np.real(np.diag(C)))
    R = np.abs(C) / np.outer(d, d)
    print(name, "|rho|:")
    for r in range(4):
        print("   " + " ".join("%5.2f" % v for v in R[r]))
