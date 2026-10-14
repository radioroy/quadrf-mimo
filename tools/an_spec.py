"""Strongest spectral lines per channel of a cs8 capture (baseband, LO = 0).
an_spec.py cap.cs8 t0_ms t1_ms [npeaks]"""
import sys
import numpy as np
from capio import open_cap, ms, psd

raw = open_cap(sys.argv[1])
npk = int(sys.argv[4]) if len(sys.argv) > 4 else 8
f, P, enbw = psd(raw, ms(sys.argv[2]), ms(sys.argv[3]), 1 << 16)
for c in range(4):
    db = 10 * np.log10(P[:, c] + 1e-20)
    pk, d = [], db.copy()
    for _ in range(npk):
        i = int(np.argmax(d))
        pk.append((f[i], d[i]))
        d[max(0, i - 40):i + 40] = -999
    print("ch%d floor %.1f dBFS/bin  total %.1f dBFS  " % (c, np.median(db), 10 * np.log10(P[:, c].sum() / enbw)) +
          "  ".join("%+8.1fk %5.1f" % (fr / 1e3, v) for fr, v in pk))
