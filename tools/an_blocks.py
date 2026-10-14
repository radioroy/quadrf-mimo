"""Per-channel tone level in fixed time blocks after the DDC (dBFS, int8 full scale).
an_blocks.py cap.cs8 [block_ms] [f_if]"""
import sys
import numpy as np
from capio import open_cap, ddc, RX_FS

raw = open_cap(sys.argv[1])
B = float(sys.argv[2]) if len(sys.argv) > 2 else 10
f_if = float(sys.argv[3]) if len(sys.argv) > 3 else 500020.5
z = ddc(raw, f_if)
FS = RX_FS / 16
n = int(B * 1e-3 * FS)
for i in range(len(z) // n):
    s = z[i * n:(i + 1) * n]
    # coherent level over 0.1 ms sub-blocks, so a few kHz of CFO does not average it away
    m = int(1e-4 * FS)
    lv = np.sqrt((np.abs(s[:len(s) // m * m].reshape(-1, m, 4).mean(1)) ** 2).mean(0))
    print("%6.0f ms  " % (i * B) + " ".join("%6.1f" % (20 * np.log10(v / 127 + 1e-9)) for v in lv))
