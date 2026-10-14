#!/usr/bin/env python3
"""Print link stats from one or more quadrf-mimo serve instances.

  tools/wsmon.py quadrf.local quadrf-2.local [-t 20] [-s key=value ...] [--frames]

Each line is one 1 s summary per host. -s sends a settings change first (same keys
as the CONFIG panel). --frames prints every decoded-frame summary instead.
"""
import argparse, base64, json, os, select, socket, struct, sys, time


class WS:
    def __init__(self, host, port=8091):
        self.s = socket.create_connection((host, port), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f"GET /ws HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
        self.s.sendall(req.encode())
        hdr = b""
        while b"\r\n\r\n" not in hdr:
            hdr += self.s.recv(1)
        self.buf = b""

    def send(self, obj):
        p = json.dumps(obj).encode()
        mask = os.urandom(4)
        n = len(p)
        h = bytes([0x81]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack(">H", n))
        self.s.sendall(h + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(p)))

    def _need(self, n):
        while len(self.buf) < n:
            d = self.s.recv(65536)
            if not d:
                raise EOFError
            self.buf += d

    def recv(self):
        self._need(2)
        n = self.buf[1] & 0x7F
        o = 2
        if n == 126:
            self._need(4); n = struct.unpack(">H", self.buf[2:4])[0]; o = 4
        elif n == 127:
            self._need(10); n = struct.unpack(">Q", self.buf[2:10])[0]; o = 10
        self._need(o + n)
        p, self.buf = self.buf[o:o + n], self.buf[o + n:]
        try:
            return json.loads(p)
        except ValueError:
            return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hosts", nargs="+")
    ap.add_argument("-t", type=float, default=15)
    ap.add_argument("-s", action="append", default=[])
    ap.add_argument("--frames", action="store_true")
    a = ap.parse_args()
    ws = {h: WS(h) for h in a.hosts}
    for w in ws.values():
        w.send({"t": "hello"})
        for kv in a.s:
            k, v = kv.split("=", 1)
            w.send({"t": "set", "k": k, "v": float(v)})
    last = {h: None for h in ws}
    t0 = time.time()
    tp = 0
    while time.time() - t0 < a.t:
        r, _, _ = select.select([w.s for w in ws.values()], [], [], 0.5)
        for h, w in ws.items():
            if w.s not in r:
                continue
            m = w.recv()
            while m is not None or w.buf:
                if m and m.get("t") == "st":
                    if a.frames and "f" in m and (last[h] is None or m["f"].get("seq") != last[h].get("f", {}).get("seq")):
                        f = m["f"]
                        print(f"{h[-3:]} seq {f['seq']:2d} mods {''.join(map(str, f['mods']))} snr {f['snr']} "
                              f"model {f['model']} pre {f['pre']} T {f['T']} st {f['stalls']} blk {f['blk']}")
                    last[h] = m
                if len(w.buf) < 2:
                    break
                m = w.recv()
        if not a.frames and time.time() - tp > 1:
            tp = time.time()
            out = []
            for h, m in last.items():
                if not m:
                    continue
                f = m.get("f", {})
                out.append(f"{h[-3:]} rx {m['gp_rx']/1e3:5.1f} tx {m['gp_tx']/1e3:5.1f} kb/s "
                           f"blk {m['bok']}/{m['btot']} peer {m['peer_bok']}/{m['peer_btot']} "
                           f"f {m['fok']}/{m['fbad']}/{m['ffalse']} ber {m['ber']:.1e} "
                           f"snr {f.get('snr')} mods {''.join(map(str, f.get('mods', [])))}")
            print(f"{time.time() - t0:5.1f}  " + "  |  ".join(out), flush=True)


if __name__ == "__main__":
    main()
