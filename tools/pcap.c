// Raw 4-channel interleaved cs8 capture from the CSI ring.
//   pcap out.cs8 <seconds> [delay_s]
#include "qrf_hw.h"

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: pcap out.cs8 secs [delay]\n"); return 1; }
    double secs = atof(argv[2]);
    double delay = argc > 3 ? atof(argv[3]) : 0;
    struct csi_rx c;
    if (csi_open(&c) != 0) { perror("csi"); return 1; }
    size_t want = (size_t)(secs * RX_FS) * 8;
    int8_t *buf = malloc(want);
    if (delay > 0) usleep((useconds_t)(delay * 1e6));
    csi_flush(&c);
    size_t got = 0;
    double t0 = mono_s();
    while (got < want && mono_s() - t0 < secs + 2) {
        size_t n = csi_read(&c, buf + got, want - got, 100);
        got += n;
    }
    FILE *f = fopen(argv[1], "wb");
    fwrite(buf, 1, got, f);
    fclose(f);
    double rms[4] = { 0 };
    size_t nf = got / 8;
    for (size_t i = 0; i < nf; i++)
        for (int ch = 0; ch < 4; ch++) {
            double a = buf[i * 8 + ch * 2], b = buf[i * 8 + ch * 2 + 1];
            rms[ch] += a * a + b * b;
        }
    fprintf(stderr, "pcap: %zu frames (%.1f ms), rms", nf, 1e3 * nf / RX_FS);
    for (int ch = 0; ch < 4; ch++) fprintf(stderr, " %.1f", sqrt(rms[ch] / (nf ? nf : 1)));
    fprintf(stderr, "\n");
    return 0;
}
