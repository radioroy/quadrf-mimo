// Does the FPGA SPI bridge keep up with back-to-back JTAG writes?
//
// Transmits a tone and measures this unit's own RX leakage power after
// setting MAX2850 TX gain (reg 9) either with a slow single write or as part
// of a batch where other SPI words follow immediately. With the RIO JTAG path
// a batch puts writes ~1.7 us apart instead of ~20 us; if the bridge were
// still shifting the previous word, the gain would land wrong.
//
//   spibb [--reps N] [--lo DB] [--hi DB] [--fif HZ]
//
//   gcc -O2 -pthread -o spibb spibb.c -lm
#include "qrf_hw.h"
#include <pthread.h>

static struct dsi_tx g_dsi;
static volatile int run = 1;

static void *dsi_thread(void *arg)
{
    (void)arg;
    while (run) dsi_queue(&g_dsi, 50);
    return NULL;
}

static uint16_t gain_word(int db) { return (uint16_t)((9u << 10) | ((unsigned)(db << 4 | 0xF) & 0x3FF)); }

static struct csi_rx g_rx;
static int8_t buf[1 << 20];

// mean |x|^2 over all four RX channels, dB re 1 LSB^2
static double rx_db(void)
{
    usleep(3000);
    csi_flush(&g_rx);
    double acc = 0;
    size_t tot = 0;
    double t_end = mono_s() + 0.03;
    while (mono_s() < t_end) {
        size_t n = csi_read(&g_rx, buf, sizeof buf, 50);
        for (size_t i = 0; i < n; i++) acc += (double)buf[i] * buf[i];
        tot += n;
    }
    return tot ? 10 * log10(acc / (tot / 2.0)) : -99;
}

int main(int argc, char **argv)
{
    int reps = 10, lo = 4, hi = 34;
    double f_if = 1e6;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--reps")) reps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--lo")) lo = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--hi")) hi = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fif")) f_if = atof(argv[++i]);
        else { fprintf(stderr, "bad arg %s\n", argv[i]); return 2; }
    }

    if (dsi_open(&g_dsi) != 0) { perror("dsi"); return 1; }
    size_t ns = g_dsi.frame_bytes / 2;
    for (uint32_t k = 0; k < g_dsi.count; k++) {
        int8_t *d = g_dsi.map + k * g_dsi.frame_bytes;
        for (size_t i = 0; i < ns; i++) {
            double ph = 2 * M_PI * f_if * (double)i / TX_FS;
            d[2 * i] = (int8_t)lrint(60 * cos(ph));
            d[2 * i + 1] = (int8_t)lrint(60 * sin(ph));
        }
    }
    pthread_t th;
    pthread_create(&th, NULL, dsi_thread, NULL);
    if (csi_open(&g_rx) != 0) { perror("csi rx"); return 1; }

    int fd = open("/dev/csi_stream0", O_RDWR);
    if (fd < 0) { perror("csi"); return 1; }
    ioctl(fd, CSI_IOC_JTAG_SETUP);
    if (lease(fd)) { perror("lease"); return 1; }
    tx_on(fd, 0);

    const uint16_t pa_on = (uint16_t)((10u << 10) | 0x001);
    const uint16_t mode = (uint16_t)((0u << 10) | 0x00C | (0xF << 5));
    double s_lo = 0, s_hi = 0, e_pair_hi = 0, e_pair_lo = 0, e_first = 0, e_burst = 0;
    double w_pair_hi = 0, w_pair_lo = 0, w_first = 0, w_burst = 0;
    for (int r = 0; r < reps; r++) {
        jw(fd, MAX2850, gain_word(lo));
        double L = rx_db();
        jw(fd, MAX2850, gain_word(hi));
        double H = rx_db();
        s_lo += L, s_hi += H;
        double mid = (L + H) / 2;

        // pair, last one should win
        jw(fd, MAX2850, gain_word(hi));
        struct csi_jtag_reg a[2] = {{MAX2850, gain_word(lo)}, {MAX2850, gain_word(hi)}};
        batch(fd, a, 2);
        double p = rx_db();
        e_pair_hi += fabs(p - H), w_pair_hi += p < mid;

        jw(fd, MAX2850, gain_word(lo));
        struct csi_jtag_reg b[2] = {{MAX2850, gain_word(hi)}, {MAX2850, gain_word(lo)}};
        batch(fd, b, 2);
        p = rx_db();
        e_pair_lo += fabs(p - L), w_pair_lo += p > mid;

        // gain word followed right away by other SPI words: is the first one lost?
        jw(fd, MAX2850, gain_word(lo));
        struct csi_jtag_reg c[3] = {{MAX2850, gain_word(hi)}, {MAX2850, pa_on}, {MAX2850, mode}};
        batch(fd, c, 3);
        p = rx_db();
        e_first += fabs(p - H), w_first += p < mid;

        // 16 alternating gain words, ending high
        jw(fd, MAX2850, gain_word(lo));
        struct csi_jtag_reg d[16];
        for (int k = 0; k < 16; k++) d[k] = (struct csi_jtag_reg){MAX2850, gain_word(k & 1 ? hi : lo)};
        batch(fd, d, 16);
        p = rx_db();
        e_burst += fabs(p - H), w_burst += p < mid;
    }
    printf("slow single writes: lo %d dB -> %.1f dB, hi %d dB -> %.1f dB (step %.1f)\n", lo, s_lo / reps, hi, s_hi / reps,
           (s_hi - s_lo) / reps);
    printf("batch [lo,hi]:            mean |err| %.2f dB, wrong %d/%d\n", e_pair_hi / reps, (int)w_pair_hi, reps);
    printf("batch [hi,lo]:            mean |err| %.2f dB, wrong %d/%d\n", e_pair_lo / reps, (int)w_pair_lo, reps);
    printf("batch [hi,pa_on,mode]:    mean |err| %.2f dB, wrong %d/%d\n", e_first / reps, (int)w_first, reps);
    printf("batch 16 alternating->hi: mean |err| %.2f dB, wrong %d/%d\n", e_burst / reps, (int)w_burst, reps);

    jw(fd, MAX2850, gain_word(lo));
    tx_off(fd);
    unlease(fd);
    run = 0;
    pthread_join(th, NULL);
    return 0;
}
