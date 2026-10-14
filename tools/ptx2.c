// Pattern transmitter: DSI CW tone at --if, per-element phase chips from a
// file of uint8 [nchips][4] (8 bit = 360 deg). Each chip is one 0x2C write
// (el0|el1) followed by one 0x2D write (el2|el3). The file is split into
// frames of --frame chips (<= 512, one batch ioctl each).
//
//   ptx2 --pat p.bin --frame 512 --secs 2 --if 5e5 --gap-us 200
#include "qrf_hw.h"
#include <pthread.h>
#include <signal.h>

static volatile sig_atomic_t run = 1;
static void stop(int s) { (void)s; run = 0; }
static struct dsi_tx g_dsi;

static void *dsi_thread(void *arg)
{
    (void)arg;
    while (run) dsi_queue(&g_dsi, 50);
    return NULL;
}

int main(int argc, char **argv)
{
    double f_if = 5e5, secs = 2, amp = 120, gap_us = 200, f_if2 = 0;
    int frame = 512, idle_off = 0;
    const char *pat = NULL;
    for (int i = 1; i < argc - 1; i++) {
        if (!strcmp(argv[i], "--if")) f_if = atof(argv[++i]);
        else if (!strcmp(argv[i], "--secs")) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pat")) pat = argv[++i];
        else if (!strcmp(argv[i], "--frame")) frame = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--gap-us")) gap_us = atof(argv[++i]);
        else if (!strcmp(argv[i], "--amp")) amp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--idle-off")) idle_off = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--if2")) f_if2 = atof(argv[++i]);
    }
    if (!pat || frame < 1 || frame > 512) { fprintf(stderr, "bad args\n"); return 1; }
    FILE *f = fopen(pat, "rb");
    if (!f) { perror(pat); return 1; }
    fseek(f, 0, SEEK_END);
    long nb = ftell(f);
    fseek(f, 0, SEEK_SET);
    int nchips = (int)(nb / 4);
    uint8_t *p = malloc(nb);
    if (fread(p, 1, nb, f) != (size_t)nb) return 1;
    fclose(f);
    int nframes = (nchips + frame - 1) / frame;

    signal(SIGINT, stop);
    signal(SIGTERM, stop);
    if (dsi_open(&g_dsi) != 0) { perror("dsi"); return 1; }
    size_t ns = g_dsi.frame_bytes / 2;
    f_if = round(f_if * ns / TX_FS) * TX_FS / ns;
    double w = 2 * M_PI * f_if / TX_FS;
    double w2 = 0, a1 = amp, a2 = 0;
    if (f_if2 != 0) {
        f_if2 = round(f_if2 * ns / TX_FS) * TX_FS / ns;
        w2 = 2 * M_PI * f_if2 / TX_FS;
        a1 = a2 = amp / 2;
        fprintf(stderr, "ptx2: second tone %.1f Hz\n", f_if2);
    }
    for (uint32_t k = 0; k < g_dsi.count; k++) {
        int8_t *d = g_dsi.map + k * g_dsi.frame_bytes;
        for (size_t i = 0; i < ns; i++) {
            d[2 * i] = (int8_t)lrint(a1 * cos(w * i) + a2 * cos(w2 * i));
            d[2 * i + 1] = (int8_t)lrint(a1 * sin(w * i) + a2 * sin(w2 * i));
        }
    }
    pthread_t th;
    pthread_create(&th, NULL, dsi_thread, NULL);
    usleep(120000);

    int fd = open("/dev/csi_stream0", O_RDWR);
    ioctl(fd, CSI_IOC_JTAG_SETUP);
    lease(fd);
    tx_phases(fd, 0, 0, 0, 0);
    tx_on(fd, 0);

    struct csi_jtag_reg *regs = malloc(sizeof(*regs) * 2 * frame);
    double t0 = mono_s(), tb = 0;
    long sent = 0;
    int fi = 0;
    while (run && mono_s() - t0 < secs) {
        int c0 = fi * frame, n = nchips - c0 < frame ? nchips - c0 : frame;
        for (int c = 0; c < n; c++) {
            const uint8_t *q = p + 4 * (c0 + c);
            regs[2 * c] = (struct csi_jtag_reg){ .addr = 0x2C, .value = (uint16_t)(q[0] << 8 | q[1]) };
            regs[2 * c + 1] = (struct csi_jtag_reg){ .addr = 0x2D, .value = (uint16_t)(q[2] << 8 | q[3]) };
        }
        double a = mono_s();
        if (batch(fd, regs, 2 * n) != 0) { perror("batch"); break; }
        tb += mono_s() - a;
        sent++;
        fi = (fi + 1) % nframes;
        if (idle_off) {
            jw(fd, 0x02, 0);
            usleep((useconds_t)gap_us);
            jw(fd, 0x02, 0xF);
        } else if (gap_us > 0) {
            usleep((useconds_t)gap_us);
        }
    }
    fprintf(stderr, "ptx2: %ld frames of %d chips, %.2f us/reg, if %.1f Hz\n",
            sent, frame, 1e6 * tb / (sent * 2.0 * frame), f_if);
    tx_off(fd);
    tx_phases(fd, 0, 0, 0, 0);
    unlease(fd);
    ioctl(fd, CSI_IOC_JTAG_RELEASE);
    run = 0;
    pthread_join(th, NULL);
    return 0;
}
