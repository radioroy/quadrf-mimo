// Phase-probe transmitter. DSI carries a CW tone at --if; the FPGA per-element
// TX phase registers (0x2C: el0|el1, 0x2D: el2|el3, 8 bit = 360 deg) are
// stepped through a 16-state factorial pattern so the receiver can solve for
// every TX->RX path.
//
//   ptx --if 2e6 --secs 2 [--hold 1] [--steps 16] [--mask 0xF]
#include "qrf_hw.h"
#include <signal.h>
#include <pthread.h>

static volatile sig_atomic_t run = 1;
static void stop(int s) { (void)s; run = 0; }

static void fill_tone(int8_t *dst, size_t n, double f, double amp)
{
    // f is snapped by the caller to an integer number of cycles per frame,
    // so every frame is identical and re-queuing never splices phase.
    double w = 2 * M_PI * f / TX_FS;
    for (size_t i = 0; i < n; i++) {
        double ph = w * (double)i;
        dst[2 * i] = (int8_t)lrint(amp * cos(ph));
        dst[2 * i + 1] = (int8_t)lrint(amp * sin(ph));
    }
}

static struct dsi_tx g_dsi;

static void *dsi_thread(void *arg)
{
    (void)arg;
    while (run) dsi_queue(&g_dsi, 50);
    return NULL;
}

int main(int argc, char **argv)
{
    double f_if = 2e6, secs = 2, amp = 120;
    int hold = 1, steps = 16, mask = 0xF;
    for (int i = 1; i < argc - 1; i++) {
        if (!strcmp(argv[i], "--mask")) { mask = (int)strtol(argv[++i], NULL, 0) & 0xF; continue; }
        if (!strcmp(argv[i], "--if")) f_if = atof(argv[++i]);
        else if (!strcmp(argv[i], "--secs")) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--hold")) hold = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--steps")) steps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--amp")) amp = atof(argv[++i]);
    }
    signal(SIGINT, stop);
    signal(SIGTERM, stop);

    if (dsi_open(&g_dsi) != 0) { perror("dsi"); return 1; }
    size_t ns = g_dsi.frame_bytes / 2;
    double cyc = round(f_if * ns / TX_FS);
    f_if = cyc * TX_FS / ns;
    for (uint32_t k = 0; k < g_dsi.count; k++)
        fill_tone(g_dsi.map + k * g_dsi.frame_bytes, ns, f_if, amp);
    fprintf(stderr, "ptx: tone %.3f Hz, %zu samples/frame, %u fbs\n", f_if, ns, g_dsi.count);

    pthread_t th;
    pthread_create(&th, NULL, dsi_thread, NULL);
    usleep(120000);

    int fd = open("/dev/csi_stream0", O_RDWR);
    if (fd < 0) { perror("csi"); return 1; }
    ioctl(fd, CSI_IOC_JTAG_SETUP);
    lease(fd);
    tx_phases(fd, 0, 0, 0, 0);
    tx_on(fd, 0);
    if (mask != 0xF) {
        // same as tx_on, but E_TX (MAX2850 reg 0 bits 8:5) and the FPGA PA enables at 0x02
        // only for the elements in mask
        spi(fd, MAX2850, 0, 0x00C | (mask << 5));
        jw(fd, 0x02, (uint16_t)mask);
    }

    // One "chip" = 0x2C then 0x2D. State s: element k at 180 deg if bit k set.
    enum { NREG = 1024 };
    struct csi_jtag_reg regs[NREG];
    int nchips = NREG / 2;
    uint64_t chip_ctr = 0;
    double t0 = mono_s(), tb_sum = 0;
    long nb = 0;
    while (run && mono_s() - t0 < secs) {
        for (int c = 0; c < nchips; c++) {
            unsigned s = (unsigned)((chip_ctr + c) / hold) % (unsigned)steps;
            uint8_t p[4];
            for (int k = 0; k < 4; k++) p[k] = (s >> k) & 1 ? 128 : 0;
            regs[2 * c] = (struct csi_jtag_reg){ .addr = 0x2C, .value = (uint16_t)(p[0] << 8 | p[1]) };
            regs[2 * c + 1] = (struct csi_jtag_reg){ .addr = 0x2D, .value = (uint16_t)(p[2] << 8 | p[3]) };
        }
        chip_ctr += nchips;
        double tb = mono_s();
        if (batch(fd, regs, NREG) != 0) { perror("batch"); break; }
        tb_sum += mono_s() - tb;
        nb++;
    }
    double el = mono_s() - t0;
    fprintf(stderr, "ptx: %ld batches, %.2f us/reg in batch, duty %.1f%%\n",
            nb, 1e6 * tb_sum / (nb * NREG), 100 * tb_sum / el);

    tx_off(fd);
    tx_phases(fd, 0, 0, 0, 0);
    unlease(fd);
    ioctl(fd, CSI_IOC_JTAG_RELEASE);
    run = 0;
    pthread_join(th, NULL);
    return 0;
}
