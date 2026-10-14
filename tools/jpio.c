// JTAG register writes clocked out by an RP1 PIO state machine, fed by DMA.
// The write period is counted in PIO clocks, so it does not move when the
// CPU or the PCIe link to RP1 is busy, as long as the 8-word TX FIFO never
// runs dry.
//
//   jpio [--div D] [--period US] [--n N] [--reps R] [--verify N] [--hog H] [--addr A]
//
// Pins (RP1 bank 0): TCK 14 and TMS 15 are the two side-set bits, TDI 23 is
// the single OUT pin. TMS only ever follows the fixed 33-clock DR sequence, so
// it lives in the program; the host sends one word per write (bits 0-24 data,
// LSB first, same order as the driver). TDO is not sampled.
//
// Holds the fpga-csi lease, swaps the three pins between PIO and RIO funcsel
// directly in IO_BANK0 so the driver can read back between PIO writes, and
// leaves them on RIO, low, with the TAP in Run-Test/Idle.
//
//   gcc -O2 -pthread -I/usr/include/piolib -o jpio jpio.c -lpio -lm
#include "qrf_hw.h"
#include <pthread.h>
#include <sched.h>
#include "piolib.h"

#define PIN_TCK 14
#define PIN_TMS 15
#define PIN_TDI 23
#define FS_RIO 5

static volatile uint32_t *iob;   // IO_BANK0, GPIOn_CTRL at 8n + 4, funcsel [4:0]

static void funcsel(unsigned fs)
{
    static const int pins[3] = {PIN_TCK, PIN_TMS, PIN_TDI};
    for (int i = 0; i < 3; i++) {
        volatile uint32_t *c = &iob[(8 * pins[i] + 4) / 4];
        *c = (*c & ~0x1fu) | fs;
    }
    (void)iob[(8 * PIN_TCK + 4) / 4];
}

static inline uint64_t cnt(void)
{
    uint64_t v;
    __asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static volatile int hog_run = 1;
static void *hog(void *arg)
{
    (void)arg;
    size_t sz = 32 << 20;
    char *a = malloc(sz), *b = malloc(sz);
    memset(a, 1, sz);
    while (hog_run) memcpy(b, a, sz), memcpy(a, b, sz);
    return NULL;
}

// Closing the PIO with DMA still running wedges the RP1 DMA channel until a
// reboot, so every batch waits for its end marker; give up loudly instead of
// blocking forever if it never comes.
static int wait_marker(PIO pio, uint sm, double timeout_s)
{
    double t_end = mono_s() + timeout_s;
    while (!pio_sm_get_rx_fifo_level(pio, sm))
        if (mono_s() > t_end) return -1;
    pio_sm_get(pio, sm);
    return 0;
}

// side-set: bit0 TCK, bit1 TMS; every half period is one instruction + 1 delay
#define SS(v) pio_encode_sideset(2, v)
#define D1 pio_encode_delay(1)
#define NOP pio_encode_mov(pio_y, pio_y)

int main(int argc, char **argv)
{
    int n = 1024, reps = 20, verify = 200, hogs = 0;
    double div = 3, period_us = 18.9;
    uint8_t addr = 0x2C;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--div")) div = atof(argv[++i]);
        else if (!strcmp(argv[i], "--period")) period_us = atof(argv[++i]);
        else if (!strcmp(argv[i], "--n")) n = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--reps")) reps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--verify")) verify = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--hog")) hogs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--addr")) addr = (uint8_t)strtol(argv[++i], NULL, 0);
        else { fprintf(stderr, "bad arg %s\n", argv[i]); return 2; }
    }

    int fd = open("/dev/csi_stream0", O_RDWR);
    if (fd < 0) { perror("csi"); return 1; }
    ioctl(fd, CSI_IOC_JTAG_SETUP);
    if (lease(fd)) { perror("lease"); return 1; }
    int g = open("/dev/gpiomem0", O_RDWR | O_SYNC);
    if (g < 0) { perror("gpiomem0"); return 1; }
    void *m = mmap(NULL, 0x30000, PROT_READ | PROT_WRITE, MAP_SHARED, g, 0);
    if (m == MAP_FAILED) { perror("mmap"); return 1; }
    iob = (volatile uint32_t *)m;

    uint16_t v0;
    if (jr(fd, addr, &v0)) { perror("read"); return 1; }

    PIO pio = pio0;
    if (PIO_IS_ERR(pio)) { fprintf(stderr, "pio0 open failed\n"); return 1; }
    int sm = pio_claim_unused_sm(pio, true);
    double fclk = clock_get_hz(clk_sys);

    // TCK low and TMS 0 while waiting in pull: the TAP sits in Run-Test/Idle.
    // Instruction 2 burns x+1 clocks, x copied from Y (preloaded once).
    // Per write: 2 + (x+1) + 33 clocks * 4 = x + 135 PIO clocks. Bit 31 of
    // the word marks the last write of a batch: the SM pushes to RX after its
    // Update-DR, which costs one extra clock on that write only.
    uint16_t prog[18] = {
        (uint16_t)(pio_encode_pull(false, true) | SS(0)),
        (uint16_t)(pio_encode_mov(pio_x, pio_y) | SS(0)),
        (uint16_t)(pio_encode_jmp_x_dec(2) | SS(0)),
        (uint16_t)(NOP | SS(2) | D1),                        // TMS 1
        (uint16_t)(NOP | SS(3) | D1),                        //   rise: Select-DR
        (uint16_t)(NOP | SS(0) | D1),
        (uint16_t)(NOP | SS(1) | D1),                        //   rise: Capture-DR
        (uint16_t)(NOP | SS(0) | D1),
        (uint16_t)(pio_encode_set(pio_x, 26) | SS(1) | D1),  //   rise: ghost -> Shift-DR
        (uint16_t)(pio_encode_out(pio_pins, 1) | SS(0) | D1),// TDI changes with TCK low
        (uint16_t)(pio_encode_jmp_x_dec(9) | SS(1) | D1),    //   27 clocks: 25 data + 2
        (uint16_t)(pio_encode_out(pio_x, 5) | SS(2) | D1),   // bits 27-31 -> X
        (uint16_t)(NOP | SS(3) | D1),                        //   rise: Exit1
        (uint16_t)(NOP | SS(2) | D1),
        (uint16_t)(NOP | SS(3) | D1),                        //   rise: Update-DR
        (uint16_t)(pio_encode_jmp_not_x(17) | SS(0) | D1),
        (uint16_t)(pio_encode_push(false, false) | SS(0)),
        (uint16_t)(NOP | SS(1) | D1),                        //   rise: Run-Test/Idle
    };
    pio_program_t pp = {prog, 18, -1, 0};
    uint off = pio_add_program(pio, &pp);
    if (off == PIO_ORIGIN_INVALID) { fprintf(stderr, "no room for program\n"); return 1; }

    long cyc = lround(period_us * 1e-6 * fclk / div);
    long x = cyc - 135;
    if (x < 0) { fprintf(stderr, "period too short for div %.2f (min %.2f us)\n", div, 135 * div / fclk * 1e6); return 1; }
    double per_us = cyc * div / fclk * 1e6;

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, off, off + 17);
    sm_config_set_sideset(&c, 2, false, false);
    sm_config_set_sideset_pins(&c, PIN_TCK);
    sm_config_set_out_pins(&c, PIN_TDI, 1);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_clkdiv(&c, (float)div);
    pio_sm_init(pio, sm, off, &c);

    const uint32_t mask = 1u << PIN_TCK | 1u << PIN_TMS | 1u << PIN_TDI;
    pio_sm_set_pins_with_mask(pio, sm, 0, mask);
    pio_sm_set_pindirs_with_mask(pio, sm, mask, mask);
    pio_gpio_init(pio, PIN_TCK);
    pio_gpio_init(pio, PIN_TMS);
    pio_gpio_init(pio, PIN_TDI);
    unsigned fs_pio = iob[(8 * PIN_TCK + 4) / 4] & 0x1f;

    pio_sm_put_blocking(pio, sm, (uint32_t)x);
    pio_sm_exec(pio, sm, pio_encode_pull(false, true) | SS(0));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_y, pio_osr) | SS(0));
    if (pio_sm_config_xfer(pio, sm, PIO_DIR_TO_SM, 16384, 2)) { fprintf(stderr, "config_xfer failed\n"); return 1; }
    pio_sm_set_enabled(pio, sm, true);
    printf("PIO clk %.1f MHz div %.2f: TCK %.2f MHz, scan %.2f us, period %.4f us (x %ld), pio funcsel %u, fifo %u\n",
           fclk / 1e6, div, fclk / div / 4 / 1e6, 132 * div / fclk * 1e6, per_us, x, fs_pio, pio_get_fifo_depth(pio));

    const uint32_t LAST = 1u << 31;
    int bad = 0;
    srand(4321);
    for (int i = 0; i < verify; i++) {
        uint16_t v = (uint16_t)rand();
        pio_sm_put_blocking(pio, sm, LAST | 1u << 24 | (uint32_t)addr << 16 | v);
        if (wait_marker(pio, sm, 0.5)) { fprintf(stderr, "no end marker on single write\n"); return 1; }
        funcsel(FS_RIO);
        uint16_t r;
        jr(fd, addr, &r);
        funcsel(fs_pio);
        if (r != v) { if (bad < 8) printf("  mismatch wrote 0x%04X read 0x%04X\n", v, r); bad++; }
    }
    printf("verify: %d/%d bad\n", bad, verify);

    pthread_t ht[8];
    for (int i = 0; i < hogs && i < 8; i++) {
        pthread_create(&ht[i], NULL, hog, NULL);
        cpu_set_t cs;
        CPU_ZERO(&cs);
        CPU_SET(i % 3, &cs);
        pthread_setaffinity_np(ht[i], sizeof cs, &cs);
    }
    {
        cpu_set_t cs;
        CPU_ZERO(&cs);
        CPU_SET(3, &cs);
        sched_setaffinity(0, sizeof cs, &cs);
        struct sched_param sp = {.sched_priority = 70};
        sched_setscheduler(0, SCHED_FIFO, &sp);
    }

    uint32_t *w = malloc(n * 4);
    for (int i = 0; i < n; i++) w[i] = 1u << 24 | (uint32_t)addr << 16 | (uint16_t)(i * 0x0101);
    w[n - 1] |= LAST;
    uint64_t f;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
    // excess = wall time from xfer call to end marker, minus n ideal periods:
    // a constant DMA start latency if the FIFO never ran dry, more if it did
    double emin = 1e30, emax = 0, esum = 0, ret_sum = 0;
    for (int rep = 0; rep < reps; rep++) {
        uint64_t t0 = cnt();
        if (pio_sm_xfer_data(pio, sm, PIO_DIR_TO_SM, n * 4, w)) { fprintf(stderr, "xfer failed\n"); break; }
        uint64_t t1 = cnt();
        if (wait_marker(pio, sm, 2.0)) { fprintf(stderr, "no end marker after batch\n"); return 1; }
        uint64_t t2 = cnt();
        double ex = (t2 - t0) * 1e6 / f - n * per_us;
        ret_sum += (t1 - t0) * 1e6 / f;
        if (ex < emin) emin = ex;
        if (ex > emax) emax = ex;
        esum += ex;
        usleep(2000);
    }
    hog_run = 0;
    for (int i = 0; i < hogs && i < 8; i++) pthread_join(ht[i], NULL);
    printf("timed: %d writes x%d, hogs %d, period %.4f us: excess over n*T min %.1f mean %.1f max %.1f us; xfer call returns after %.0f of %.0f us\n",
           n, reps, hogs, per_us, emin, esum / reps, emax, ret_sum / reps, n * per_us);

    funcsel(FS_RIO);
    uint16_t last = (uint16_t)((n - 1) * 0x0101), r;
    jr(fd, addr, &r);
    printf("after timed batch: read 0x%04X expect 0x%04X %s\n", r, last, r == last ? "ok" : "BAD");

    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &pp, off);
    pio_sm_unclaim(pio, sm);
    pio_close(pio);
    funcsel(FS_RIO);
    jw(fd, addr, v0);
    unlease(fd);
    return bad || r != last;
}
