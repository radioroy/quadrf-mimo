// JTAG register writes bit-banged straight into RP1's RIO block from userspace,
// bypassing gpiolib. Holds the fpga-csi lease so the kernel never drives the
// pins at the same time, and leaves the TAP in Run-Test/Idle with TCK/TMS/TDI
// low, which is what the driver expects on its next scan.
//
//   jfast [--pad N] [--n N] [--verify N] [--addr A]
//
// --pad N:    N extra no-op stores per TCK half period (slows TCK)
// --n N:      timed writes per batch (default 1024)
// --verify N: N random values written fast, each read back through the driver
//
//   gcc -O2 -o jfast jfast.c
#include "qrf_hw.h"

// RP1 bank 0 RIO inside /dev/gpiomem0; atomic aliases at +0x1000/2000/3000.
#define RIO0 0x10000
#define RIO_OUT 0x00
#define RIO_SYNC_IN 0x08
#define RIO_XOR 0x1000

#define TCK (1u << 14)
#define TMS (1u << 15)
#define TDO (1u << 18)
#define TDI (1u << 23)

static volatile uint32_t *rio;
static int pad;

static inline uint64_t cnt(void)
{
    uint64_t v;
    __asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static inline void xr(uint32_t m)
{
    rio[RIO_XOR / 4] = m;
    for (int i = 0; i < pad; i++) rio[RIO_XOR / 4] = 0;
}

// Same 33-clock sequence as the driver's jtag_shift_dr25_write_fast():
// Select-DR, Capture-DR, ghost, 25 data, 2 extra, Exit1, Update, RTI.
// Data changes ride on the falling edge (TAP samples TMS/TDI on the rising
// edge), so every clock is exactly two stores to the XOR alias.
static void scan_write(uint32_t word)
{
    uint32_t seq[33];
    int k = 0;
    seq[k++] = TMS;
    seq[k++] = 0;
    seq[k++] = 0;
    for (int i = 0; i < 25; i++) seq[k++] = (word >> i) & 1 ? TDI : 0;
    seq[k++] = 0;
    seq[k++] = 0;
    seq[k++] = TMS;
    seq[k++] = TMS;
    seq[k++] = 0;

    uint32_t cur = 0;
    if (seq[0]) xr(seq[0]);
    cur = seq[0];
    xr(TCK);
    for (int c = 1; c < 33; c++) {
        xr(TCK | (cur ^ seq[c]));
        cur = seq[c];
        xr(TCK);
    }
    xr(TCK | cur);
}

static inline void reg_write(uint8_t a, uint16_t v) { scan_write(1u << 24 | (uint32_t)a << 16 | v); }

int main(int argc, char **argv)
{
    int n = 1024, verify = 200;
    uint8_t addr = 0x2C;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--pad")) pad = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--n")) n = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--verify")) verify = atoi(argv[++i]);
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
    rio = (volatile uint32_t *)((char *)m + RIO0);

    uint16_t v0;
    if (jr(fd, addr, &v0)) { perror("read"); return 1; }
    uint32_t out = rio[RIO_OUT / 4];
    printf("addr 0x%02X = 0x%04X  RIO OUT tck %d tms %d tdi %d\n", addr, v0, !!(out & TCK), !!(out & TMS), !!(out & TDI));
    if (out & (TCK | TMS | TDI)) { fprintf(stderr, "pins not idle\n"); return 1; }

    // kernel round trip first, so a fast-path failure is not a readback quirk
    jw(fd, addr, 0x1234);
    uint16_t k;
    jr(fd, addr, &k);
    printf("kernel write 0x1234 -> read 0x%04X\n", k);

    int bad = 0;
    srand(12345);
    for (int i = 0; i < verify; i++) {
        uint16_t v = (uint16_t)rand();
        reg_write(addr, v);
        uint16_t r;
        jr(fd, addr, &r);
        if (r != v) { if (bad < 8) printf("  mismatch wrote 0x%04X read 0x%04X\n", v, r); bad++; }
    }
    printf("verify: %d/%d bad (pad %d)\n", bad, verify, pad);

    double tick = 1e9 / 54e6;
    {
        uint64_t f;
        __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
        tick = 1e9 / f;
    }
    double best = 1e30, sum = 0;
    for (int rep = 0; rep < 20; rep++) {
        uint64_t t0 = cnt();
        for (int i = 0; i < n; i++) reg_write(addr, (uint16_t)(i * 0x0101));
        (void)rio[RIO_SYNC_IN / 4];
        double us = (cnt() - t0) * tick / 1e3 / n;
        if (us < best) best = us;
        sum += us;
    }
    printf("timed: %d writes x20, per write best %.3f us mean %.3f us (incl. final drain read)\n", n, best, sum / 20);

    uint16_t last = (uint16_t)((n - 1) * 0x0101), r;
    jr(fd, addr, &r);
    printf("after timed batch: read 0x%04X expect 0x%04X %s\n", r, last, r == last ? "ok" : "BAD");
    jw(fd, addr, v0);
    unlease(fd);
    return bad || r != last;
}
