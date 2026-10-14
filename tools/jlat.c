// Per-write JTAG latency under controlled background load, to find what
// stretches the bit-banged GPIO writes (RP1 sits behind PCIe on the Pi 5).
//
//   jlat [--secs S] [--cpu N] [--prio P] [--mode jtag|mmio]
//        [--dsi off|cont|once] [--hog N] [--rx] [--out FILE]
//
// jtag: one CSI_IOC_JTAG_REG_WRITE (0x2C, TX should be off) per sample.
// mmio: 66 posted stores of 0 to the RP1 RIO SET alias (sets no pins, but is
//       the same PCIe write path the driver's gpiod_set_value() takes), then
//       one non-posted read of RIO SYNC_IN.
//
// Output file: u32 n, u32 nq, then n x {u32 t_us, u32 a_ns, u32 b_ns},
// then nq x u32 DSI queue-return times (us). a = write (or store burst),
// b = SYNC_IN read (mmio only).
//
//   gcc -O2 -pthread -o jlat jlat.c -lm
#include "qrf_hw.h"
#include <pthread.h>
#include <sched.h>

#define RIO0 0x10000
#define RIO_SYNC_IN 0x08
#define RIO_SET 0x2000

static volatile int run = 1;
static double t_start;
static uint32_t *qt;
static uint32_t nq, qmax = 1 << 16;

static inline uint64_t cnt(void)
{
    uint64_t v;
    __asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static inline uint64_t cntfrq(void)
{
    uint64_t v;
    __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(v));
    return v;
}

static int cmp_u32(const void *x, const void *y)
{
    uint32_t a = *(const uint32_t *)x, b = *(const uint32_t *)y;
    return (a > b) - (a < b);
}

static void pin(int cpu, int prio)
{
    cpu_set_t s;
    CPU_ZERO(&s);
    CPU_SET(cpu, &s);
    pthread_setaffinity_np(pthread_self(), sizeof(s), &s);
    if (prio > 0) {
        struct sched_param sp = { .sched_priority = prio };
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }
}

static struct dsi_tx dsi;

static void *dsi_thread(void *arg)
{
    (void)arg;
    while (run) {
        if (dsi_queue(&dsi, 50) == 0 && nq < qmax)
            qt[nq++] = (uint32_t)((mono_s() - t_start) * 1e6);
    }
    return NULL;
}

static int hog_cpu[4];

static void *hog_thread(void *arg)
{
    pin(*(int *)arg, 0);
    size_t n = 4 << 20;
    char *a = malloc(n), *b = malloc(n);
    memset(a, 1, n);
    memset(b, 2, n);
    while (run) {
        memcpy(b, a, n);
        memcpy(a, b, n);
    }
    return NULL;
}

static struct csi_rx crx;

static void *rx_thread(void *arg)
{
    (void)arg;
    pin(1, 50);
    size_t n = 4 << 20;
    int8_t *buf = malloc(n);
    while (run) csi_read(&crx, buf, n, 50);
    return NULL;
}

int main(int argc, char **argv)
{
    double secs = 3;
    int cpu = 3, prio = 70, hogs = 0, rx = 0, mmio = 0;
    const char *dmode = "off", *out = "/tmp/jlat.bin";
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--secs")) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--cpu")) cpu = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--prio")) prio = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--mode")) mmio = !strcmp(argv[++i], "mmio");
        else if (!strcmp(argv[i], "--dsi")) dmode = argv[++i];
        else if (!strcmp(argv[i], "--hog")) hogs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--rx")) rx = 1;
        else if (!strcmp(argv[i], "--out")) out = argv[++i];
        else { fprintf(stderr, "bad arg %s\n", argv[i]); return 2; }
    }

    int fd = open("/dev/csi_stream0", O_RDWR);
    if (fd < 0) { perror("csi"); return 1; }
    volatile uint32_t *rio = NULL;
    if (mmio) {
        int g = open("/dev/gpiomem0", O_RDWR | O_SYNC);
        if (g < 0) { perror("gpiomem0"); return 1; }
        void *m = mmap(NULL, 0x30000, PROT_READ | PROT_WRITE, MAP_SHARED, g, 0);
        if (m == MAP_FAILED) { perror("mmap gpiomem0"); return 1; }
        rio = (volatile uint32_t *)((char *)m + RIO0);
    } else {
        ioctl(fd, CSI_IOC_JTAG_SETUP);
        if (lease(fd)) { perror("lease"); return 1; }
    }

    qt = calloc(qmax, sizeof(uint32_t));
    pthread_t th[8];
    int nth = 0;
    if (strcmp(dmode, "off")) {
        if (dsi_open(&dsi)) { perror("dsi"); return 1; }
        memset(dsi.map, 0, dsi.map_len);
        if (!strcmp(dmode, "once")) {
            for (int k = 0; k < 3; k++) dsi_queue(&dsi, 200);
            usleep(200000);
        }
    }
    if (rx && csi_open(&crx)) { perror("csi ring"); return 1; }

    size_t cap = (size_t)(secs * 200000) + 1024;
    uint32_t *rec = malloc(cap * 3 * sizeof(uint32_t));
    double tick_ns = 1e9 / cntfrq();

    t_start = mono_s();
    if (!strcmp(dmode, "cont")) pthread_create(&th[nth++], NULL, dsi_thread, NULL);
    for (int h = 0, c = 0; h < hogs && c < 4; c++) {
        if (c == cpu) continue;
        hog_cpu[h] = c;
        pthread_create(&th[nth++], NULL, hog_thread, &hog_cpu[h]);
        h++;
    }
    if (rx) pthread_create(&th[nth++], NULL, rx_thread, NULL);

    pin(cpu, prio);
    usleep(100000);
    struct csi_jtag_reg r = { .addr = 0x2C };
    size_t n = 0;
    double t_end = mono_s() + secs;
    while (n < cap) {
        uint64_t t0 = cnt(), t1, t2;
        if (mmio) {
            for (int k = 0; k < 66; k++) rio[RIO_SET / 4] = 0;
            t1 = cnt();
            (void)rio[RIO_SYNC_IN / 4];
            t2 = cnt();
        } else {
            r.value = (uint16_t)(n & 1 ? 0x8080 : 0x0000);
            if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r) != 0) { perror("write"); break; }
            t1 = t2 = cnt();
        }
        double now = mono_s();
        rec[3 * n] = (uint32_t)((now - t_start) * 1e6);
        rec[3 * n + 1] = (uint32_t)((t1 - t0) * tick_ns);
        rec[3 * n + 2] = (uint32_t)((t2 - t1) * tick_ns);
        n++;
        if (now > t_end) break;
    }
    run = 0;
    for (int i = 0; i < nth; i++) pthread_join(th[i], NULL);

    if (!mmio) {
        r.value = 0;
        ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r);
        unlease(fd);
    }

    FILE *f = fopen(out, "wb");
    uint32_t hdr[2] = { (uint32_t)n, nq };
    fwrite(hdr, 4, 2, f);
    fwrite(rec, 4, 3 * n, f);
    fwrite(qt, 4, nq, f);
    fclose(f);

    uint32_t *a = malloc(n * 4);
    for (size_t i = 0; i < n; i++) a[i] = rec[3 * i + 1];
    qsort(a, n, 4, cmp_u32);
    double med = a[n / 2];
    size_t s125 = 0, s2 = 0;
    for (size_t i = 0; i < n; i++) { s125 += a[i] > 1.25 * med; s2 += a[i] > 2 * med; }
    printf("n %zu  med %.2f us  p99 %.2f  p99.9 %.2f  max %.2f  >1.25x %.3f%%  >2x %.4f%%  dsiq %u\n",
           n, med / 1e3, a[(size_t)(n * .99)] / 1e3, a[(size_t)(n * .999)] / 1e3, a[n - 1] / 1e3,
           100.0 * s125 / n, 100.0 * s2 / n, nq);
    return 0;
}
