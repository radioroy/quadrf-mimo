// JTAG register write latency on the QuadRF CSI driver.
// Times single CSI_IOC_JTAG_REG_WRITE calls and CSI_IOC_JTAG_BATCH_WRITE of
// the two TX phase registers (0x2C/0x2D), which is one per-element phase chip.
//
//   gcc -O2 -o jtag_bench jtag_bench.c -lm
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <quadrf/fpga_csi.h>

static double now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e6 + t.tv_nsec * 1e-3;
}

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static void report(const char *name, double *v, int n)
{
    qsort(v, n, sizeof(double), cmp_d);
    double s = 0;
    for (int i = 0; i < n; i++) s += v[i];
    printf("%-28s mean %7.2f us  p50 %7.2f  p99 %7.2f  max %8.2f\n",
           name, s / n, v[n / 2], v[(int)(n * 0.99)], v[n - 1]);
}

int main(int argc, char **argv)
{
    int n = argc > 1 ? atoi(argv[1]) : 4000;
    int fd = open("/dev/csi_stream0", O_RDWR);
    if (fd < 0) { perror("open"); return 1; }
    ioctl(fd, CSI_IOC_JTAG_SETUP);
    while (ioctl(fd, CSI_IOC_JTAG_ACQUIRE_LEASE) != 0 && errno == EBUSY) usleep(1000);

    double *v = calloc(n, sizeof(double));
    struct csi_jtag_reg r = { .addr = 0x2C, .value = 0 };

    for (int i = 0; i < n; i++) {
        r.value = (uint16_t)(i & 1 ? 0x8080 : 0x0000);
        double t0 = now_us();
        if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r) != 0) { perror("write"); return 1; }
        v[i] = now_us() - t0;
    }
    report("single REG_WRITE", v, n);

    struct csi_jtag_reg pair[2] = { { .addr = 0x2C }, { .addr = 0x2D } };
    struct csi_jtag_batch b = { .regs_ptr = (uintptr_t)pair, .count = 2 };
    for (int i = 0; i < n; i++) {
        pair[0].value = pair[1].value = (uint16_t)(i & 1 ? 0x8080 : 0x0000);
        double t0 = now_us();
        if (ioctl(fd, CSI_IOC_JTAG_BATCH_WRITE, &b) != 0) { perror("batch2"); return 1; }
        v[i] = now_us() - t0;
    }
    report("BATCH_WRITE x2 (one chip)", v, n);

    enum { BN = 512 };
    struct csi_jtag_reg big[BN];
    for (int i = 0; i < BN; i++) {
        big[i].addr = (i & 1) ? 0x2D : 0x2C;
        big[i].value = (uint16_t)((i >> 1) & 1 ? 0x8080 : 0);
    }
    struct csi_jtag_batch bb = { .regs_ptr = (uintptr_t)big, .count = BN };
    int reps = 40;
    for (int i = 0; i < reps; i++) {
        double t0 = now_us();
        if (ioctl(fd, CSI_IOC_JTAG_BATCH_WRITE, &bb) != 0) { perror("batch512"); return 1; }
        v[i] = (now_us() - t0) / BN;
    }
    report("BATCH_WRITE x512 per-reg", v, reps);

    struct csi_jtag_reg z = { .addr = 0x2C, .value = 0 };
    ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &z);
    z.addr = 0x2D;
    ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &z);
    ioctl(fd, CSI_IOC_JTAG_RELEASE_LEASE);
    ioctl(fd, CSI_IOC_JTAG_RELEASE);
    close(fd);
    return 0;
}
