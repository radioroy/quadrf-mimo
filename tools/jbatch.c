// Times CSI_IOC_JTAG_BATCH_WRITE of 1024 TX phase writes, the way the link
// sends a frame, and checks the last value through the driver's read path.
// With --period, also runs the paced batch ioctl and reports its schedule.
//
//   jbatch [--reps N] [--verify N] [--period NS] [--hog N]
//
//   gcc -O2 -pthread -o jbatch jbatch.c
#include "qrf_hw.h"
#include <pthread.h>

#ifndef CSI_IOC_JTAG_BATCH_PACED
struct csi_jtag_batch_paced {
    __u64 regs_ptr;
    __u32 count;
    __u32 period_ns;
    __u64 t_first_ns;
    __u32 slips;
    __u32 max_late_ns;
};
#define CSI_IOC_JTAG_BATCH_PACED _IOWR(CSI_IOC_MAGIC, 0x17, struct csi_jtag_batch_paced)
#endif

static void *hog(void *arg)
{
    cpu_set_t s;
    CPU_ZERO(&s);
    CPU_SET((int)(long)arg, &s);
    pthread_setaffinity_np(pthread_self(), sizeof(s), &s);
    size_t n = 4 << 20;
    char *a = malloc(n), *b = malloc(n);
    memset(a, 1, n);
    memset(b, 2, n);
    for (;;) { memcpy(b, a, n); memcpy(a, b, n); }
    return NULL;
}

int main(int argc, char **argv)
{
    int reps = 50, verify = 2000, hogs = 0;
    unsigned period = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--reps")) reps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--verify")) verify = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--period")) period = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--hog")) hogs = atoi(argv[++i]);
    }
    for (int h = 0; h < hogs && h < 3; h++) {
        pthread_t t;
        pthread_create(&t, NULL, hog, (void *)(long)h);
    }
    {
        cpu_set_t s;
        CPU_ZERO(&s);
        CPU_SET(3, &s);
        sched_setaffinity(0, sizeof(s), &s);
        struct sched_param sp = { .sched_priority = 70 };
        sched_setscheduler(0, SCHED_FIFO, &sp);
    }
    int fd = open("/dev/csi_stream0", O_RDWR);
    if (fd < 0) { perror("csi"); return 1; }
    ioctl(fd, CSI_IOC_JTAG_SETUP);
    if (lease(fd)) { perror("lease"); return 1; }

    static struct csi_jtag_reg r[1024];
    double best = 1e9, sum = 0, worst = 0;
    for (int rep = 0; rep < reps; rep++) {
        for (int i = 0; i < 1024; i++) {
            r[i].addr = i & 1 ? 0x2D : 0x2C;
            r[i].value = (uint16_t)((rep * 7919 + i * 40503) & 0xFFFF);
        }
        double t0 = mono_s();
        if (batch(fd, r, 1024)) { perror("batch"); return 1; }
        double us = (mono_s() - t0) * 1e6 / 1024;
        best = us < best ? us : best;
        worst = us > worst ? us : worst;
        sum += us;
    }
    uint16_t a, b;
    jr(fd, 0x2C, &a);
    jr(fd, 0x2D, &b);
    printf("batch 1024: per write best %.3f mean %.3f worst %.3f us; last 2C %s 2D %s\n", best, sum / reps, worst,
           a == r[1022].value ? "ok" : "BAD", b == r[1023].value ? "ok" : "BAD");

    if (period) {
        unsigned slips = 0, frames_slipped = 0, worst_late = 0, last_bad = 0;
        double span_min = 1e9, span_max = 0;
        for (int rep = 0; rep < reps; rep++) {
            // fresh values each frame so the readback proves this batch landed
            for (int i = 0; i < 1024; i++)
                r[i].value = (uint16_t)((rep * 104729 + i * 7907 + 0x5A5A) & 0xFFFF);
            struct csi_jtag_batch_paced p = { .regs_ptr = (uintptr_t)r, .count = 1024, .period_ns = period };
            double t0 = mono_s();
            if (ioctl(fd, CSI_IOC_JTAG_BATCH_PACED, &p)) { perror("paced"); return 1; }
            double span = (mono_s() - t0) * 1e6;
            span_min = span < span_min ? span : span_min;
            span_max = span > span_max ? span : span_max;
            slips += p.slips;
            frames_slipped += p.slips > 0;
            worst_late = p.max_late_ns > worst_late ? p.max_late_ns : worst_late;
            jr(fd, 0x2C, &a);
            jr(fd, 0x2D, &b);
            last_bad += a != r[1022].value || b != r[1023].value;
        }
        printf("paced %u ns x1024 x%d: ioctl span %.1f..%.1f us (ideal %.1f)  slips %u in %u frames  worst late %.2f us  last values bad %u/%d\n",
               period, reps, span_min, span_max, 1023 * period / 1e3, slips, frames_slipped, worst_late / 1e3, last_bad, reps);
    }

    int bad = 0;
    double tr = 0;
    srand(777);
    for (int i = 0; i < verify; i++) {
        uint8_t ad = i & 1 ? 0x2D : 0x2C;
        uint16_t v = (uint16_t)rand(), got;
        jw(fd, ad, v);
        double t0 = mono_s();
        jr(fd, ad, &got);
        tr += mono_s() - t0;
        bad += got != v;
    }
    printf("single write+read verify: %d/%d bad, read %.2f us each\n", bad, verify, tr * 1e6 / verify);
    jw(fd, 0x2C, 0);
    jw(fd, 0x2D, 0);
    unlease(fd);
    return bad != 0;
}
