// Exercises the fpga-csi JTAG write stream (CSI_IOC_JTAG_STREAM_*):
// timed start, back-to-back queueing for a few seconds, idle slots, AT slots,
// error paths, and teardown on close. Writes the TX phase registers 0x2C/0x2D
// and checks the last values through the normal read path after each stop.
//
//   jstream [--period NS] [--secs S] [--hog N]
//
//   gcc -O2 -pthread -I. -o jstream jstream.c     (needs the patched fpga_csi.h)
#include "qrf_hw.h"
#include <pthread.h>

#ifndef CSI_IOC_JTAG_STREAM_START
#error "fpga_csi.h without JTAG streams"
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

static uint64_t mono_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + t.tv_nsec;
}

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static void fill(struct csi_jtag_reg *r, unsigned n, unsigned seed)
{
    for (unsigned i = 0; i < n; i++) {
        r[i].addr = i & 1 ? 0x2D : 0x2C;
        r[i].value = (uint16_t)((seed * 104729u + i * 7907u + 0x5A5A) & 0xFFFF);
    }
}

static int start(int fd, unsigned period, uint64_t at, struct csi_jtag_reg *pre, unsigned n,
                 struct csi_jtag_stream_start *s)
{
    memset(s, 0, sizeof(*s));
    s->period_ns = period;
    s->start_ns = at;
    s->regs_ptr = (uintptr_t)pre;
    s->count = n;
    return ioctl(fd, CSI_IOC_JTAG_STREAM_START, s);
}

static int stop(int fd, unsigned flags) { return ioctl(fd, CSI_IOC_JTAG_STREAM_STOP, &flags); }

static void status(int fd, struct csi_jtag_stream_status *st)
{
    memset(st, 0, sizeof(*st));
    ioctl(fd, CSI_IOC_JTAG_STREAM_STATUS, st);
}

static int last_ok(int fd, uint16_t a2c, uint16_t a2d)
{
    uint16_t a = 0, b = 0;
    jr(fd, 0x2C, &a);
    jr(fd, 0x2D, &b);
    return a == a2c && b == a2d;
}

int main(int argc, char **argv)
{
    unsigned period = 5000, hogs = 0;
    double secs = 3;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--period")) period = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--secs")) secs = atof(argv[++i]);
        else if (!strcmp(argv[i], "--hog")) hogs = (unsigned)atoi(argv[++i]);
    }
    for (unsigned h = 0; h < hogs && h < 3; h++) {
        pthread_t t;
        pthread_create(&t, NULL, hog, (void *)(long)h);
    }
    int fd = open("/dev/csi_stream0", O_RDWR);
    if (fd < 0) { perror("csi"); return 1; }
    ioctl(fd, CSI_IOC_JTAG_SETUP);
    if (lease(fd)) { perror("lease"); return 1; }

    static struct csi_jtag_reg r[4096];
    struct csi_jtag_stream_start s;
    struct csi_jtag_stream_status st;
    struct csi_jtag_stream_queue q;

    // 1. timed start with preloaded slots, drain
    fill(r, 1024, 1);
    uint64_t want = mono_ns() + 20000000;
    if (start(fd, period, want, r, 1024, &s)) { perror("start"); return 1; }
    printf("start: period %u ns  slot0 - requested %+.2f us  +/- %.2f us  lead %u slots\n",
           s.period_ns, ((int64_t)(s.t_slot0_ns - want)) / 1e3, s.t_slot0_err_ns / 1e3, s.lead_slots);
    // the SM enable is a firmware mailbox call (~13 us idle, ~80 us under DRAM
    // load); slot 0 lands inside it, so the window must start at or after
    // the requested time and contain it within a few us
    CHECK(s.t_slot0_ns + s.t_slot0_err_ns >= want && s.t_slot0_ns - s.t_slot0_err_ns <= want + 5000,
          "slot0 %+lld ns, err %u ns", (long long)(s.t_slot0_ns - want), s.t_slot0_err_ns);
    {
        struct csi_jtag_reg x = { .addr = 0x2C };
        CHECK(ioctl(fd, CSI_IOC_JTAG_REG_READ, &x) < 0 && errno == EBUSY, "REG_READ during stream not EBUSY");
        struct csi_jtag_batch_paced p = { .regs_ptr = (uintptr_t)r, .count = 4, .period_ns = 5000 };
        CHECK(ioctl(fd, CSI_IOC_JTAG_BATCH_PACED, &p) < 0 && errno == EBUSY, "BATCH_PACED during stream not EBUSY");
    }
    double t0 = mono_s();
    CHECK(stop(fd, CSI_JTAG_STREAM_DRAIN) == 0, "stop drain: %s", strerror(errno));
    double tstop = mono_s() - t0;
    uint64_t tend = mono_ns();
    printf("drain 1024: stop returned %.2f ms after start returned, %.1f us after the last slot ended\n",
           tstop * 1e3, ((int64_t)(tend - (s.t_slot0_ns + 1024ull * s.period_ns))) / 1e3);
    CHECK(last_ok(fd, r[1022].value, r[1023].value), "preload last values");

    // 2. idle stream must not write anything
    jw(fd, 0x2C, 0x1234);
    jw(fd, 0x2D, 0x4321);
    if (start(fd, period, 0, NULL, 0, &s)) { perror("start idle"); return 1; }
    usleep(500000);
    status(fd, &st);
    printf("idle 0.5 s: slot_now %llu committed %llu underruns %u\n",
           (unsigned long long)st.slot_now, (unsigned long long)st.committed, st.underruns);
    CHECK(st.underruns == 0, "idle underruns %u", st.underruns);

    // 3. AT slot in the future, then one already committed
    fill(r, 2, 2);
    q = (struct csi_jtag_stream_queue){ .regs_ptr = (uintptr_t)r, .count = 2, .flags = CSI_JTAG_STREAM_AT,
                                        .slot = st.committed + st.lead_slots + 20000 };
    CHECK(ioctl(fd, CSI_IOC_JTAG_STREAM_QUEUE, &q) == 0, "AT queue: %s", strerror(errno));
    printf("AT: slot %llu at %+.3f ms from now\n", (unsigned long long)q.slot,
           ((int64_t)(q.t_first_ns - mono_ns())) / 1e6);
    struct csi_jtag_stream_queue q2 = { .regs_ptr = (uintptr_t)r, .count = 1, .flags = CSI_JTAG_STREAM_AT, .slot = 0 };
    CHECK(ioctl(fd, CSI_IOC_JTAG_STREAM_QUEUE, &q2) < 0 && errno == ETIME, "past slot not ETIME");
    q2.slot = q.slot + 1;
    CHECK(ioctl(fd, CSI_IOC_JTAG_STREAM_QUEUE, &q2) < 0 && errno == EINVAL, "overlap not EINVAL");
    CHECK(stop(fd, CSI_JTAG_STREAM_DRAIN) == 0, "stop AT: %s", strerror(errno));
    CHECK(last_ok(fd, r[0].value, r[1].value), "AT values");

    // 4. continuous queueing, writes back to back with the ring kept full
    if (start(fd, period, 0, NULL, 0, &s)) { perror("start cont"); return 1; }
    unsigned chunks = 0, eagain = 0;
    uint64_t first = 0;
    double tq = mono_s(), worst_call = 0;
    while (mono_s() - tq < secs) {
        fill(r, 1024, 100 + chunks);
        q = (struct csi_jtag_stream_queue){ .regs_ptr = (uintptr_t)r, .count = 1024 };
        double c0 = mono_s();
        if (ioctl(fd, CSI_IOC_JTAG_STREAM_QUEUE, &q)) {
            if (errno == EAGAIN) { eagain++; continue; }
            perror("queue");
            break;
        }
        double c = mono_s() - c0;
        worst_call = c > worst_call ? c : worst_call;
        if (!chunks)
            first = q.slot;
        CHECK(q.slot == first + 1024ull * chunks, "chunk %u at slot %llu", chunks, (unsigned long long)q.slot);
        chunks++;
    }
    status(fd, &st);
    CHECK(stop(fd, CSI_JTAG_STREAM_DRAIN) == 0, "stop cont: %s", strerror(errno));
    double rate = chunks * 1024.0 / (mono_s() - tq);
    printf("continuous: %u x1024 writes, %.0f writes/s (ideal %.0f), underruns %u, worst queue call %.2f ms\n",
           chunks, rate, 1e9 / s.period_ns, st.underruns, worst_call * 1e3);
    CHECK(st.underruns == 0, "continuous underruns %u", st.underruns);
    CHECK(last_ok(fd, r[1022].value, r[1023].value), "continuous last values");

    // 5. paced batch still works and goes through the stream
    fill(r, 1024, 7);
    struct csi_jtag_batch_paced p = { .regs_ptr = (uintptr_t)r, .count = 1024, .period_ns = period };
    CHECK(ioctl(fd, CSI_IOC_JTAG_BATCH_PACED, &p) == 0, "paced: %s", strerror(errno));
    printf("paced batch: slips %u  t_first err %.2f us\n", p.slips, p.max_late_ns / 1e3);
    CHECK(last_ok(fd, r[1022].value, r[1023].value), "paced last values");

    // 6. close with a stream running; the driver has to stop it
    if (start(fd, period, 0, NULL, 0, &s)) { perror("start close"); return 1; }
    fill(r, 1024, 9);
    q = (struct csi_jtag_stream_queue){ .regs_ptr = (uintptr_t)r, .count = 1024 };
    ioctl(fd, CSI_IOC_JTAG_STREAM_QUEUE, &q);
    close(fd);
    fd = open("/dev/csi_stream0", O_RDWR);
    ioctl(fd, CSI_IOC_JTAG_SETUP);
    lease(fd);
    status(fd, &st);
    CHECK(!st.active, "stream survived close");
    uint16_t v;
    srand(5);
    int bad = 0;
    for (int i = 0; i < 200; i++) {
        uint16_t w = (uint16_t)rand();
        jw(fd, 0x2C, w);
        jr(fd, 0x2C, &v);
        bad += v != w;
    }
    CHECK(!bad, "%d/200 bad RIO writes after close", bad);

    jw(fd, 0x2C, 0);
    jw(fd, 0x2D, 0);
    unlease(fd);
    printf("%s (%d failures)\n", fails ? "FAILED" : "all ok", fails);
    return fails != 0;
}
