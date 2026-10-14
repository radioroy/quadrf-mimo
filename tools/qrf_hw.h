// Minimal QuadRF hardware access for bench tools: JTAG registers over the
// CSI driver, CSI RX ring, DSI TX framebuffers.
#pragma once
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <quadrf/fpga_csi.h>

#define MAX2850 0x42
#define MAX2851 0x43

// TX: 2-lane DSI, 175 MHz byte clock, 3072x1080 payload per frame.
// RX: 4-lane CSI, /4 for 4-channel interleaved cs8.
#define TX_FS (0.5 * 175000000.0 * (3317760.0 / 3372462.0))
#define RX_FS (((65536.0 / 76714.0) * 175000000.0) / 4.0)

struct dsi_fb_info {
    uint64_t fb_bytes;
    uint32_t fb_count;
    uint32_t head;
    uint32_t tail;
    int32_t queued;
    uint32_t _pad;
};
#define DSI_IOC_GET_FB_INFO _IOR('D', 0x10, struct dsi_fb_info)
#define DSI_IOC_QUEUE_NEXT _IO('D', 0x11)

static inline double mono_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static inline int jw(int fd, uint8_t a, uint16_t v)
{
    struct csi_jtag_reg r = { .addr = a, .value = v };
    for (int i = 0; i < 200; i++) {
        if (ioctl(fd, CSI_IOC_JTAG_REG_WRITE, &r) == 0) return 0;
        if (errno != EBUSY && errno != EINTR) return -1;
        usleep(200);
    }
    return -1;
}

static inline int jr(int fd, uint8_t a, uint16_t *v)
{
    struct csi_jtag_reg r = { .addr = a };
    for (int i = 0; i < 200; i++) {
        if (ioctl(fd, CSI_IOC_JTAG_REG_READ, &r) == 0) { *v = r.value; return 0; }
        if (errno != EBUSY && errno != EINTR) return -1;
        usleep(200);
    }
    return -1;
}

static inline int spi(int fd, uint8_t chip, unsigned reg, unsigned d10)
{
    return jw(fd, chip, (uint16_t)(((reg & 0x3F) << 10) | (d10 & 0x3FF)));
}

static inline int lease(int fd)
{
    for (int i = 0; i < 5000; i++) {
        if (ioctl(fd, CSI_IOC_JTAG_ACQUIRE_LEASE) == 0) return 0;
        if (errno == ENOTTY) return 0;
        if (errno != EBUSY) return -1;
        usleep(200);
    }
    return -1;
}

static inline void unlease(int fd) { ioctl(fd, CSI_IOC_JTAG_RELEASE_LEASE); }

static inline int batch(int fd, struct csi_jtag_reg *r, unsigned n)
{
    struct csi_jtag_batch b = { .regs_ptr = (uintptr_t)r, .count = n };
    return ioctl(fd, CSI_IOC_JTAG_BATCH_WRITE, &b);
}

// PA + FPGA TX path on, MAX2850 MODE=TX with E_TX for all four elements.
// Gain and LO are left as the control page set them.
static inline void tx_on(int fd, int bw40)
{
    jw(fd, 0x23, 0x0000);
    spi(fd, MAX2850, 0, 0x00C | (bw40 ? 2 : 0) | (0xF << 5));
    spi(fd, MAX2850, 10, 0x001);
    jw(fd, 0x02, 0x000F);
}

static inline void tx_off(int fd)
{
    spi(fd, MAX2850, 0, 0x002);
    jw(fd, 0x23, 0x0001);
    spi(fd, MAX2850, 10, 0x000);
    jw(fd, 0x02, 0x0000);
}

static inline void tx_phases(int fd, uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
    jw(fd, 0x2C, (uint16_t)(p0 << 8 | p1));
    jw(fd, 0x2D, (uint16_t)(p2 << 8 | p3));
}

struct dsi_tx {
    int fd;
    int8_t *map;
    size_t frame_bytes, map_len;
    uint32_t count;
};

static inline int dsi_open(struct dsi_tx *d)
{
    d->fd = open("/dev/dsi_stream0", O_RDWR | O_NONBLOCK);
    if (d->fd < 0) return -1;
    struct dsi_fb_info info = { 0 };
    for (int i = 0; i < 200; i++) {
        if (ioctl(d->fd, DSI_IOC_GET_FB_INFO, &info) == 0 && info.fb_bytes) break;
        usleep(5000);
    }
    if (!info.fb_bytes) { info.fb_bytes = 3317760; info.fb_count = 3; }
    d->frame_bytes = info.fb_bytes;
    d->count = info.fb_count;
    long pg = sysconf(_SC_PAGESIZE);
    d->map_len = ((size_t)d->count * d->frame_bytes + pg - 1) & ~((size_t)pg - 1);
    d->map = mmap(NULL, d->map_len, PROT_READ | PROT_WRITE, MAP_SHARED, d->fd, 0);
    return d->map == MAP_FAILED ? -1 : 0;
}

// Queue the next staging frame (already filled). Returns 0 if queued.
static inline int dsi_queue(struct dsi_tx *d, int timeout_ms)
{
    struct pollfd p = { .fd = d->fd, .events = POLLOUT };
    if (poll(&p, 1, timeout_ms) <= 0) return -1;
    return ioctl(d->fd, DSI_IOC_QUEUE_NEXT);
}

static inline int8_t *dsi_head(struct dsi_tx *d)
{
    struct dsi_fb_info info = { 0 };
    if (ioctl(d->fd, DSI_IOC_GET_FB_INFO, &info) != 0) return NULL;
    return d->map + (size_t)(info.head % d->count) * d->frame_bytes;
}

struct csi_rx {
    int fd;
    const int8_t *ring;
    uint32_t size;
};

static inline int csi_open(struct csi_rx *c)
{
    c->fd = open("/dev/csi_stream0", O_RDWR | O_NONBLOCK);
    if (c->fd < 0) return -1;
    struct csi_ring_info ri;
    if (ioctl(c->fd, CSI_IOC_GET_RING_INFO, &ri) != 0) return -1;
    c->size = ri.ring_size;
    long pg = sysconf(_SC_PAGESIZE);
    size_t ml = ((size_t)ri.ring_size + pg - 1) & ~((size_t)pg - 1);
    c->ring = mmap(NULL, ml, PROT_READ, MAP_SHARED, c->fd, 0);
    return c->ring == MAP_FAILED ? -1 : 0;
}

static inline void csi_flush(struct csi_rx *c)
{
    struct csi_ring_info ri;
    if (ioctl(c->fd, CSI_IOC_GET_RING_INFO, &ri) == 0) {
        uint32_t used = ri.head >= ri.tail ? ri.head - ri.tail : ri.ring_size - (ri.tail - ri.head);
        if (used) ioctl(c->fd, CSI_IOC_CONSUME_BYTES, &used);
    }
}

// Copy up to max_bytes (multiple of 8) of available ring data; returns bytes.
static inline size_t csi_read(struct csi_rx *c, int8_t *dst, size_t max_bytes, int timeout_ms)
{
    struct pollfd p = { .fd = c->fd, .events = POLLIN };
    if (poll(&p, 1, timeout_ms) <= 0) return 0;
    struct csi_ring_info ri;
    if (ioctl(c->fd, CSI_IOC_GET_RING_INFO, &ri) != 0) return 0;
    uint32_t used = ri.head >= ri.tail ? ri.head - ri.tail : ri.ring_size - (ri.tail - ri.head);
    size_t n = used < max_bytes ? used : max_bytes;
    n &= ~(size_t)7;
    if (!n) return 0;
    size_t first = ri.ring_size - ri.tail;
    if (n <= first) {
        memcpy(dst, c->ring + ri.tail, n);
    } else {
        memcpy(dst, c->ring + ri.tail, first);
        memcpy(dst + first, c->ring, n - first);
    }
    uint32_t cn = (uint32_t)n;
    ioctl(c->fd, CSI_IOC_CONSUME_BYTES, &cn);
    return n;
}
