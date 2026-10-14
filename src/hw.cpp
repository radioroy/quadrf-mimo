#include "hw.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

extern "C" {
#include <quadrf/fpga_csi.h>
}

// Paced batch from the RIO fpga-csi build; older drivers answer ENOTTY.
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

namespace qm {

namespace {
constexpr uint8_t MAX2850 = 0x42;

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

std::string run_cmd(const std::string &cmd)
{
    std::string out;
    FILE *p = popen(cmd.c_str(), "r");
    if (!p) return out;
    char buf[512];
    while (fgets(buf, sizeof buf, p)) out += buf;
    pclose(p);
    return out;
}
}  // namespace

Hw::~Hw() { close_all(); }

bool Hw::open_all(std::string &err)
{
    jfd_ = ::open("/dev/csi_stream0", O_RDWR);
    rfd_ = ::open("/dev/csi_stream0", O_RDWR | O_NONBLOCK);
    if (jfd_ < 0 || rfd_ < 0) { err = std::string("csi_stream0: ") + strerror(errno); return false; }
    ioctl(jfd_, CSI_IOC_JTAG_SETUP);

    csi_ring_info ri{};
    if (ioctl(rfd_, CSI_IOC_GET_RING_INFO, &ri) != 0) { err = "CSI ring info"; return false; }
    ring_size_ = ri.ring_size;
    long pg = sysconf(_SC_PAGESIZE);
    ring_len_ = ((size_t)ri.ring_size + pg - 1) & ~((size_t)pg - 1);
    void *m = mmap(nullptr, ring_len_, PROT_READ, MAP_SHARED, rfd_, 0);
    if (m == MAP_FAILED) { err = "CSI ring mmap"; return false; }
    ring_ = (const int8_t *)m;

    dfd_ = ::open("/dev/dsi_stream0", O_RDWR | O_NONBLOCK);
    if (dfd_ < 0) { err = std::string("dsi_stream0: ") + strerror(errno); return false; }
    dsi_fb_info info{};
    for (int i = 0; i < 200; i++) {
        if (ioctl(dfd_, DSI_IOC_GET_FB_INFO, &info) == 0 && info.fb_bytes) break;
        usleep(5000);
    }
    if (!info.fb_bytes) { info.fb_bytes = 3317760; info.fb_count = 3; }
    dsi_frame_ = info.fb_bytes;
    dsi_count_ = info.fb_count;
    dsi_len_ = ((size_t)dsi_count_ * dsi_frame_ + pg - 1) & ~((size_t)pg - 1);
    m = mmap(nullptr, dsi_len_, PROT_READ | PROT_WRITE, MAP_SHARED, dfd_, 0);
    if (m == MAP_FAILED) { err = "DSI mmap"; return false; }
    dsi_map_ = (int8_t *)m;
    return true;
}

void Hw::close_all()
{
    stop_tone();
    if (dsi_map_) { munmap(dsi_map_, dsi_len_); dsi_map_ = nullptr; }
    if (ring_) { munmap((void *)ring_, ring_len_); ring_ = nullptr; }
    if (jfd_ >= 0) { ioctl(jfd_, CSI_IOC_JTAG_RELEASE_LEASE); ioctl(jfd_, CSI_IOC_JTAG_RELEASE); ::close(jfd_); }
    if (rfd_ >= 0) ::close(rfd_);
    if (dfd_ >= 0) ::close(dfd_);
    jfd_ = rfd_ = dfd_ = -1;
}

bool Hw::lease()
{
    for (int i = 0; i < 10000; i++) {
        if (ioctl(jfd_, CSI_IOC_JTAG_ACQUIRE_LEASE) == 0) return true;
        if (errno == ENOTTY) return true;
        if (errno != EBUSY && errno != EINTR) return false;
        usleep(200);
    }
    return false;
}

void Hw::unlease() { ioctl(jfd_, CSI_IOC_JTAG_RELEASE_LEASE); }

bool Hw::write(uint8_t addr, uint16_t v)
{
    csi_jtag_reg r{};
    r.addr = addr;
    r.value = v;
    for (int i = 0; i < 200; i++) {
        if (ioctl(jfd_, CSI_IOC_JTAG_REG_WRITE, &r) == 0) return true;
        if (errno != EBUSY && errno != EINTR) return false;
        usleep(200);
    }
    return false;
}

bool Hw::batch(const JtagReg *r, unsigned n, double period_us, PaceStats *ps)
{
    static thread_local std::vector<csi_jtag_reg> buf;
    buf.resize(n);
    for (unsigned i = 0; i < n; i++) { buf[i].addr = r[i].addr; buf[i].value = r[i].value; }
    if (period_us > 0 && paced_ok_) {
        csi_jtag_batch_paced p{};
        p.regs_ptr = (uintptr_t)buf.data();
        p.count = n;
        p.period_ns = (uint32_t)std::lround(period_us * 1e3);
        for (int i = 0; i < 50; i++) {
            if (ioctl(jfd_, CSI_IOC_JTAG_BATCH_PACED, &p) == 0) {
                if (ps) { ps->paced = true; ps->slips = p.slips; ps->max_late_us = p.max_late_ns * 1e-3f; }
                return true;
            }
            if (errno == ENOTTY) { paced_ok_ = false; break; }
            if (errno != EBUSY && errno != EINTR) return false;
            usleep(200);
        }
    }
    if (ps) *ps = PaceStats{};
    csi_jtag_batch b{};
    b.regs_ptr = (uintptr_t)buf.data();
    b.count = n;
    for (int i = 0; i < 50; i++) {
        if (ioctl(jfd_, CSI_IOC_JTAG_BATCH_WRITE, &b) == 0) return true;
        if (errno != EBUSY && errno != EINTR) return false;
        usleep(200);
    }
    return false;
}

bool Hw::spi_tx(unsigned reg, unsigned d10)
{
    return write(MAX2850, (uint16_t)(((reg & 0x3F) << 10) | (d10 & 0x3FF)));
}

// Main0 0x00C = TX mode at 20 MHz, bits 8:5 enable TX channels 0..3.
// Main10 bit 0 is PA bias. FPGA 0x23 = 0 routes the RF switch to TX (the RX mux
// then leaves the polarization switch), 0x02 is the baseband element mask.
void Hw::tx_on(unsigned mask)
{
    mask &= 0xF;
    write(0x23, 0x0000);
    spi_tx(0, 0x00C | (mask << 5));
    spi_tx(10, 0x001);
    write(0x02, (uint16_t)mask);
}

void Hw::tx_off()
{
    spi_tx(0, 0x002);
    write(0x23, 0x0001);
    spi_tx(10, 0x000);
    write(0x02, 0x0000);
}

int Hw::tx_gain_cap()
{
    const char *p = getenv("QUADRF_TX_FULL_POWER");
    if (!p || !*p) p = "/var/lib/quadrf/tx_full_power";
    return access(p, F_OK) == 0 ? 63 : 40;
}

bool Hw::pio_paced()
{
    auto on = [](const char *p) {
        char c = 0;
        FILE *f = fopen(p, "r");
        if (!f) return false;
        const bool ok = fread(&c, 1, 1, f) == 1 && (c == 'Y' || c == '1');
        fclose(f);
        return ok;
    };
    return on("/sys/module/fpga_csi/parameters/jtag_rio") && on("/sys/module/fpga_csi/parameters/jtag_pio");
}

int Hw::set_tx_gain(int db)
{
    db = std::clamp(db, 0, tx_gain_cap());
    spi_tx(9, (unsigned)((db << 4) | 0xF));
    return db;
}

int Hw::set_rx_gain(int db)
{
    db = std::clamp(db, 0, 63);
    write(0x6A, (uint16_t)db);
    return db;
}

void Hw::tx_phases(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
{
    write(0x2C, (uint16_t)(p0 << 8 | p1));
    write(0x2D, (uint16_t)(p2 << 8 | p3));
}

double Hw::start_tone(double f_if, double amp)
{
    stop_tone();
    size_t ns = dsi_frame_ / 2;
    f_if = std::round(f_if * ns / TX_FS) * TX_FS / ns;
    double w = 2 * M_PI * f_if / TX_FS;
    for (uint32_t k = 0; k < dsi_count_; k++) {
        int8_t *d = dsi_map_ + k * dsi_frame_;
        for (size_t i = 0; i < ns; i++) {
            d[2 * i] = (int8_t)lrint(amp * cos(w * i));
            d[2 * i + 1] = (int8_t)lrint(amp * sin(w * i));
        }
    }
    dsi_run_ = true;
    dsi_th_ = std::thread([this] {
        // Every frame holds the same tone and the CRTC keeps scanning out the last flipped
        // buffer, so one pass over the staging frames is enough. Each flip makes the driver's
        // flip thread memcpy 3.3 MB into the framebuffer every 19.3 ms, and DRAM traffic backs
        // up the CPU's MMIO stores to RP1 over PCIe: with the CSI ring also being drained, ~47%
        // of the JTAG writes in the ~1 ms after each flip ran 25-125 us instead of 19 us.
        // QM_DSI_CONT restores continuous queueing for comparison.
        static const bool cont = getenv("QM_DSI_CONT") != nullptr;
        uint32_t flips = 0;
        while (dsi_run_ && (cont || flips < dsi_count_)) {
            pollfd p{dfd_, POLLOUT, 0};
            if (poll(&p, 1, 50) > 0 && ioctl(dfd_, DSI_IOC_QUEUE_NEXT) == 0) flips++;
        }
    });
    return f_if;
}

void Hw::stop_tone()
{
    dsi_run_ = false;
    if (dsi_th_.joinable()) dsi_th_.join();
}

size_t Hw::rx_read(int8_t *dst, size_t max_bytes, int timeout_ms)
{
    pollfd p{rfd_, POLLIN, 0};
    if (poll(&p, 1, timeout_ms) <= 0) return 0;
    csi_ring_info ri{};
    if (ioctl(rfd_, CSI_IOC_GET_RING_INFO, &ri) != 0) return 0;
    uint32_t used = ri.head >= ri.tail ? ri.head - ri.tail : ri.ring_size - (ri.tail - ri.head);
    last_fill_ = ri.ring_size ? (float)(used + ri.span_bytes) / ri.ring_size : 0.f;
    size_t n = used < max_bytes ? used : max_bytes;
    n &= ~(size_t)7;
    if (!n) return 0;
    size_t first = ri.ring_size - ri.tail;
    if (n <= first) {
        memcpy(dst, ring_ + ri.tail, n);
    } else {
        memcpy(dst, ring_ + ri.tail, first);
        memcpy(dst + first, ring_, n - first);
    }
    uint32_t cn = (uint32_t)n;
    ioctl(rfd_, CSI_IOC_CONSUME_BYTES, &cn);
    return n;
}

void Hw::rx_flush()
{
    csi_ring_info ri{};
    if (ioctl(rfd_, CSI_IOC_GET_RING_INFO, &ri) == 0) {
        uint32_t used = ri.head >= ri.tail ? ri.head - ri.tail : ri.ring_size - (ri.tail - ri.head);
        if (used) ioctl(rfd_, CSI_IOC_CONSUME_BYTES, &used);
    }
}

size_t Hw::rx_span()
{
    csi_ring_info ri{};
    if (ioctl(rfd_, CSI_IOC_GET_RING_INFO, &ri) != 0) return 0;
    return ri.span_bytes;
}

bool Hw::rx_full()
{
    csi_ring_info ri{};
    if (ioctl(rfd_, CSI_IOC_GET_RING_INFO, &ri) != 0) return true;
    uint32_t used = ri.head >= ri.tail ? ri.head - ri.tail : ri.ring_size - (ri.tail - ri.head);
    return used + ri.span_bytes >= ri.ring_size;
}

bool Hw::apply_required_rx()
{
    // 4-channel interleaved capture with no FPGA beam steering or test tone;
    // the receiver estimates the full 4x4 channel itself.
    int r = system("quadrf-jtag --rx antennas=15,interleave=1,tone_en=0,autosteer=0,"
                   "p1=0,p2=0,p3=0,p4=0 >/dev/null 2>&1");
    int t = system("quadrf-jtag --tx tone_en=0,tx_follow_rx=0,p1=0,p2=0,p3=0,p4=0 >/dev/null 2>&1");
    return r == 0 && t == 0;
}

bool Hw::apply_defaults(int rx_gain, int tx_gain, double freq_mhz, int bw)
{
    char cmd[512];
    snprintf(cmd, sizeof cmd,
             "quadrf-jtag --rx antennas=15,interleave=1,tone_en=0,autosteer=0,p1=0,p2=0,p3=0,p4=0,"
             "freq=%.3f,gain=%d,bw=%d >/dev/null 2>&1",
             freq_mhz, rx_gain, bw);
    int a = system(cmd);
    // --tx with gain= also enters TX mode; turn it straight back off, the
    // link layer keys TX per burst.
    snprintf(cmd, sizeof cmd,
             "quadrf-jtag --tx antennas=15,tone_en=0,tx_follow_rx=0,p1=0,p2=0,p3=0,p4=0,"
             "freq=%.3f,gain=%d,bw=%d >/dev/null 2>&1; quadrf-jtag --tx off >/dev/null 2>&1",
             freq_mhz, tx_gain, bw);
    int b = system(cmd);
    return a == 0 && b == 0;
}

RadioStatus Hw::query_status()
{
    RadioStatus s;
    auto num = [](const std::string &line) {
        size_t c = line.find(':');
        return c == std::string::npos ? 0.0 : atof(line.c_str() + c + 1);
    };
    std::string rx = run_cmd("quadrf-jtag --status rx 2>/dev/null");
    std::string tx = run_cmd("quadrf-jtag --status tx 2>/dev/null");
    if (rx.empty() && tx.empty()) return s;
    s.valid = true;
    size_t pos = 0;
    while (pos < rx.size()) {
        size_t e = rx.find('\n', pos);
        std::string l = rx.substr(pos, e == std::string::npos ? std::string::npos : e - pos);
        pos = e == std::string::npos ? rx.size() : e + 1;
        size_t f = l.find_first_not_of(" \t");
        if (f == std::string::npos) continue;
        l = l.substr(f);
        if (l.rfind("- LO Frequency:", 0) == 0) s.rx_freq = num(l);
        else if (l.rfind("- Gain:", 0) == 0) s.rx_gain = (int)num(l);
        else if (l.rfind("- Interleaved Mode:", 0) == 0) s.rx_interleave = l.find("ON") != std::string::npos;
        else if (l.rfind("- Auto Steer:", 0) == 0) s.rx_autosteer = l.find("ON") != std::string::npos;
        else if (l.rfind("- Test Tone:", 0) == 0) s.rx_tone = l.find("ON") != std::string::npos;
        else if (l.rfind("- AGC:", 0) == 0) s.rx_agc = l.find("Enabled") != std::string::npos;
        else if (l.rfind("- Polarization:", 0) == 0) s.rx_pol = l.find("RHCP") != std::string::npos ? "RHCP" : "LHCP";
        else if (l.rfind("- Digital Bandwidth (k):", 0) == 0) s.rx_bw_k = l.find("Disabled") != std::string::npos ? -1 : (int)num(l);
    }
    pos = 0;
    while (pos < tx.size()) {
        size_t e = tx.find('\n', pos);
        std::string l = tx.substr(pos, e == std::string::npos ? std::string::npos : e - pos);
        pos = e == std::string::npos ? tx.size() : e + 1;
        size_t f = l.find_first_not_of(" \t");
        if (f == std::string::npos) continue;
        l = l.substr(f);
        if (l.rfind("- LO Frequency:", 0) == 0) s.tx_freq = num(l);
        else if (l.rfind("- Gain:", 0) == 0) s.tx_gain = (int)num(l);
        else if (l.rfind("- Tx is", 0) == 0) s.tx_on = l.find("ON") != std::string::npos;
    }
    return s;
}

}  // namespace qm
