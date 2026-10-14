// QuadRF hardware access: JTAG registers through the CSI driver, MAX2850 TX
// control, DSI baseband tone, CSI 4-channel RX ring.
#pragma once
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace qm {

// TX: 2-lane DSI at 175 MHz byte clock, 3072x1080 payload per frame, cs8.
// RX: 4-lane CSI, 4-channel interleaved cs8 (8 bytes per sample instant).
constexpr double TX_FS = 0.5 * 175e6 * (3317760.0 / 3372462.0);
constexpr double RX_FS = ((65536.0 / 76714.0) * 175e6) / 4.0;

struct JtagReg { uint8_t addr; uint16_t value; };

// What the driver reports for a paced batch (fpga-csi RIO build).
struct PaceStats {
    bool paced = false;      // false: driver has no paced batch, writes ran back to back
    unsigned slips = 0;      // writes that started > period/4 late and moved the schedule
    float max_late_us = 0;
};

struct RadioStatus {
    bool valid = false;
    double rx_freq = 0, tx_freq = 0;
    int rx_gain = -1, tx_gain = -1;
    bool rx_interleave = false, rx_autosteer = false, rx_tone = false, rx_agc = false;
    bool tx_on = false;
    std::string rx_pol;
    int rx_bw_k = -1;
    double age_s = 0;
};

class Hw {
public:
    ~Hw();
    bool open_all(std::string &err);
    void close_all();

    // JTAG (CSI ioctl). Lease gives exclusive access for a sequence of writes.
    bool lease();
    void unlease();
    bool write(uint8_t addr, uint16_t v);
    // period_us > 0: driver starts write i at t0 + i * period (crystal-timed) if it can
    bool batch(const JtagReg *r, unsigned n, double period_us = 0, PaceStats *ps = nullptr);
    // fpga-csi clocks paced batches from an RP1 PIO state machine (jtag_rio and jtag_pio on):
    // write timing then ignores CPU and DRAM load, unlike the RIO busy-wait
    static bool pio_paced();
    bool spi_tx(unsigned reg, unsigned d10);

    // MAX2850 TX path on/off; LO stays as the control page set it.
    void tx_on(unsigned mask = 0xF);  // bit k keys element k
    void tx_off();
    void tx_phases(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3);
    // Gains, caller holds the lease. TX: MAX2850 Main9, 0..63 dB, capped at tx_gain_cap().
    // RX: FPGA 0x6A total gain in dB with the AGC bit (7) cleared; the FPGA splits it over
    // LNA/VGA/digital. Both return the value written.
    int set_tx_gain(int db);
    int set_rx_gain(int db);
    static int tx_gain_cap();  // 63 once the lawful-use notice is accepted on the Controls page, else 40

    // DSI: CW at f_if snapped to an integer number of cycles per DSI frame so
    // frames can be re-queued without a phase splice.
    double start_tone(double f_if, double amp);
    void stop_tone();

    // CSI ring: copy out up to max_bytes (multiple of 8), returns bytes.
    size_t rx_read(int8_t *dst, size_t max_bytes, int timeout_ms);
    void rx_flush();
    size_t rx_span();  // bytes the driver pushes per CSI frame-end IRQ
    bool rx_full();    // ring cannot take another span: the CSI IRQ thread is parked
    float rx_fill() const { return last_fill_; }  // ring fill seen by the last rx_read, 1 = parked

    // quadrf-jtag CLI wrappers (shared with the control GUI).
    static bool apply_defaults(int rx_gain, int tx_gain, double freq_mhz, int bw);
    static bool apply_required_rx();
    static RadioStatus query_status();

private:
    int jfd_ = -1, rfd_ = -1, dfd_ = -1;
    bool paced_ok_ = true;
    const int8_t *ring_ = nullptr;
    size_t ring_len_ = 0;
    uint32_t ring_size_ = 0;
    float last_fill_ = 0;
    int8_t *dsi_map_ = nullptr;
    size_t dsi_len_ = 0, dsi_frame_ = 0;
    uint32_t dsi_count_ = 0;
    std::thread dsi_th_;
    std::atomic<bool> dsi_run_{false};
};

}  // namespace qm
