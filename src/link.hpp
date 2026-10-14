// Half-duplex TDD link over the phase-coded frame PHY.
//
// One unit holds the token and sends a burst of frames (last one flagged), then the
// other side answers. Each frame payload is a list of 24-byte blocks: ID (call sign) and
// FEEDBACK in some frames of a burst, ACKs, pending chat fragments, then PRBS fill used for BER.
#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "frame.hpp"
#include "hw.hpp"
#include "rx.hpp"

namespace qm {

struct LinkConfig {
    double f_if = 500e3;       // DSI tone IF, Hz (snapped to whole cycles per DSI frame)
    double tx_amp = 120;       // DSI tone amplitude, int8 counts
    int burst = 16;            // frames per TDD turn; each turn costs ~35 ms of ring fill, lead and turnaround
    uint8_t mode = FM_SDM;
    uint8_t mod = EM_BPSK;     // modulation on every element that carries data
    // TX elements in use, bit k = element k. Off elements are keyed down and marked off in
    // the header. Payload bits interleave chip by chip over the elements, so one dead element
    // (q3 el3 TX arrives 21 dB under its others) fails every CRC block until it is turned off.
    uint8_t tx_elems = 0xF;
    // TX gain, dB. With atpc on this is only the starting point: the loop steers the peer's
    // reported ADC level to level_target. RX gain stays where it is set unless auto_rx is on and
    // the peer's TX is pinned at a limit.
    int tx_gain = 10, tx_gain_max = 40, rx_gain = 16;
    bool atpc = true, auto_rx = true;
    // ADC counts, frame p99 of block peaks. The 8-bit RX samples top out at +-39: q3 RX gain
    // sweep, BPSK blocks 99% at peak 22, 98% at 32, 67% at 39 (pinned), 4% at RX gain 34.
    float level_target = 24;
    float level_max = 34;      // above this the peer backs off hard
    // Static code offset on el1/el3 (64 = +90 deg). Only helps when the two elements of a write
    // pair arrive near co-phased and equally strong on a rank-1 channel; measured q2->q3 pair
    // angles were 29 and 53 deg, so +90 helped one pair and hurt the other. Off by default.
    uint8_t pair_offset = 0;
    // Link rate as the TX JTAG write period: one write per period on the driver's PIO stream
    // (exact to 5 ns), or busy-waited deadlines on RIO; the stock driver ignores it and runs
    // ~19-20 us with stalls. The peer needs no matching setting: each burst announces the
    // period of the next one in its feedback block and the receiver seeds from that and from
    // the period it measures. With auto_rate this is only the starting point.
    double rate_us = 10;
    // Auto rate: step through RATE_TABLE_US on the peer's reported block delivery for our
    // bursts. Go one step faster after rate_dwell bursts at >= rate_up_ok, one step slower
    // when delivery drops under the goodput break-even with the slower rate (T_i/T_(i-1)) or
    // under rate_min_ok, whichever is higher, and back off probing a rate that failed
    // (15 s, doubling to 2 min). Bounds: rate_min_us (fastest) .. rate_max_us.
    bool auto_rate = true;
    double rate_min_us = 7, rate_max_us = 18.9;
    float rate_min_ok = 0.8f, rate_up_ok = 0.98f;
    int rate_dwell = 3;
    int lead_us = 2000;        // quiet tone after TX on: the TX synth settles over ~1 ms and the CFO estimate uses the last 280 us
    int quiet_ms = 30;         // expected ring fill time before keying up; the wait ends early once the ring is full, and gives up at 3x
    int gap_us = 250;          // extra quiet between frames (ioctl overhead adds ~120 us)
    int turn_ms = 3;           // wait after the peer's last frame before keying up
    int beacon_min_ms = 150, beacon_max_ms = 300;  // token recovery timeout
    bool tx_enable = true;
};

struct ChatMsg {
    uint16_t id = 0;
    bool mine = false;
    std::string text;
    double t = 0;              // wall time sent / received
    bool acked = false;
    int tries = 0;
};

struct LinkStats {
    // RX side
    int frames_ok = 0, frames_bad = 0, blocks_ok = 0, blocks_total = 0;
    int false_starts = 0;
    long fill_bits = 0, fill_errs = 0;
    double rx_goodput_bps = 0;     // CRC-good block data bits per second, 2 s window
    double rx_phy_bps = 0;         // raw payload bits per frame time, last frame
    double frame_ms = 0;
    std::string my_call, peer_call;
    // TX side
    int frames_sent = 0, bursts = 0;
    std::array<uint8_t, 4> tx_mods{};
    uint8_t tx_mode = 0;
    double tx_phy_bps = 0;
    float tx_us_reg = 0;       // measured JTAG batch time per register write
    bool tx_paced = false;     // driver paced the last frame's writes
    unsigned tx_slips = 0;     // paced writes that started late, cumulative
    float tx_late_us = 0;      // RIO: worst write start lateness; PIO stream: +/- on the burst's slot 0 time
    float tx_period_us = 0;    // write period of the last burst
    float tx_next_us = 0;      // ... and of the next one, as announced
    const char *rate_state = "MANUAL";
    float rate_ok = -1;        // peer's block delivery for our last rate window, -1 = none yet
    float rx_peer_period_us = 0;  // peer's measured write period (our RX seed)
    // radio
    int tx_gain = -1, rx_gain = -1;  // as written to the chips
    const char *atpc_state = "MANUAL";
    float rx_level = 0, rx_lane[4]{};  // ADC level of the newest peer frame
    float rx_clip = 0;                 // share of DDC blocks at the ADC ceiling, last second, decoded or not
    // peer's view of our signal (from its feedback)
    bool peer_valid = false;
    float peer_snr[4]{}, peer_model_snr = 0, peer_level = 0;
    int peer_tx_gain = -1, peer_rx_gain = -1;
    float peer_age_s = 1e9;          // how long ago the peer last decoded one of our frames, as it reported
    int peer_frames_ok = 0, peer_frames_bad = 0, peer_blocks_ok = 0, peer_blocks_total = 0;
    double last_rx_age_s = 1e9;
    bool tx_active = false;
};

class Link {
public:
    explicit Link(Hw &hw);
    ~Link();
    bool start(const LinkConfig &cfg, std::string &err);
    void stop();
    void set_config(const LinkConfig &c);
    LinkConfig config();
    RxParams rx_params() { return rx_->params(); }
    // detector candidates, decode failures, jobs dropped by an overloaded decoder
    void rx_counts(int &det, int &fail, int &drop, int &overrun) const
    {
        det = rx_->det_frames.load();
        fail = rx_->dec_fail.load();
        drop = rx_->dec_drop.load();
        overrun = rx_overruns_.load();
    }
    void set_rx_params(const RxParams &p) { rx_->set_params(p); }

    uint16_t send_chat(const std::string &text);
    std::vector<ChatMsg> chat_log();
    LinkStats stats();
    bool last_frame(FrameResult &out);  // most recent decoded frame (with viz taps if enabled)
    bool viz_raw(VizRaw &v) { return rx_->viz_raw(v); }
    void set_viz(bool on) { viz_ = on; }
    void set_callsign(const std::string &cs);  // sent once per burst
    std::function<void(const ChatMsg &)> on_chat;
    std::function<void(const FrameResult &)> on_frame;
    static double now();  // steady clock, s

private:
    void rx_loop();
    void mac_loop();
    void handle_frame(FrameResult &R);
    void drain_frames();  // pop decoded frames, handle them, mac thread
    void tx_burst();
    void build_payload(const Header &h, int f, int nf, std::vector<uint8_t> &out);  // frame f of nf in the burst
    void radio_control();

    Hw &hw_;
    std::unique_ptr<Receiver> rx_;
    std::mutex cm_;
    LinkConfig cfg_;
    std::atomic<bool> run_{false}, tx_active_{false}, viz_{false};
    std::thread rx_th_, mac_th_;

    std::mutex sm_;
    LinkStats st_;
    FrameResult last_;
    bool last_fresh_ = false;
    std::deque<std::pair<double, int>> good_bits_;  // (time, bits) for the goodput window
    double last_heard_t_ = -1e9;     // when we last decoded a peer frame
    double last_good_t_ = -1e9;      // ... and a CRC-good block in one
    // feedback bookkeeping for the radio loops (under sm_)
    int fb_count_ = 0, fb_used_ = 0;
    double fb_t_ = -1e9;

    // radio loops, mac thread only
    float atpc_g_ = -1;              // dB, fractional so small corrections accumulate
    int tx_applied_ = -1, rx_applied_ = -1, rx_set_ = -1;
    int cfg_tx_seen_ = -1, cfg_rx_seen_ = -1;  // a user change restarts the loop from it
    double good_t_ = -1e9, search_t_ = 0, rx_adj_t_ = 0, gain_t_ = 0, clip_t_ = 0;
    float clip_frac_ = 0;
    int bursts_at_fb_ = -1;

    // Rate control, mac thread. One sample per burst: good blocks the peer decoded from our
    // last burst (its feedback) over the blocks that burst offered, at that burst's period.
    void rate_control();
    double tx_period_ = 0, next_period_ = 0;  // this burst's write period, the announced next one
    double last_period_ = 0;                  // ... of the burst before
    int last_offered_ = 0, cur_offered_ = 0;  // blocks offered in the burst before, in this one
    int rate_idx_ = -1;                       // RATE_TABLE_US index
    double cfg_rate_seen_ = -1;
    bool cfg_auto_seen_ = false;
    long win_ok_ = 0, win_off_ = 0;
    int win_n_ = 0;
    double hold_until_[RATE_N]{}, hold_s_[RATE_N]{};
    double rate_fb_t_ = 0, rate_slow_t_ = 0;
    // peer's report on our last burst, newest copy (under sm_); good = -1: it heard none of it
    struct RateSnap { int good = -1; double t = 0; } snap_;
    // our count of good blocks in the peer's current burst (under sm_)
    double rb_epoch_ = -1;
    int rb_good_ = 0;

    std::mutex qm_;
    std::vector<ChatMsg> log_;
    std::map<uint16_t, std::vector<std::string>> rx_frag_;
    std::vector<uint16_t> ack_out_;
    uint16_t next_id_;
    uint8_t seq_ = 0;

    std::atomic<bool> got_last_{false};
    std::atomic<double> last_rx_t_{0}, last_tx_end_{0};  // on-air end of the newest peer frame; end of our last burst
    std::atomic<double> epoch_t0_[16]{};
    std::atomic<int> rx_overruns_{0};
    std::array<std::array<float, 4>, 8> snr_hist_{};  // el SNR of the last 8 decoded peer frames
    int snr_hist_n_ = 0;
    std::atomic<double> last_push_t_{0};               // steady time of sample 0 of each receiver epoch
};

}  // namespace qm
