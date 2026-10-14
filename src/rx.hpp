// Receiver: DDC -> quiet-gap frame detector -> preamble sync / LS channel
// estimate -> joint timing + ML pair tracker -> header / payload demap.
#pragma once
#include <array>
#include <atomic>
#include <complex>
#include <cstdint>
#include <deque>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "frame.hpp"

namespace qm {

using cf = std::complex<float>;
using cd = std::complex<double>;
using Mat4 = std::array<std::array<cd, 4>, 4>;

constexpr int DECIM = 16;  // 37.38 MS/s -> 2.336 MS/s, 0.43 us per sample

// Write periods the auto rate control steps through, slow to fast. Neighbours are at most
// 1.19x apart, inside the preamble search's 0.65..1.4 T window around its seed, so a
// receiver seeded with one step still locks the next. The receiver also sweeps this
// table while it has decoded nothing for a while.
constexpr float RATE_TABLE_US[] = {18.9f, 16.f, 14.f, 12.f, 11.f, 10.f, 9.f, 8.f, 7.f, 6.f};
constexpr int RATE_N = sizeof RATE_TABLE_US / sizeof RATE_TABLE_US[0];

// Live-adjustable receiver settings. Defaults come from OTA tests at 6 ft.
struct RxParams {
    int lanes = 0xF;         // RX lanes in use, bit k = lane k; others are zeroed after the DDC
    // quiet run that marks a frame start. TX gap + ioctl + 16 lead-in writes is ~680 us; with
    // one QPSK element a quarter of chip steps are zero, and 200 us gave 6-8 false starts
    // per frame that crowded real frames out of the decode queue on the Pi
    float gap_us = 400;
    float det_k = 8;         // step detector threshold, multiples of the noise floor
    float det_rel = 0.02;    // ... and relative to signal power: a strong steady tone wanders
                             // ~-25 dBc per 3.4 us (TX synth settling, phase noise), which beats
                             // a noise-referenced threshold and hides the first preamble step
    // Preamble sync seed before anything has decoded. After that the seed is the peer's
    // last measured write period, plus the period it announced for its next burst.
    float t0_us = 19.2;
    float sigma_us = 1.0;    // timing prior width, early side
    float late_mult = 3;     // late side prior is wider: JTAG stalls only stretch writes
    float late_cap = 8;      // ... and capped, slow bursts reach 2.9 T
    float win_lo = 0.75;     // tracker search window, in write periods after last boundary
    float win_hi = 3.2;
    // Common-phase loop. Relative LO wander slews ~+-1 kHz in 5-15 ms; at 0.2/0.01 the
    // frequency integrator lagged those slews and frames with fast wander came out wide.
    // 0.1/0.03 cut failed blocks by a third in a 4-round on-air A/B between q2 and q3.
    float pll_kp = 0.1;
    float pll_ki = 0.03;
    float h_mu = 0.02;       // LMS step for channel tracking during the frame
    int ls_win = 144;        // decision-directed LS refit of H over this many slots, every 8 (0 = LMS only;
                             // 48 followed QPSK misdecisions into rotated columns)
    // weight of the timing prior against the data likelihood. QPSK OTA q2>q3 blocks: lam 1 /
    // sigma 1.5 252/576, lam 4 / sigma 1 402/576 (outside bursts the writes are regular, and
    // a weak prior lets near-cancelling pair hypotheses pull the boundary)
    float lam = 4.0;
    float la_margin = 1000;  // one-write lookahead when an alternative boundary is this close (0 = off)
    int redecide = 0;        // passes re-deciding payload pairs over both full slots after tracking
    float tim_alpha = 1.0;   // boundary track gain: expected next write = smoothed track + T (1 = last boundary)
    float cfo_alpha = 0.5;   // NCO pull-in gain per frame
    int refine = 2;          // preamble re-segmentation passes with the known states
    float pre_gate_db = -4;  // max preamble LS residual to accept a frame start
    // Channel filter after the DDC: boxcars of lpf and lpf+1 samples (nulls at FS/8 = 292 kHz
    // and FS/9 = 260 kHz for lpf = 8). TX synth phase spurs sit at +-250..350 kHz, -9 to
    // -19 dBc, i.e. 15-35 deg rms of fast phase jitter that the 16-tap boxcar passes. The
    // decoder then skips lpf samples either side of every write boundary (the step ramp).
    // Those spurs were on the old q3 RF board; neither current board shows anything above
    // -45 dBc there. At lpf = 8 the 3.4 us ramp per boundary made q2 lock 8 of 40 q3 frames;
    // lpf = 2 locks 28 of 40 (q3 locks q2's 32 of 39 either way) with a third of the stalls.
    // Raise it (live, CHANNEL FILTER) if a unit shows strong +-250..350 kHz spurs.
    int lpf = 2;             // 0 = off
    int pm_lines = 3;        // TX phase-spur lines fitted and removed per frame, 0 = off
};

struct FrameResult {
    bool hdr_ok = false;
    Header hdr;
    std::vector<uint8_t> payload;
    double t_start = 0;          // seconds on the RX sample clock
    int epoch = 0;               // Receiver::reset() count when the frame was cut; t_start restarts at 0 each epoch
    float f_res = 0, cfo_hz = 0;
    float T_us = 0, pre_resid_db = 0, model_snr_db = 0, tone_snr_db = 0;
    Mat4 H{};
    float sv[4]{}, cond = 0;
    float snr_db[4]{};           // per element, SIC/MRC constellation EVM
    float hdr_margin = 0;
    int writes = 0, stalls = 0;
    int blk_ok = 0, blk_n = 0;  // filled by the link layer after CRC16 checks
    // ADC level over the frame: 99th percentile of max(|I|,|Q|) per 16-sample block, per lane.
    // The RX chain compresses near 38-40 counts here (well below the int8 range), and model SNR
    // drops from 24 to 5 dB going from 20 to 39 counts, so this is the TX power control input.
    float adc_lane[4]{}, adc_level = 0;
    float slot_mean_us = 0, slot_std_us = 0, slot_max_us = 0;
    // stage taps (filled when viz is on)
    std::vector<cf> constel[4];
    std::vector<float> slot_us, theta;
    std::vector<cf> slot_y;     // [writes][4] slot means, derotated
    std::vector<std::array<uint8_t, 4>> slot_x;
    float pm_hz[4]{}, pm_deg[4]{};  // fitted phase spurs (pm_estimate)  // decided element codes per slot
    std::vector<float> step_e;  // detector output around the frame start
    float step_thr = 0;
    int step_b = 0;
    // frame PSD at the decimated rate, PSD_N bins fftshifted, residual CFO removed,
    // mean over the 4 channels; bins sum to mean sample power. Filled every few frames.
    std::vector<float> psd;
};
constexpr int PSD_N = 1024;

struct VizRaw {
    std::vector<int8_t> raw;    // interleaved 4ch cs8 snapshot
    std::vector<cf> bb;         // baseband snapshot [n][4]
    float raw_rms[4]{}, raw_peak[4]{};
    double nco_hz = 0;
    bool trig = false;          // signal present; false = 1 s free-run snapshot of noise
    int lanes = 0xF;
};

class Receiver {
public:
    explicit Receiver(double f_if);
    ~Receiver();
    void set_params(const RxParams &p) { std::lock_guard<std::mutex> l(pm_); P_ = p; }
    RxParams params() { std::lock_guard<std::mutex> l(pm_); return P_; }
    void reset();
    // ninst = number of 8-byte sample instants
    void push(const int8_t *raw, size_t ninst, bool viz);
    bool pop(FrameResult &out);
    void drain();  // wait for queued frame decodes
    // Park the decoders while we transmit: decode load on the other cores stretches the
    // bit-banged JTAG writes (3 workers: TX p90 21.1 vs 19.9 us/reg, 4x the stalled frames).
    // hold(true) waits up to max_ms for jobs already running.
    void hold(bool on, int max_ms = 0);
    double nco_hz() const { return f_if_ + cfo_; }
    double cfo_hz() const { return cfo_; }
    float noise_var() const { return nv_tot_; }
    int epoch() const { return epoch_.load(); }
    // seconds of RX samples since the detector last saw a phase step; peer frames step every
    // ~19 us, so this is carrier sense that does not wait for frame decodes
    double quiet_s() const { return quiet_s_.load(); }
    // DDC blocks with a raw sample at the ADC ceiling since the last call, out of all blocks.
    // Counts whether or not anything decodes, so an overdriven receiver can see itself.
    float take_clip_frac()
    {
        const uint32_t c = clip_blk_.exchange(0), a = all_blk_.exchange(0);
        return a ? (float)c / a : 0.f;
    }
    static constexpr int CLIP_PK = 38;  // int8 RX samples top out at ~39
    // Write period the peer says its next burst uses (its feedback block), us; 0 = unknown.
    void set_peer_next_period(float us) { peer_next_us_ = us; }
    float peer_period() const { return peer_T_us_.load(); }
    bool viz_raw(VizRaw &v);
    bool offline = false;  // file/synth input: block on a full decode queue instead of dropping
    std::atomic<int> det_frames{0}, dec_ok{0}, dec_fail{0}, dec_drop{0};
    double prof_[4]{};  // seconds in DDC, viz/noise, detector, decoder

private:
    struct Job {
        std::vector<cf> z;
        int64_t a = 0, b = 0;
        double nco = 0;  // cfo_ the DDC was using when this frame was cut
        int epoch = 0;
        float nv[4]{};
        float adc[4]{};
        RxParams P;
        bool viz = false;
        std::vector<float> step_e;
        float step_thr = 0;
        int step_b = 0;
    };
    void detect(int64_t from);
    // preamble sync seeds to try, first = most likely
    void seed_periods(const RxParams &P, std::vector<float> &out) const;
    std::atomic<float> peer_T_us_{0}, peer_next_us_{0};
    std::atomic<double> last_ok_wall_{0};
    void try_decode(bool viz);
    void worker();
    void run_job(Job &j);

    std::mutex pm_;
    RxParams P_;
    double f_if_, ph_ = 0;
    std::atomic<double> cfo_{0};
    std::atomic<int> epoch_{0};
    std::atomic<double> quiet_s_{0};
    std::atomic<uint32_t> clip_blk_{0}, all_blk_{0};
    std::vector<cf> buf_;       // [n][4] baseband, absolute index base_ + i
    std::vector<uint8_t> pk_;   // [n][4] ADC peak per lane over each decimation block, same indexing
    int64_t base_ = 0, det_pos_ = 0, last_hit_ = -1;
    std::atomic<int64_t> busy_until_{0};  // set by the decoder: skip starts inside a decoded frame
    float nv_[4]{1, 1, 1, 1}, nv_tot_ = 4;
    int64_t nv_upd_ = 0;
    // channel filter state: two running sums per lane, histories [L][4]
    int lf_ = -1, li1_ = 0, li2_ = 0;
    cd ls1_[4]{}, ls2_[4]{};
    std::vector<cf> lh1_, lh2_;
    std::deque<std::pair<int64_t, int64_t>> cand_;
    std::mutex out_m_;
    std::deque<FrameResult> out_;
    std::mutex viz_m_;
    VizRaw viz_;
    bool viz_fresh_ = false;
    int64_t viz_next_ = 0;
    double viz_free_t_ = 0;  // steady-clock s: free-run snapshot deadline, survives reset()
    // frame decodes run on their own thread so DDC + detection keep up with the CSI ring
    // Three decoders: a false start costs about as much as a frame, and with one worker the
    // queue overflowed (232 frames dropped in 20 s at q2). CPU3 stays for the RT TX thread.
    std::vector<std::thread> th_;
    std::mutex jm_;
    std::condition_variable jcv_;
    std::deque<Job> jobs_;
    bool stop_ = false, hold_ = false;
    int busy_ = 0;
};

bool decode_frame(const cf *z, int N, int a, int b, const RxParams &P, const float nv[4],
                  bool viz, FrameResult &R);

void hermitian_eigs(const Mat4 &A, double ev[4]);

}  // namespace qm
