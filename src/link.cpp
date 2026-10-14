#include "link.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

namespace qm {

namespace {
constexpr int CHAT_TXT = BLK_DATA - 4;  // [id16][frag][nfrag][text]
constexpr int ACK_MAX = (BLK_DATA - 1) / 2;
std::mt19937 &rng()
{
    static std::mt19937 r(std::random_device{}());
    return r;
}
}  // namespace

double Link::now()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

Link::Link(Hw &hw) : hw_(hw), next_id_((uint16_t)rng()()) {}

Link::~Link() { stop(); }

bool Link::start(const LinkConfig &cfg, std::string &err)
{
    stop();
    // CPU3 belongs to the TX thread (see mac_loop); everything spawned from here, the
    // decoder worker and the web server included, stays on CPU0-2.
    if (!getenv("QM_NORT")) {
        cpu_set_t cs;
        CPU_ZERO(&cs);
        for (int k = 0; k < 3; k++) CPU_SET(k, &cs);
        pthread_setaffinity_np(pthread_self(), sizeof cs, &cs);
    }
    cfg_ = cfg;
    double f = hw_.start_tone(cfg.f_if, cfg.tx_amp);
    cfg_.f_if = f;
    rx_ = std::make_unique<Receiver>(f);
    hw_.lease();
    hw_.tx_off();
    hw_.tx_phases(0, 0, 0, 0);
    hw_.unlease();
    hw_.rx_flush();
    run_ = true;
    last_tx_end_ = now();
    rx_th_ = std::thread(&Link::rx_loop, this);
    mac_th_ = std::thread(&Link::mac_loop, this);
    return true;
}

void Link::stop()
{
    if (!run_) return;
    run_ = false;
    if (mac_th_.joinable()) mac_th_.join();
    if (rx_th_.joinable()) rx_th_.join();
    hw_.lease();
    hw_.tx_off();
    hw_.tx_phases(0, 0, 0, 0);
    hw_.unlease();
    hw_.stop_tone();
}

void Link::set_config(const LinkConfig &c)
{
    std::lock_guard<std::mutex> l(cm_);
    double f = cfg_.f_if;
    cfg_ = c;
    cfg_.f_if = f;  // the tone and NCO are fixed for the session
}

LinkConfig Link::config()
{
    std::lock_guard<std::mutex> l(cm_);
    return cfg_;
}

// ------------------------------------------------------------------ RX

void Link::rx_loop()
{
    // Above the decoders, below the TX thread: if the DDC falls behind the ring parks and
    // the stream gets holes (92-152 per 20 s with three decoders at normal priority).
    if (!getenv("QM_NORT")) {
        sched_param sp{};
        sp.sched_priority = 50;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }
    std::vector<int8_t> buf(1 << 20);
    bool dirty = false, first = true;
    while (run_) {
        // Own TX saturates the receiver through the T/R switch: drop it plus 1 ms of switch
        // settling without copying (a 300 MB/s ring memcpy makes JTAG writes stall), then
        // restart the detector on clean samples.
        // No CSI ioctls at all while keyed: they share the driver with the JTAG batch and
        // stretch writes to 2-3 T. The ring just overruns and is flushed afterwards.
        if (tx_active_ || now() < last_tx_end_ + 1e-3) {
            dirty = true;
            usleep(1000);
            continue;
        }
        if (dirty) {
            // The CSI IRQ thread parks on the full ring while the DMA keeps cycling its
            // buffers, so the first spans after a flush are stale or overwritten mid-span.
            hw_.rx_flush();
            size_t skip = 2 * hw_.rx_span(), got = 0;
            while (got < skip && run_) {
                size_t n = hw_.rx_read(buf.data(), std::min(buf.size(), skip - got), 20);
                if (!n) break;
                got += n;
            }
            static const bool dbg = getenv("QM_TXDBG") != nullptr;
            if (dbg) fprintf(stderr, "rx resume: skipped %zu bytes (span %zu)\n", got, hw_.rx_span());
        }
        size_t n = hw_.rx_read(buf.data(), buf.size(), 5);
        if (!n) continue;
        // a parked ring outside our own TX means the push/DDC thread fell behind and the
        // stream has a hole the detector cannot see
        if (!dirty && !first && hw_.rx_fill() >= 1.f) rx_overruns_++;
        if (dirty || first) {
            if (dirty) rx_->reset();
            dirty = first = false;
            epoch_t0_[rx_->epoch() & 15] = now() - (double)(n / 8) / RX_FS;
        }
        rx_->push(buf.data(), n / 8, viz_);
        last_push_t_ = now();
    }
}

void Link::handle_frame(FrameResult &R)
{
    const double t = now();
    std::lock_guard<std::mutex> l(sm_);
    st_.last_rx_age_s = 0;
    if (!R.hdr_ok) {
        // preamble gate rejects (carrier on/off edges, noise) are not lost frames
        if (R.pre_resid_db > rx_->params().pre_gate_db) st_.false_starts++;
        else st_.frames_bad++;
        return;
    }
    st_.frames_ok++;
    last_heard_t_ = t;
    std::copy(R.snr_db, R.snr_db + 4, snr_hist_[snr_hist_n_++ % 8].begin());
    st_.rx_level = R.adc_level;
    std::copy(R.adc_lane, R.adc_lane + 4, st_.rx_lane);
    // Decoding runs behind the air by up to a frame, so a peer frame heard just before our
    // own burst can pop out after it. Turn-taking uses on-air time and ignores those.
    const double t_air = epoch_t0_[R.epoch & 15] + R.t_start + R.T_us * NWRITES * 1e-6;
    if (t_air > last_tx_end_) {
        last_rx_t_ = t_air;
        if (R.hdr.last) got_last_ = true;
    }
    const int nb = payload_blocks(R.hdr);
    st_.frame_ms = R.T_us * NWRITES * 1e-3;
    st_.rx_phy_bps = payload_bits(R.hdr) / ((R.T_us * (NWRITES + LEAD_WRITES) + 400) * 1e-6);
    int good = 0;
    R.blk_n = nb;
    for (int b = 0; b < nb && (size_t)(b + 1) * BLK_BYTES <= R.payload.size(); b++) {
        const uint8_t *blk = R.payload.data() + b * BLK_BYTES;
        uint8_t type;
        int len;
        const uint8_t *d;
        st_.blocks_total++;
        uint8_t ref[BLK_BYTES];
        fill_prbs(R.hdr.seq, b, ref);
        if (!parse_block(blk, type, len, d)) {
            int e = 0;
            for (int i = 0; i < BLK_BYTES; i++) e += __builtin_popcount(blk[i] ^ ref[i]);
            if (e < BLK_BYTES * 2) { st_.fill_bits += BLK_BYTES * 8; st_.fill_errs += e; }
            continue;
        }
        st_.blocks_ok++;
        R.blk_ok++;
        good += BLK_DATA * 8;
        last_good_t_ = t;
        if (type == BT_FILL) {
            st_.fill_bits += BLK_BYTES * 8;
            continue;
        }
        if (type == BT_FEEDBACK && len >= 18) {
            st_.peer_valid = true;
            for (int k = 0; k < 4; k++) st_.peer_snr[k] = (int8_t)d[k] * 0.5f;
            st_.peer_model_snr = (int8_t)d[4] * 0.5f;
            st_.peer_level = d[6];
            st_.peer_frames_ok = d[7] | d[8] << 8;
            const int good = d[9] | d[10] << 8;
            st_.peer_blocks_ok = d[11] | d[12] << 8;
            st_.peer_blocks_total = d[13] | d[14] << 8;
            st_.peer_tx_gain = d[15];
            st_.peer_rx_gain = d[16];
            st_.peer_age_s = d[17] == 255 ? 1e9f : d[17] * 0.01f;
            if (len >= 20) {
                const int p = d[18] | d[19] << 8;
                if (p > 0) rx_->set_peer_next_period(p * 0.01f);
            }
            // every frame of a burst repeats the same feedback; count it once per burst
            if (t_air > last_tx_end_ && fb_t_ < last_tx_end_) {
                fb_count_++;
                fb_t_ = t;
            }
            // Rate control takes the newest copy: the one in the peer's last frame was built
            // after it had decoded all of our burst, the ones in frames 0 and 1 may not be.
            if (t_air > last_tx_end_) snap_ = {good == 0xFFFF ? -1 : good, t};
        } else if (type == BT_ACK && len >= 1) {
            std::lock_guard<std::mutex> q(qm_);
            for (int i = 0; i < d[0] && 1 + 2 * i + 1 < len; i++) {
                uint16_t id = d[1 + 2 * i] | d[2 + 2 * i] << 8;
                for (auto &m : log_)
                    if (m.mine && m.id == id) m.acked = true;
            }
        } else if (type == BT_ID && len >= 1) {
            std::string cs;
            for (int i = 0; i < len; i++)
                if (d[i] > 0x20 && d[i] < 0x7F) cs += (char)d[i];
            st_.peer_call = cs;
        } else if (type == BT_CHAT && len >= 4) {
            uint16_t id = d[0] | d[1] << 8;
            int frag = d[2], nfrag = d[3];
            if (nfrag == 0 || frag >= nfrag) continue;
            std::lock_guard<std::mutex> q(qm_);
            if (std::find(ack_out_.begin(), ack_out_.end(), id) == ack_out_.end()) ack_out_.push_back(id);
            auto &v = rx_frag_[id];
            if (v.size() == 1 && v[0] == "\x01") continue;  // already delivered, just re-ACK
            v.resize(nfrag);
            v[frag] = std::string((const char *)d + 4, len - 4);
            if (v[frag].empty()) v[frag] = std::string(1, '\0');
            if (std::all_of(v.begin(), v.end(), [](const std::string &s) { return !s.empty(); })) {
                ChatMsg m;
                m.id = id;
                for (auto &s : v)
                    if (s != std::string(1, '\0')) m.text += s;
                m.t = t;
                log_.push_back(m);
                v.assign(1, "\x01");
                if (on_chat) on_chat(m);
            }
        }
    }
    // good blocks per peer burst: everything heard since our last burst ended
    if (t_air > last_tx_end_) {
        if (rb_epoch_ != last_tx_end_) { rb_epoch_ = last_tx_end_; rb_good_ = 0; }
        rb_good_ += R.blk_ok;
    }
    good_bits_.emplace_back(t, good);
    while (!good_bits_.empty() && good_bits_.front().first < t - 2.0) good_bits_.pop_front();
    long sum = 0;
    for (auto &g : good_bits_) sum += g.second;
    st_.rx_goodput_bps = sum / 2.0;
}

// ------------------------------------------------------------------ TX / MAC

uint16_t Link::send_chat(const std::string &text)
{
    std::lock_guard<std::mutex> q(qm_);
    ChatMsg m;
    m.id = next_id_++;
    m.mine = true;
    m.text = text.substr(0, CHAT_TXT * 64);
    m.t = now();
    log_.push_back(m);
    return m.id;
}

void Link::set_callsign(const std::string &cs)
{
    std::lock_guard<std::mutex> l(sm_);
    st_.my_call = cs.substr(0, BLK_DATA);
}

std::vector<ChatMsg> Link::chat_log()
{
    std::lock_guard<std::mutex> q(qm_);
    return log_;
}

LinkStats Link::stats()
{
    std::lock_guard<std::mutex> l(sm_);
    LinkStats s = st_;
    s.last_rx_age_s = last_rx_t_ > 0 ? now() - last_rx_t_ : 1e9;
    s.tx_active = tx_active_;
    double t = now();
    long sum = 0;
    for (auto &g : good_bits_)
        if (g.first >= t - 2.0) sum += g.second;
    s.rx_goodput_bps = sum / 2.0;
    return s;
}

bool Link::last_frame(FrameResult &out)
{
    std::lock_guard<std::mutex> l(sm_);
    if (!last_fresh_) return false;
    out = last_;
    last_fresh_ = false;
    return true;
}

void Link::build_payload(const Header &h, int f, int nf, std::vector<uint8_t> &out)
{
    const int nb = payload_blocks(h);
    out.assign(payload_bits(h) / 8, 0);
    int b = 0;
    auto put = [&](uint8_t type, const uint8_t *d, int len) {
        make_block(type, d, len, out.data() + b * BLK_BYTES);
        b++;
    };
    // Call sign once per burst (station ID for amateur use), feedback in the first two and
    // the last frame so one lost frame does not cost the peer its power-control update. One
    // element on BPSK carries only 2 blocks a frame, so these cannot go in every frame.
    if (f == 0 && b < nb) {
        std::lock_guard<std::mutex> l(sm_);
        put(BT_ID, (const uint8_t *)st_.my_call.data(), (int)st_.my_call.size());
    }
    if ((f <= 1 || f == nf - 1) && b < nb) {
        std::lock_guard<std::mutex> l(sm_);
        // [0..3] el SNR, [4] model SNR (int8, 0.5 dB), [6] ADC level, [7..8] frames ok,
        // [9..10] good blocks from the peer's last burst (0xFFFF: heard none), [11..14] blocks,
        // [15] our TX gain, [16] our RX gain, [17] age of our newest decoded peer frame (10 ms),
        // [18..19] write period of our next burst (10 ns), so the peer seeds preamble sync with it
        uint8_t d[BLK_DATA] = {};
        // Per-element upper quartile over the last 8 decoded frames: a frame whose tracker
        // slips reads 3-10 dB on every element at once (25-35% of QPSK frames on the q2/q3
        // channel), which is a frame-level loss and not the element's SNR.
        const int nh = std::min(snr_hist_n_, 8);
        for (int k = 0; k < 4; k++) {
            float v[8];
            for (int i = 0; i < nh; i++) v[i] = snr_hist_[i][k];
            std::nth_element(v, v + nh * 3 / 4, v + nh);
            const float m = nh ? v[nh * 3 / 4] : last_.snr_db[k];
            d[k] = (uint8_t)(int8_t)std::clamp(std::lround(m * 2), -127L, 127L);
        }
        d[4] = (uint8_t)(int8_t)std::clamp(std::lround(last_.model_snr_db * 2), -127L, 127L);
        d[6] = (uint8_t)std::clamp(std::lround(last_.adc_level), 0L, 255L);
        auto u16 = [&](int i, int v) { d[i] = v & 0xFF; d[i + 1] = (v >> 8) & 0xFF; };
        u16(7, st_.frames_ok);
        u16(9, rb_epoch_ == last_tx_end_ ? std::min(rb_good_, 0xFFFE) : 0xFFFF);
        u16(11, st_.blocks_ok);
        u16(13, st_.blocks_total);
        d[15] = (uint8_t)std::max(0, st_.tx_gain);
        d[16] = (uint8_t)std::max(0, st_.rx_gain);
        d[17] = (uint8_t)std::clamp(std::lround((now() - last_heard_t_) * 100), 0L, 255L);
        u16(18, (int)std::clamp(std::lround(next_period_ * 100), 0L, 65535L));
        put(BT_FEEDBACK, d, 20);
    }
    {
        std::lock_guard<std::mutex> q(qm_);
        while (!ack_out_.empty() && b < nb) {
            uint8_t d[BLK_DATA] = {};
            int n = std::min<int>(ACK_MAX, ack_out_.size());
            d[0] = (uint8_t)n;
            for (int i = 0; i < n; i++) { d[1 + 2 * i] = ack_out_[i] & 0xFF; d[2 + 2 * i] = ack_out_[i] >> 8; }
            ack_out_.erase(ack_out_.begin(), ack_out_.begin() + n);
            put(BT_ACK, d, 1 + 2 * n);
        }
        for (auto &m : log_) {
            if (!m.mine || m.acked) continue;
            int nf = std::max<int>(1, (m.text.size() + CHAT_TXT - 1) / CHAT_TXT);
            if (b + nf > nb) break;
            for (int f = 0; f < nf; f++) {
                uint8_t d[BLK_DATA] = {};
                d[0] = m.id & 0xFF;
                d[1] = m.id >> 8;
                d[2] = (uint8_t)f;
                d[3] = (uint8_t)nf;
                std::string part = m.text.substr(f * CHAT_TXT, CHAT_TXT);
                memcpy(d + 4, part.data(), part.size());
                put(BT_CHAT, d, 4 + (int)part.size());
            }
            m.tries++;
        }
    }
    for (; b < nb; b++) fill_prbs(h.seq, b, out.data() + b * BLK_BYTES);
}

void Link::tx_burst()
{
    LinkConfig c = config();
    Header h;
    h.mode = c.mode;
    // off elements are keyed down and marked off in the header, which is all the peer needs
    for (int k = 0; k < 4; k++) h.mods[k] = c.tx_elems >> k & 1 ? c.mod : (uint8_t)EM_OFF;

    // This burst runs at the period announced in the last one; rate_control() then picks the
    // next period, which this burst's feedback blocks announce.
    if (next_period_ <= 0) next_period_ = c.rate_us;
    last_period_ = tx_period_;
    last_offered_ = cur_offered_;
    cur_offered_ = 0;
    tx_period_ = next_period_;
    rate_control();

    std::vector<std::array<uint8_t, 4>> chips;
    std::vector<uint8_t> pay;
    std::vector<JtagReg> regs(LEAD_WRITES + NWRITES);
    tx_active_ = true;
    // Key up only once the ring is full and the IRQ thread has parked: its 300 MB/s ring
    // memcpy stretches JTAG writes to 2-3 T. The RX thread can sit in a 20 ms poll before
    // it notices tx_active_, and the 8 MB ring then needs ~27 ms to fill.
    // Parked decoders keep their load off RIO-timed writes. PIO-clocked writes do not care,
    // and decoding on lets the peer's last frames count in this burst's feedback.
    const bool hold = !(tx_period_ > 0 && Hw::pio_paced());
    if (hold) rx_->hold(true, 25);
    int waited = 0;
    for (; waited < c.quiet_ms * 3 && !hw_.rx_full(); waited++) usleep(1000);
    static const bool txdbg = getenv("QM_TXDBG") != nullptr;
    if (txdbg) fprintf(stderr, "%.3f key up after %d ms, ring %s\n", now(), waited, hw_.rx_full() ? "full" : "NOT full");
    hw_.lease();
    // tail state (all phases 0 plus the pair offset), which the lead-in writes repeat
    hw_.tx_phases(0, c.pair_offset, 0, c.pair_offset);
    hw_.tx_on(c.tx_elems);
    usleep(c.lead_us);
    for (int k = 0; k < LEAD_WRITES / 2; k++) {
        regs[2 * k] = {0x2C, c.pair_offset};
        regs[2 * k + 1] = {0x2D, c.pair_offset};
    }
    for (int f = 0; f < c.burst && run_; f++) {
        // the peer's last frames finish decoding while we send; count them before the
        // feedback in this frame is built
        if (!hold) drain_frames();
        h.seq = seq_++ & 15;
        h.last = f == c.burst - 1;
        build_payload(h, f, c.burst, pay);
        build_frame(h, pay, chips);
        // optional static offset on el1/el3; constant per element, so the preamble LS absorbs it into H
        for (int k = 0; k < FRAME_CHIPS; k++) {
            const uint8_t e1 = (uint8_t)(chips[k][1] + c.pair_offset);
            const uint8_t e3 = (uint8_t)(chips[k][3] + c.pair_offset);
            regs[LEAD_WRITES + 2 * k] = {0x2C, (uint16_t)(chips[k][0] << 8 | e1)};
            regs[LEAD_WRITES + 2 * k + 1] = {0x2D, (uint16_t)(chips[k][2] << 8 | e3)};
        }
        const double tb = now();
        PaceStats ps;
        hw_.batch(regs.data(), (unsigned)regs.size(), tx_period_, &ps);
        const double us_reg = (now() - tb) * 1e6 / regs.size();
        if (txdbg) fprintf(stderr, "%.3f tx frame %d seq %d: %.2f us/reg%s slips %u late %.1f us\n", now(), f, h.seq,
                           us_reg, ps.paced ? " paced" : "", ps.slips, ps.max_late_us);
        {
            std::lock_guard<std::mutex> l(sm_);
            st_.tx_us_reg = (float)us_reg;
            st_.tx_paced = ps.paced;
            st_.tx_slips += ps.slips;
            st_.tx_late_us = ps.max_late_us;
            st_.frames_sent++;
            cur_offered_ += payload_blocks(h);
            st_.tx_period_us = (float)tx_period_;
            st_.tx_next_us = (float)next_period_;
            st_.tx_mods = h.mods;
            st_.tx_mode = h.mode;
            // write period (19.2 us when unpaced) plus the inter-frame quiet (gap + ~120 us ioctl)
            const double wp = ps.paced ? tx_period_ : 19.2;
            st_.tx_phy_bps = payload_bits(h) / (((LEAD_WRITES + NWRITES) * wp + c.gap_us + 120) * 1e-6);
        }
        usleep(c.gap_us);
    }
    hw_.tx_off();
    hw_.unlease();
    if (hold) rx_->hold(false);
    last_tx_end_ = now();
    // a peer 'last' frame handled during our burst ended the turn before this burst
    got_last_ = false;
    tx_active_ = false;
    std::lock_guard<std::mutex> l(sm_);
    st_.bursts++;
}

// TX power control on the ADC level the peer reports, plus a slow RX gain trim.
//
// Tracking: one correction per round trip (our burst -> the peer's feedback on it), 0.7 of the
// dB error, at most 4 dB, deadband 1.5 dB. Above level_max the receiver is compressing and
// may still decode, so back off 6 dB at once.
// No usable feedback for 3 s while the peer's headers still decode: hold the gain.
// No feedback and nothing heard for 3 s: sawtooth search, +5 dB per second up to the cap, then wrap to
// 0 dB. A unit that hears nothing never parks at full power, and a peer that is being
// overdriven into failure gets the low end of the sweep every cycle.
// RX gain moves only when the peer's TX is pinned at a limit: down 6 dB if our ADC level is
// over level_max while the peer sends at 0 dB, up 6 dB if it is 12 dB under target while the
// peer is at its cap.
void Link::radio_control()
{
    LinkConfig c = config();
    const double t = now();
    const int cap = std::min(c.tx_gain_max, Hw::tx_gain_cap());
    if (c.tx_gain != cfg_tx_seen_) { cfg_tx_seen_ = c.tx_gain; atpc_g_ = (float)c.tx_gain; good_t_ = t; }
    if (c.rx_gain != cfg_rx_seen_) { cfg_rx_seen_ = c.rx_gain; rx_set_ = c.rx_gain; }

    float lvl, my_lvl, age;
    int peer_tx, bursts;
    double fb_t, heard;
    {
        std::lock_guard<std::mutex> l(sm_);
        lvl = st_.peer_level;
        age = st_.peer_age_s;
        peer_tx = st_.peer_tx_gain;
        my_lvl = st_.rx_level;
        bursts = st_.bursts;
        fb_t = fb_t_;
        heard = last_heard_t_;
    }

    const char *state;
    if (!c.atpc) {
        atpc_g_ = (float)std::min(c.tx_gain, cap);
        state = "MANUAL";
    } else {
        // feedback on a burst that went out entirely at the gain now applied
        const double te = last_tx_end_;
        const bool fresh = fb_t > te && te > gain_t_ && bursts != bursts_at_fb_ && age < 0.5f && lvl > 0;
        if (fresh) {
            bursts_at_fb_ = bursts;
            good_t_ = t;
            if (lvl >= c.level_max) atpc_g_ -= 6;
            else {
                const float e = 20 * std::log10(c.level_target / lvl);
                if (std::fabs(e) > 1.5f) atpc_g_ += std::clamp(0.7f * e, -4.0f, 4.0f);
            }
            state = "TRACK";
        } else if (t - good_t_ > 3.0 && t - heard < 1.0) {
            // We decode the peer's headers but not its feedback block: the link is there and
            // the payload is failing. More power only pushes both receivers into compression
            // (QPSK duplex ran both units to 25-35 dB this way), so hold.
            state = "HOLD";
        } else if (t - good_t_ > 3.0) {
            if (t - search_t_ > 1.0) {
                search_t_ = t;
                atpc_g_ += 5;
                if (atpc_g_ > cap + 0.5f) atpc_g_ = 0;
            }
            state = "SEARCH";
        } else {
            state = "TRACK";
        }
        atpc_g_ = std::clamp(atpc_g_, 0.0f, (float)cap);
    }

    if (t - clip_t_ >= 1.0) { clip_frac_ = rx_->take_clip_frac(); clip_t_ = t; }
    if (c.auto_rx && t - rx_adj_t_ > 2.0 && t - heard < 1.0 && peer_tx >= 0 && my_lvl > 0) {
        if (my_lvl >= c.level_max && peer_tx <= 1 && rx_set_ > 0) { rx_set_ = std::max(0, rx_set_ - 6); rx_adj_t_ = t; }
        else if (my_lvl < c.level_target / 4 && peer_tx >= 38 && rx_set_ < 50) { rx_set_ = std::min(50, rx_set_ + 6); rx_adj_t_ = t; }
    }
    // Clipped into failure: headers may still decode but no block passes CRC, so our feedback
    // never reaches the peer (it holds or searches upward) and its feedback never reaches us.
    // Neither the rule above nor either side's power control can get out of that. RX gain 34
    // at 6 ft does it even with the peer at 0 dB.
    double good_rx;
    {
        std::lock_guard<std::mutex> l(sm_);
        good_rx = last_good_t_;
    }
    if (c.auto_rx && t - rx_adj_t_ > 2.0 && t - good_rx > 2.0 && clip_frac_ > 0.02f && rx_set_ > 0) {
        rx_set_ = std::max(0, rx_set_ - 6);
        rx_adj_t_ = t;
    }
    if (!c.auto_rx) rx_set_ = c.rx_gain;

    const int tg = (int)std::lround(atpc_g_);
    if (tg != tx_applied_ || rx_set_ != rx_applied_) {
        hw_.lease();
        if (tg != tx_applied_) { tx_applied_ = hw_.set_tx_gain(tg); gain_t_ = t; }
        if (rx_set_ != rx_applied_) rx_applied_ = hw_.set_rx_gain(rx_set_);
        hw_.unlease();
    }
    std::lock_guard<std::mutex> l(sm_);
    st_.tx_gain = tx_applied_;
    st_.rx_gain = rx_applied_;
    st_.rx_clip = clip_frac_;
    st_.atpc_state = state;
}

// Auto rate: one sample per burst, the good blocks the peer reports from our last burst over
// the blocks it offered, so whole lost frames count as lost blocks. Samples are summed per
// table rate and judged after rate_dwell of them.
void Link::rate_control()
{
    LinkConfig c = config();
    const double t = now();
    auto nearest = [&](double us) {
        int best = 0;
        for (int i = 1; i < RATE_N; i++)
            if (std::fabs(RATE_TABLE_US[i] - us) < std::fabs(RATE_TABLE_US[best] - us)) best = i;
        return best;
    };
    auto allowed = [&](int i) {
        return i >= 0 && i < RATE_N && RATE_TABLE_US[i] >= c.rate_min_us - 1e-3 && RATE_TABLE_US[i] <= c.rate_max_us + 1e-3;
    };
    auto set_idx = [&](int i) {
        if (i != rate_idx_) { win_ok_ = win_off_ = 0; win_n_ = 0; }
        rate_idx_ = i;
        next_period_ = RATE_TABLE_US[i];
    };

    RateSnap s;
    {
        std::lock_guard<std::mutex> l(sm_);
        s = snap_;
    }
    const char *state = "MANUAL";
    float ok = -1;
    if (!c.auto_rate) {
        next_period_ = c.rate_us;
        rate_idx_ = -1;
    } else {
        if (rate_idx_ < 0 || c.rate_us != cfg_rate_seen_ || !cfg_auto_seen_) {
            int i = nearest(c.rate_us);
            while (!allowed(i) && i > 0 && RATE_TABLE_US[i] < c.rate_min_us) i--;
            while (!allowed(i) && i < RATE_N - 1 && RATE_TABLE_US[i] > c.rate_max_us) i++;
            set_idx(i);
            rate_fb_t_ = t;
        }
        state = "TRACK";
        // snap_ only takes reports that arrived after our last burst ended, i.e. on that burst
        if (s.t > rate_fb_t_ && s.good >= 0 && last_offered_ > 0) {
            if (std::fabs(last_period_ - RATE_TABLE_US[rate_idx_]) < 1e-3) {
                win_ok_ += std::min(s.good, last_offered_);
                win_off_ += last_offered_;
                win_n_++;
            }
            rate_fb_t_ = s.t;
            rate_slow_t_ = t;
        }
        const int i = rate_idx_;
        // Goodput break-even with the next slower rate, assuming that rate delivers
        // everything: e/T_i < 1/T_(i-1)  <=>  e < T_i/T_(i-1), ~0.85-0.92 across the table.
        // A plain "95% delivered" test throws away ~40% of goodput at 10 us for a few lost blocks.
        const float thr = allowed(i - 1) ? std::max(c.rate_min_ok, (float)(RATE_TABLE_US[i] / RATE_TABLE_US[i - 1])) : 0.f;
        const float e2 = win_off_ ? std::min(1.0f, (float)win_ok_ / win_off_) : 1.f;
        if (win_n_ >= 2 && win_n_ < c.rate_dwell && e2 < thr - 0.1f && allowed(i - 1)) {
            // collapsed: do not wait out the dwell
            hold_s_[i] = std::clamp(hold_s_[i] * 2, 15.0, 120.0);
            hold_until_[i] = t + hold_s_[i];
            set_idx(i - 1);
            state = "DOWN";
            ok = e2;
        } else if (win_n_ >= c.rate_dwell) {
            const float e = std::min(1.0f, (float)win_ok_ / win_off_);
            win_ok_ = win_off_ = 0;
            win_n_ = 0;
            if (e < thr && allowed(i - 1)) {
                hold_s_[i] = std::clamp(hold_s_[i] * 2, 15.0, 120.0);
                hold_until_[i] = t + hold_s_[i];
                set_idx(i - 1);
                state = "DOWN";
            } else if (e >= c.rate_up_ok && allowed(i + 1) && t >= hold_until_[i + 1]) {
                hold_s_[i] = 7.5;  // this rate holds: a later failure here backs off from 15 s
                set_idx(i + 1);
                state = "UP";
            } else {
                state = t < hold_until_[i + 1 < RATE_N ? i + 1 : i] ? "HOLD" : "TRACK";
            }
            ok = e;
        } else if (t - rate_fb_t_ > 3.0 && t - rate_slow_t_ > 1.0 && allowed(i - 1)) {
            // no feedback on our bursts: the peer may not sync at this rate at all
            rate_slow_t_ = t;
            set_idx(i - 1);
            state = "NO FEEDBACK";
        }
    }
    cfg_rate_seen_ = c.rate_us;
    cfg_auto_seen_ = c.auto_rate;
    std::lock_guard<std::mutex> l(sm_);
    st_.rate_state = state;
    if (ok >= 0) st_.rate_ok = ok;
    st_.tx_next_us = (float)next_period_;
    st_.rx_peer_period_us = rx_->peer_period();
}

void Link::drain_frames()
{
    FrameResult R;
    while (rx_->pop(R)) {
        handle_frame(R);
        std::lock_guard<std::mutex> l(sm_);
        if (R.hdr_ok || !last_.hdr_ok) {
            last_ = R;
            last_fresh_ = true;
        }
        if (on_frame) on_frame(R);
    }
}

void Link::mac_loop()
{
    // fpga-csi IRQ and its copy kworkers sit on CPU0; a preempted JTAG batch
    // stretches one chip by tens of us, so keep the TX thread on CPU3 at RT prio.
    if (!getenv("QM_NORT")) {
        cpu_set_t cs;
        CPU_ZERO(&cs);
        CPU_SET(3, &cs);
        pthread_setaffinity_np(pthread_self(), sizeof cs, &cs);
        sched_param sp{};
        sp.sched_priority = 60;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    }
    std::uniform_int_distribution<int> bd;
    auto beacon = [&] {
        LinkConfig c = config();
        return std::uniform_int_distribution<int>(c.beacon_min_ms, c.beacon_max_ms)(rng()) * 1e-3;
    };
    double timeout = beacon();
    while (run_) {
        drain_frames();
        radio_control();
        LinkConfig c = config();
        const double t = now(), lr = last_rx_t_, lt = last_tx_end_, lp = last_push_t_;
        // Carrier sense from the step detector. Decodes trail the air by 10-30 ms and a lost
        // frame adds 20 ms more, so "no decoded frame for a while" fires inside the peer's
        // burst; the detector runs on every RX block and sees each write directly.
        const double quiet = lp > lt ? rx_->quiet_s() + (t - lp) : 0;
        bool go = false;
        if (c.tx_enable && (c.tx_elems & 0xF)) {
            if (got_last_ && t - lr > c.turn_ms * 1e-3 && quiet > c.turn_ms * 1e-3) go = true;
            // heard the peer's burst but its last frame was lost
            else if (lr > lt && quiet > 0.015) go = true;
            else if (t - std::max(lr, lt) > timeout && quiet > 0.015) go = true;
        }
        if (go) {
            got_last_ = false;
            tx_burst();
            timeout = beacon();
        } else {
            usleep(1000);
        }
    }
}

}  // namespace qm
