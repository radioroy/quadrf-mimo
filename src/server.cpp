// quadrf-mimo serve: web UI over one WebSocket. Link, radio gain and receiver settings are
// live and persisted; LO is left to the QuadRF control page. While this runs it owns TX/RX
// gain (and turns the RX AGC off: a gain step inside a frame breaks the channel estimate).
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

extern "C" {
#include "mongoose.h"
}

#include "fft.hpp"
#include "hw.hpp"
#include "link.hpp"

using namespace qm;

namespace {

volatile sig_atomic_t g_run = 1;
void on_sig(int) { g_run = 0; }

// ------------------------------------------------------------------ settings

struct Setting {
    const char *key, *label, *group, *type, *unit;  // type: num | bool | sel
    double lo, hi, step;
    const char *opts;  // sel: comma separated labels, value = index
    std::function<double(const LinkConfig &, const RxParams &)> get;
    std::function<void(LinkConfig &, RxParams &, double)> set;
    double def = 0;
};

#define LNUM(k, lab, grp, unit, lo, hi, st, field)                                                              \
    Setting{k, lab, grp, "num", unit, lo, hi, st, "", [](const LinkConfig &c, const RxParams &) { return (double)c.field; }, \
            [](LinkConfig &c, RxParams &, double v) { c.field = (decltype(c.field))v; }}
#define RNUM(k, lab, grp, unit, lo, hi, st, field)                                                              \
    Setting{k, lab, grp, "num", unit, lo, hi, st, "", [](const LinkConfig &, const RxParams &p) { return (double)p.field; }, \
            [](LinkConfig &, RxParams &p, double v) { p.field = (decltype(p.field))v; }}
#define LBOOL(k, lab, grp, field)                                                                              \
    Setting{k, lab, grp, "bool", "", 0, 1, 1, "", [](const LinkConfig &c, const RxParams &) { return (double)c.field; }, \
            [](LinkConfig &c, RxParams &, double v) { c.field = v != 0; }}
std::vector<Setting> make_settings()
{
    std::vector<Setting> s = {
        LBOOL("tx", "TRANSMIT", "RADIO", tx_enable),
        Setting{"mod", "MODULATION", "RADIO", "sel", "", 0, 2, 1, "BPSK,QPSK,8PSK",
                [](const LinkConfig &c, const RxParams &) { return (double)c.mod - 1; },
                [](LinkConfig &c, RxParams &, double v) { c.mod = (uint8_t)(v + 1); }},
        Setting{"tx_elems", "TX ELEMENTS", "RADIO", "mask", "", 0, 15, 1, "",
                [](const LinkConfig &c, const RxParams &) { return (double)c.tx_elems; },
                [](LinkConfig &c, RxParams &, double v) { c.tx_elems = (uint8_t)v; }},
        Setting{"rx_lanes", "RX ELEMENTS", "RADIO", "mask", "", 0, 15, 1, "",
                [](const LinkConfig &, const RxParams &p) { return (double)p.lanes; },
                [](LinkConfig &, RxParams &p, double v) { p.lanes = (int)v; }},
        LBOOL("atpc", "AUTO TX POWER", "RADIO", atpc),
        LNUM("tx_gain", "TX GAIN (MANUAL / START)", "RADIO", "dB", 0, 63, 1, tx_gain),
        LNUM("tx_gain_max", "TX GAIN LIMIT", "RADIO", "dB", 0, 63, 1, tx_gain_max),
        LNUM("level_target", "PEER RX LEVEL TARGET", "RADIO", "counts", 4, 40, 1, level_target),
        LNUM("level_max", "ADC LEVEL LIMIT", "RADIO", "counts", 10, 60, 1, level_max),
        LNUM("rx_gain", "RX GAIN", "RADIO", "dB", 0, 63, 1, rx_gain),
        LBOOL("auto_rx", "AUTO RX GAIN AT TX LIMITS", "RADIO", auto_rx),
        LNUM("burst", "FRAMES PER TURN", "LINK", "", 1, 32, 1, burst),
        Setting{"mode", "MODE", "LINK", "sel", "", 0, 2, 1, "SDM,CDM4,CDM8",
                [](const LinkConfig &c, const RxParams &) { return (double)c.mode; },
                [](LinkConfig &c, RxParams &, double v) { c.mode = (uint8_t)v; }},
        LBOOL("auto_rate", "AUTO LINK RATE", "LINK", auto_rate),
        LNUM("rate_us", "WRITE PERIOD (MANUAL / START)", "LINK", "us", 0, 40, 0.1, rate_us),
        LNUM("rate_min_us", "AUTO RATE FASTEST", "LINK", "us", 6, 18.9, 0.1, rate_min_us),
        LNUM("rate_max_us", "AUTO RATE SLOWEST", "LINK", "us", 6, 18.9, 0.1, rate_max_us),
        LNUM("rate_min_ok", "AUTO RATE MIN DELIVERY", "LINK", "", 0.5, 1, 0.01, rate_min_ok),
        LNUM("rate_up_ok", "AUTO RATE STEP-UP DELIVERY", "LINK", "", 0.5, 1, 0.005, rate_up_ok),
        LNUM("lead_us", "LEAD TONE", "MAC", "us", 200, 6000, 100, lead_us),
        LNUM("gap_us", "FRAME GAP", "MAC", "us", 0, 2000, 10, gap_us),
        LNUM("turn_ms", "TURNAROUND", "MAC", "ms", 0, 40, 1, turn_ms),
        LNUM("quiet_ms", "RING FILL WAIT", "MAC", "ms", 0, 60, 1, quiet_ms),
        LNUM("beacon_min_ms", "BEACON MIN", "MAC", "ms", 50, 3000, 10, beacon_min_ms),
        LNUM("beacon_max_ms", "BEACON MAX", "MAC", "ms", 50, 3000, 10, beacon_max_ms),
        RNUM("lpf", "CHANNEL FILTER", "DETECT", "samples", 0, 16, 1, lpf),
        RNUM("pm_lines", "SPUR CANCEL", "DETECT", "lines", 0, 4, 1, pm_lines),
        RNUM("det_k", "STEP THRESHOLD", "DETECT", "x N0", 2, 40, 0.5, det_k),
        RNUM("det_rel", "STEP / SIGNAL", "DETECT", "", 0.001, 0.2, 0.001, det_rel),
        RNUM("rx_gap_us", "QUIET BEFORE START", "DETECT", "us", 40, 1000, 10, gap_us),
        RNUM("pre_gate_db", "PREAMBLE GATE", "DETECT", "dB", -20, 0, 0.5, pre_gate_db),
        RNUM("sigma_us", "TIMING SIGMA", "TRACK", "us", 0.3, 5, 0.1, sigma_us),
        RNUM("late_mult", "LATE SIGMA MULT", "TRACK", "", 1, 8, 0.5, late_mult),
        RNUM("late_cap", "LATE CAP", "TRACK", "", 2, 16, 0.5, late_cap),
        RNUM("win_lo", "WINDOW LOW", "TRACK", "T", 0.4, 1, 0.05, win_lo),
        RNUM("win_hi", "WINDOW HIGH", "TRACK", "T", 1.5, 5, 0.1, win_hi),
        RNUM("lam", "TIMING PRIOR WEIGHT", "TRACK", "", 0, 16, 0.1, lam),
        RNUM("la_margin", "LOOKAHEAD MARGIN", "TRACK", "", 0, 5000, 10, la_margin),
        RNUM("redecide", "TWO-SLOT REDECIDE", "TRACK", "passes", 0, 4, 1, redecide),
        RNUM("ls_win", "CHANNEL LS WINDOW", "TRACK", "slots", 0, 256, 8, ls_win),
        RNUM("pll_kp", "PLL KP", "TRACK", "", 0, 1, 0.01, pll_kp),
        RNUM("pll_ki", "PLL KI", "TRACK", "", 0, 0.1, 0.001, pll_ki),
        RNUM("h_mu", "CHANNEL LMS STEP", "TRACK", "", 0, 0.2, 0.005, h_mu),
        RNUM("cfo_alpha", "NCO PULL-IN", "TRACK", "", 0, 1, 0.05, cfo_alpha),
        RNUM("refine", "PREAMBLE PASSES", "TRACK", "", 0, 4, 1, refine),
    };
    LinkConfig c0;
    RxParams p0;
    for (auto &x : s) x.def = x.get(c0, p0);
    return s;
}

// ------------------------------------------------------------------ json helpers

struct J {
    std::string s;
    bool first = true;
    void sep() { if (!first) s += ','; first = false; }
    J &key(const char *k) { sep(); s += '"'; s += k; s += "\":"; first = true; return *this; }
    J &num(double v, int prec = 3)
    {
        sep();
        if (!std::isfinite(v)) { s += "null"; return *this; }
        char b[48];
        snprintf(b, sizeof b, "%.*f", prec, v);
        // trim trailing zeros
        char *e = b + strlen(b) - 1;
        if (strchr(b, '.')) { while (*e == '0') *e-- = 0; if (*e == '.') *e = 0; }
        s += b;
        return *this;
    }
    J &boolean(bool v) { sep(); s += v ? "true" : "false"; return *this; }
    J &str(const std::string &v)
    {
        sep();
        s += '"';
        for (unsigned char ch : v) {
            if (ch == '"' || ch == '\\') { s += '\\'; s += (char)ch; }
            else if (ch < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", ch); s += b; }
            else s += (char)ch;
        }
        s += '"';
        return *this;
    }
    J &raw(const std::string &v) { sep(); s += v; return *this; }
    J &obj() { sep(); s += '{'; first = true; return *this; }
    J &end_obj() { s += '}'; first = false; return *this; }
    J &arr() { sep(); s += '['; first = true; return *this; }
    J &end_arr() { s += ']'; first = false; return *this; }
    template <class T> J &nums(const T *v, size_t n, int prec = 2)
    {
        arr();
        for (size_t i = 0; i < n; i++) num(v[i], prec);
        return end_arr();
    }
};

// CALLSIGN as set on the QuadRF control page (NOCALL until the user enters one)
std::string read_callsign()
{
    std::string cs = "NOCALL";
    if (FILE *f = fopen("/etc/quadrf/quadrf.conf", "r")) {
        char line[256];
        while (fgets(line, sizeof line, f)) {
            const char *v = nullptr;
            if (!strncmp(line, "CALLSIGN=", 9)) v = line + 9;
            else if (!strncmp(line, "QUADRF_CALLSIGN=", 16)) v = line + 16;
            if (!v) continue;
            std::string s = v;
            while (!s.empty() && strchr("'\" \r\n", s.back())) s.pop_back();
            while (!s.empty() && strchr("'\" ", s.front())) s.erase(0, 1);
            if (!s.empty()) { cs = s; if (!strncmp(line, "CALLSIGN=", 9)) break; }
        }
        fclose(f);
    }
    return cs;
}

// power spectrum, dB, n bins fftshifted, reduced to `out` bins by max
std::vector<float> reduce_db(const std::vector<float> &p, size_t out, float ref)
{
    std::vector<float> r(out, -200);
    const size_t k = p.size() / out;
    for (size_t i = 0; i < out; i++) {
        float m = 0;
        for (size_t j = 0; j < k; j++) m = std::max(m, p[i * k + j]);
        r[i] = 10 * std::log10(m / ref + 1e-20f);
    }
    return r;
}

// ------------------------------------------------------------------ server state

struct Server {
    Hw hw;
    std::unique_ptr<Link> link;
    std::vector<Setting> settings;
    std::string state_path, web_dir, host;
    struct mg_mgr mgr;
    bool viz = false;

    size_t chat_sig = 0;
    double last_viz = 0, last_call = -1e9;
    FrameResult last_frame;  // newest CRC-good frame
    bool have_frame = false;
    FrameResult viz_frame;   // newest frame that carries viz taps
    bool have_viz_frame = false;
    std::vector<float> psd_avg;  // averaged frame PSD, normalized per frame
    double T_avg = 0;            // write period, us
    // peer-reported block counts, for the TX-direction goodput
    std::deque<std::pair<double, long>> peer_blk;
    int peer_last = -1, peer_tot_last = -1;
    long peer_acc = 0, peer_tot_acc = 0;

    void load_settings()
    {
        LinkConfig c = link->config();
        RxParams p = link->rx_params();
        FILE *f = fopen(state_path.c_str(), "r");
        if (f) {
            char k[64];
            double v;
            while (fscanf(f, "%63s %lf", k, &v) == 2)
                for (auto &s : settings)
                    if (!strcmp(s.key, k)) s.set(c, p, std::clamp(v, s.lo, s.hi));
            fclose(f);
        }
        link->set_config(c);
        link->set_rx_params(p);
    }
    void save_settings()
    {
        LinkConfig c = link->config();
        RxParams p = link->rx_params();
        std::string dir = state_path.substr(0, state_path.rfind('/'));
        mkdir(dir.c_str(), 0755);
        FILE *f = fopen(state_path.c_str(), "w");
        if (!f) return;
        // only what differs from the defaults, so new defaults in later builds take effect
        for (auto &s : settings) {
            double v = s.get(c, p);
            if (std::fabs(v - s.def) > 1e-9) fprintf(f, "%s %.6g\n", s.key, v);
        }
        fclose(f);
    }

    std::string settings_json()
    {
        LinkConfig c = link->config();
        RxParams p = link->rx_params();
        J j;
        j.obj().key("t").str("settings").key("list").arr();
        for (auto &s : settings) {
            j.obj().key("k").str(s.key).key("label").str(s.label).key("group").str(s.group).key("type").str(s.type)
                .key("unit").str(s.unit).key("lo").num(s.lo, 4).key("hi").num(s.hi, 4).key("step").num(s.step, 4)
                .key("def").num(s.def, 4).key("v").num(s.get(c, p), 4).key("opts").str(s.opts).end_obj();
        }
        j.end_arr().key("viz").boolean(viz).end_obj();
        return j.s;
    }

    void broadcast(const std::string &m)
    {
        for (struct mg_connection *c = mgr.conns; c; c = c->next)
            if (c->is_websocket) mg_ws_send(c, m.data(), m.size(), WEBSOCKET_OP_TEXT);
    }

    void on_msg(struct mg_connection *c, struct mg_str d)
    {
        char *t = mg_json_get_str(d, "$.t");
        if (!t) return;
        std::string type = t;
        free(t);
        if (type == "chat") {
            char *x = mg_json_get_str(d, "$.text");
            if (x && *x) link->send_chat(std::string(x).substr(0, 512));
            free(x);
        } else if (type == "set") {
            char *k = mg_json_get_str(d, "$.k");
            double v;
            if (k && mg_json_get_num(d, "$.v", &v)) {
                LinkConfig lc = link->config();
                RxParams p = link->rx_params();
                for (auto &s : settings)
                    if (!strcmp(s.key, k)) s.set(lc, p, std::clamp(v, s.lo, s.hi));
                if (lc.beacon_max_ms < lc.beacon_min_ms) lc.beacon_max_ms = lc.beacon_min_ms;
                link->set_config(lc);
                link->set_rx_params(p);
                save_settings();
                broadcast(settings_json());
            }
            free(k);
        } else if (type == "defaults") {
            LinkConfig lc = link->config();
            RxParams p = link->rx_params();
            for (auto &s : settings) s.set(lc, p, s.def);
            link->set_config(lc);
            link->set_rx_params(p);
            save_settings();
            broadcast(settings_json());
        } else if (type == "viz") {
            bool on = false;
            mg_json_get_bool(d, "$.on", &on);
            viz = on;
            link->set_viz(on);
            broadcast(settings_json());
        } else if (type == "hello") {
            std::string m = settings_json();
            mg_ws_send(c, m.data(), m.size(), WEBSOCKET_OP_TEXT);
            chat_sig = (size_t)-1;
        }
    }

    void tick()
    {
        const double t = Link::now();
        if (t - last_call > 5.0) {
            last_call = t;
            link->set_callsign(read_callsign());
        }
        FrameResult R;
        while (link->last_frame(R)) {
            if (R.hdr_ok) {
                if (!R.psd.empty()) {
                    double tot = 0;
                    for (float v : R.psd) tot += v;
                    if (psd_avg.size() != R.psd.size()) psd_avg.assign(R.psd.size(), 0);
                    for (size_t i = 0; i < R.psd.size(); i++)
                        psd_avg[i] = 0.8f * psd_avg[i] + 0.2f * (float)(R.psd[i] / tot);
                }
                // Median write period from the decoder. 15-30 µs was the unpaced ~19 µs
                // cluster; paced writes are 7-19 µs and were rejected, leaving the
                // 19.3 µs fallback so 1/(2T) sat at 25.9 kHz.
                if (R.T_us > 4 && R.T_us < 40) T_avg = T_avg > 0 ? 0.9 * T_avg + 0.1 * R.T_us : R.T_us;
                if (!R.constel[0].empty()) { viz_frame = R; have_viz_frame = true; }
                R.constel[0].clear();
                last_frame = std::move(R);
                have_frame = true;
            }
        }
        LinkStats s = link->stats();
        LinkConfig cfg = link->config();

        // peer counters are 16 bit and cumulative
        if (s.peer_valid) {
            if (peer_last >= 0) peer_acc += (uint16_t)(s.peer_blocks_ok - peer_last);
            else peer_acc = s.peer_blocks_ok;
            if (peer_tot_last >= 0) peer_tot_acc += (uint16_t)(s.peer_blocks_total - peer_tot_last);
            else peer_tot_acc = s.peer_blocks_total;
            peer_last = s.peer_blocks_ok;
            peer_tot_last = s.peer_blocks_total;
        }
        peer_blk.emplace_back(t, peer_acc);
        while (peer_blk.size() > 2 && peer_blk.front().first < t - 3.0) peer_blk.pop_front();
        double gp_tx = 0;
        if (peer_blk.size() > 1 && peer_blk.back().first > peer_blk.front().first)
            gp_tx = (peer_blk.back().second - peer_blk.front().second) * BLK_DATA * 8.0 /
                    (peer_blk.back().first - peer_blk.front().first);
        // Bandwidth: each element holds its phase for two JTAG writes, so a rectangular
        // symbol of 2T and a sinc^2 spectrum with first nulls at +-1/(2T).
        const FrameResult &F = last_frame;
        const double T = T_avg > 0 ? T_avg : 19.3;
        const double bw_sym = 1e6 / (2 * T);
        double bw90 = 0, bw99 = 0, lobe = 0;
        const double fs = RX_FS / DECIM;
        if (!psd_avg.empty()) {
            const int n = (int)psd_avg.size();
            // floor from the outer bins (> 700 kHz off), then occupied bandwidth within +-1 MHz
            std::vector<float> outer;
            for (int i = 0; i < n; i++) {
                double f = (i - n / 2) * fs / n;
                if (std::fabs(f) > 700e3 && std::fabs(f) < 1.05e6) outer.push_back(psd_avg[i]);
            }
            std::nth_element(outer.begin(), outer.begin() + outer.size() / 2, outer.end());
            const float fl = outer.empty() ? 0 : outer[outer.size() / 2];
            std::vector<double> cum;
            std::vector<double> fr;
            double acc = 0, in_lobe = 0;
            for (int i = 0; i < n; i++) {
                double f = (i - n / 2) * fs / n;
                if (std::fabs(f) > 1e6) continue;
                double v = std::max(0.0f, psd_avg[i] - fl);
                acc += v;
                if (std::fabs(f) <= bw_sym) in_lobe += v;
                cum.push_back(acc);
                fr.push_back(f);
            }
            auto occ = [&](double p) {
                double lo = (1 - p) / 2 * acc, hi = (1 + p) / 2 * acc;
                size_t a = std::lower_bound(cum.begin(), cum.end(), lo) - cum.begin();
                size_t b = std::lower_bound(cum.begin(), cum.end(), hi) - cum.begin();
                if (b >= fr.size()) b = fr.size() - 1;
                return fr[b] - fr[a] + fs / n;
            };
            if (acc > 0) { bw90 = occ(0.90); bw99 = occ(0.99); lobe = in_lobe / acc; }
        }

        J j;
        j.obj().key("t").str("st").key("host").str(host).key("now").num(t, 3);
        j.key("up").boolean(s.last_rx_age_s < 2.0).key("rx_age").num(std::min(s.last_rx_age_s, 1e4), 2);
        j.key("txa").boolean(s.tx_active).key("tx_en").boolean(cfg.tx_enable);
        j.key("fok").num(s.frames_ok, 0).key("fbad").num(s.frames_bad, 0).key("ffalse").num(s.false_starts, 0);
        j.key("bok").num(s.blocks_ok, 0).key("btot").num(s.blocks_total, 0);
        j.key("ber").num(s.fill_bits ? (double)s.fill_errs / s.fill_bits : -1, 7).key("ber_bits").num((double)s.fill_bits, 0);
        j.key("gp_rx").num(s.rx_goodput_bps, 0).key("gp_tx").num(gp_tx, 0);
        j.key("phy_rx").num(s.rx_phy_bps, 0).key("phy_tx").num(s.tx_phy_bps, 0);
        j.key("frame_ms").num(s.frame_ms, 2).key("us_reg").num(s.tx_us_reg, 2);
        j.key("tx_paced").boolean(s.tx_paced).key("tx_slips").num(s.tx_slips, 0).key("tx_late_us").num(s.tx_late_us, 1);
        j.key("tx_period_us").num(s.tx_period_us, 2).key("tx_next_us").num(s.tx_next_us, 2);
        j.key("rate_state").str(s.rate_state).key("rate_ok").num(s.rate_ok, 4);
        j.key("rx_peer_period_us").num(s.rx_peer_period_us, 2);
        j.key("fsent").num(s.frames_sent, 0).key("bursts").num(s.bursts, 0);
        j.key("tx_mods").nums(s.tx_mods.data(), 4, 0).key("tx_mode").num(s.tx_mode, 0);
        j.key("txg").num(s.tx_gain, 0).key("rxg").num(s.rx_gain, 0).key("atpc").str(s.atpc_state);
        j.key("txg_cap").num(std::min(cfg.tx_gain_max, Hw::tx_gain_cap()), 0);
        j.key("lvl").num(s.rx_level, 0).key("lvl_lane").nums(s.rx_lane, 4, 0).key("clip").num(s.rx_clip, 3);
        j.key("lvl_target").num(cfg.level_target, 0).key("lvl_max").num(cfg.level_max, 0);
        j.key("peer_lvl").num(s.peer_level, 0).key("peer_txg").num(s.peer_tx_gain, 0).key("peer_rxg").num(s.peer_rx_gain, 0);
        j.key("mod").num(cfg.mod, 0).key("tx_elems").num(cfg.tx_elems, 0).key("rx_lanes").num(link->rx_params().lanes, 0);
        j.key("call").str(s.my_call).key("peer_call").str(s.peer_call);
        {
            std::array<int, 4> rc{};
            link->rx_counts(rc[0], rc[1], rc[2], rc[3]);
            j.key("rxc").nums(rc.data(), 4, 0);
        }
        j.key("peer_valid").boolean(s.peer_valid).key("peer_snr").nums(s.peer_snr, 4, 1);
        j.key("peer_model").num(s.peer_model_snr, 1);
        j.key("peer_bok").num((double)peer_acc, 0).key("peer_btot").num((double)peer_tot_acc, 0);
        j.key("peer_fok").num(s.peer_frames_ok, 0);
        j.key("bw_sym").num(bw_sym, 0).key("bw90").num(bw90, 0).key("bw99").num(bw99, 0).key("lobe").num(lobe, 3);
        if (have_frame) {
            j.key("f").obj();
            j.key("seq").num(F.hdr.seq, 0).key("mods").nums(F.hdr.mods.data(), 4, 0).key("mode").num(F.hdr.mode, 0);
            j.key("snr").nums(F.snr_db, 4, 1).key("model").num(F.model_snr_db, 1).key("cond").num(F.cond, 1);
            j.key("sv").nums(F.sv, 4, 3).key("T").num(F.T_us, 3).key("stalls").num(F.stalls, 0);
            j.key("slot_std").num(F.slot_std_us, 2).key("slot_max").num(F.slot_max_us, 1);
            j.key("cfo").num(F.cfo_hz, 0).key("tone").num(F.tone_snr_db, 1).key("pre").num(F.pre_resid_db, 1);
            j.key("blk").nums(std::array<int, 2>{F.blk_ok, F.blk_n}.data(), 2, 0);
            j.key("Hm").arr();
            for (int r = 0; r < 4; r++)
                for (int k = 0; k < 4; k++) j.num(20 * std::log10(std::abs(F.H[r][k]) + 1e-12), 1);
            j.end_arr().key("Hp").arr();
            for (int r = 0; r < 4; r++)
                for (int k = 0; k < 4; k++) j.num(std::arg(F.H[r][k]) * 180 / M_PI, 0);
            j.end_arr().key("Hc").arr();
            for (int r = 0; r < 4; r++)
                for (int k = 0; k < 4; k++) j.num(F.H[r][k].real(), 3).num(F.H[r][k].imag(), 3);
            j.end_arr();
            j.end_obj();
        }
        j.end_obj();
        broadcast(j.s);

        // chat log: resend when anything changed (new message, ACK, tries)
        auto log = link->chat_log();
        size_t sig = log.size();
        for (auto &m : log) sig = sig * 131 + m.acked * 7 + m.id;
        if (sig != chat_sig) {
            chat_sig = sig;
            J c;
            c.obj().key("t").str("chat").key("msgs").arr();
            for (auto &m : log)
                c.obj().key("id").num(m.id, 0).key("mine").boolean(m.mine).key("text").str(m.text)
                    .key("acked").boolean(m.acked).key("tries").num(m.tries, 0).end_obj();
            c.end_arr().end_obj();
            broadcast(c.s);
        }

        if (viz && t - last_viz > 0.25) {
            last_viz = t;
            send_viz(psd_avg, bw_sym);
        }
    }

    void send_viz(const std::vector<float> &psd, double bw_sym)
    {
        J j;
        j.obj().key("t").str("viz");
        VizRaw v;
        if (link->viz_raw(v) && !v.raw.empty()) {
            // ADC: per channel spectrum (dBFS, int8 full scale = 127) and a short I/Q trace
            const size_t n = v.raw.size() / 8;
            j.key("trig").boolean(v.trig).key("lanes").num(v.lanes, 0);
            j.key("adc").obj().key("fs").num(RX_FS, 0).key("rms").nums(v.raw_rms, 4, 2).key("peak").nums(v.raw_peak, 4, 0);
            j.key("psd").arr();
            for (int c = 0; c < 4; c++) {
                std::vector<cf> x(n);
                for (size_t i = 0; i < n; i++) x[i] = cf(v.raw[8 * i + 2 * c], v.raw[8 * i + 2 * c + 1]);
                std::vector<float> p;
                int segs = welch_acc(x.data(), n, 1, 1024, p);
                // full-scale complex sine of amplitude 127 puts 127^2 * (sum w)^2 in one bin
                const float ref = 127.0f * 127.0f * (512.0f * 512.0f) * std::max(1, segs);
                auto r = reduce_db(p, 512, ref);
                j.nums(r.data(), r.size(), 1);
            }
            j.end_arr();
            j.key("iq").arr();
            const size_t m = std::min<size_t>(n, 400);
            for (int c = 0; c < 4; c++) {
                j.arr();
                for (size_t i = 0; i < m; i++) j.num(v.raw[8 * i + 2 * c], 0).num(v.raw[8 * i + 2 * c + 1], 0);
                j.end_arr();
            }
            j.end_arr().end_obj();

            // DDC output at 2.336 MS/s: spectrum and envelope per channel
            const size_t nb = v.bb.size() / 4;
            if (nb >= 1024) {
                j.key("ddc").obj().key("fs").num(RX_FS / DECIM, 0).key("nco").num(v.nco_hz, 0);
                j.key("psd").arr();
                for (int c = 0; c < 4; c++) {
                    std::vector<float> p;
                    int segs = welch_acc(v.bb.data() + c, nb, 4, 512, p);
                    const float ref = 127.0f * 127.0f * (256.0f * 256.0f) * std::max(1, segs);
                    auto r = reduce_db(p, 512, ref);
                    j.nums(r.data(), r.size(), 1);
                }
                j.end_arr().key("env").arr();
                const size_t k = std::max<size_t>(1, nb / 512);
                for (int c = 0; c < 4; c++) {
                    j.arr();
                    for (size_t i = 0; i + k <= nb; i += k) {
                        float mx = 0;
                        for (size_t q = 0; q < k; q++) mx = std::max(mx, std::abs(v.bb[4 * (i + q) + c]));
                        j.num(mx, 1);
                    }
                    j.end_arr();
                }
                j.end_arr().key("dt_us").num(k * DECIM / RX_FS * 1e6, 3).end_obj();
            }
        }
        if (!psd.empty()) {
            float mx = *std::max_element(psd.begin(), psd.end());
            auto r = reduce_db(psd, psd.size(), mx);
            j.key("fpsd").obj().key("fs").num(RX_FS / DECIM, 0).key("bw_sym").num(bw_sym, 0).key("db").nums(r.data(), r.size(), 1).end_obj();
        }
        if (have_viz_frame) {
            const FrameResult &F = viz_frame;
            j.key("fr").obj();
            j.key("seq").num(F.hdr.seq, 0).key("mods").nums(F.hdr.mods.data(), 4, 0).key("snr").nums(F.snr_db, 4, 1);
            j.key("T").num(F.T_us, 3).key("thr").num(10 * std::log10(F.step_thr + 1e-12), 2).key("b").num(F.step_b, 0);
            j.key("det").arr();
            for (float e : F.step_e) j.num(10 * std::log10(std::max(e, 1e-12f)), 1);
            j.end_arr();
            j.key("theta").arr();
            double prev = 0, un = 0;
            for (size_t i = 0; i < F.theta.size(); i++) {
                double d = F.theta[i] - prev;
                d -= 2 * M_PI * std::round(d / (2 * M_PI));
                un = i ? un + d : F.theta[0];
                prev = F.theta[i];
                j.num(un * 180 / M_PI, 1);
            }
            j.end_arr();
            j.key("const").arr();
            for (int k = 0; k < 4; k++) {
                j.arr();
                const auto &c = F.constel[k];
                const size_t step = std::max<size_t>(1, c.size() / 400);
                for (size_t i = 0; i < c.size(); i += step) j.num(c[i].real(), 3).num(c[i].imag(), 3);
                j.end_arr();
            }
            j.end_arr().end_obj();
        }
        j.end_obj();
        broadcast(j.s);
    }
};

Server *g_srv = nullptr;

void ev_handler(struct mg_connection *c, int ev, void *ev_data)
{
    Server &S = *g_srv;
    if (ev == MG_EV_HTTP_MSG) {
        auto *hm = (struct mg_http_message *)ev_data;
        if (mg_match(hm->uri, mg_str("/ws"), NULL)) {
            mg_ws_upgrade(c, hm, NULL);
        } else {
            struct mg_http_serve_opts opts = {};
            opts.root_dir = S.web_dir.c_str();
            opts.extra_headers = "Cache-Control: no-store\r\n";
            mg_http_serve_dir(c, hm, &opts);
        }
    } else if (ev == MG_EV_WS_MSG) {
        auto *wm = (struct mg_ws_message *)ev_data;
        S.on_msg(c, wm->data);
    }
}

void timer_fn(void *) { g_srv->tick(); }

}  // namespace

int serve_main(int argc, char **argv)
{
    int port = 8091;
    std::string web = QM_WEB_DIR, state = "/var/lib/quadrf-mimo/settings.conf";
    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--port") port = atoi(next());
        else if (a == "--web") web = next();
        else if (a == "--state") state = next();
        else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 1; }
    }
    if (!Hw::apply_required_rx()) fprintf(stderr, "warning: quadrf-jtag RX setup failed\n");
    Server S;
    g_srv = &S;
    S.web_dir = web;
    S.state_path = state;
    // mDNS name from Network Setup (quadrf-2, ...); the kernel hostname stays "quadrf"
    char hn[128] = {};
    gethostname(hn, sizeof hn - 1);
    S.host = hn;
    if (FILE *f = fopen("/etc/quadrf/quadrf.conf", "r")) {
        char line[256];
        while (fgets(line, sizeof line, f)) {
            if (strncmp(line, "QUADRF_HOSTNAME=", 16)) continue;
            std::string v = line + 16;
            while (!v.empty() && strchr("'\" \r\n", v.back())) v.pop_back();
            while (!v.empty() && strchr("'\" ", v.front())) v.erase(0, 1);
            if (!v.empty()) S.host = v;
        }
        fclose(f);
    }
    S.settings = make_settings();
    std::string err;
    if (!S.hw.open_all(err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    S.link = std::make_unique<Link>(S.hw);
    LinkConfig cfg;
    if (!S.link->start(cfg, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    S.load_settings();

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    mg_log_set(MG_LL_ERROR);
    mg_mgr_init(&S.mgr);
    char url[64];
    snprintf(url, sizeof url, "http://0.0.0.0:%d", port);
    if (!mg_http_listen(&S.mgr, url, ev_handler, NULL)) {
        fprintf(stderr, "cannot listen on %s\n", url);
        S.link->stop();
        return 1;
    }
    mg_timer_add(&S.mgr, 200, MG_TIMER_REPEAT, timer_fn, NULL);
    fprintf(stderr, "serving %s on %s\n", web.c_str(), url);
    while (g_run) mg_mgr_poll(&S.mgr, 50);
    mg_mgr_free(&S.mgr);
    S.link->stop();
    return 0;
}
