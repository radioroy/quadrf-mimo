// quadrf-mimo: phase-coded 4x4 MIMO link between two QuadRFs.
//
//   quadrf-mimo cli [--mod bpsk|qpsk|8psk] [--mode sdm|cdm4|cdm8] [--burst N] [--secs S]
//                   [--txg dB] [--rxg dB] [--atpc 0|1] [--auto-rx 0|1]
//                   [--tx-elems MASK] [--rx-lanes MASK] [--target counts] [--say TEXT] [--listen]
//   quadrf-mimo serve [--port 8091] [--web DIR] [--state FILE]
//
// LO stays as the QuadRF control page set it; TX/RX gain belong to the link while it runs.
#include <poll.h>
#include <signal.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "hw.hpp"
#include "link.hpp"

using namespace qm;

static volatile sig_atomic_t g_run = 1;
static void on_sig(int) { g_run = 0; }

int serve_main(int argc, char **argv);

static bool parse_mod(const char *s, LinkConfig &c)
{
    if (!strcmp(s, "bpsk")) c.mod = EM_BPSK;
    else if (!strcmp(s, "qpsk")) c.mod = EM_QPSK;
    else if (!strcmp(s, "8psk")) c.mod = EM_8PSK;
    else return false;
    return true;
}

static int cli_main(int argc, char **argv)
{
    LinkConfig cfg;
    double secs = 0;
    std::string say;
    bool verbose = true;
    int rx_lanes = 0xF;
    for (int i = 2; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> const char * { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--mod") { if (!parse_mod(next(), cfg)) { fprintf(stderr, "bad --mod\n"); return 1; } }
        else if (a == "--txg") cfg.tx_gain = atoi(next());
        else if (a == "--rxg") cfg.rx_gain = atoi(next());
        else if (a == "--atpc") cfg.atpc = atoi(next()) != 0;
        else if (a == "--auto-rx") cfg.auto_rx = atoi(next()) != 0;
        else if (a == "--tx-elems") cfg.tx_elems = (uint8_t)strtol(next(), nullptr, 0);
        else if (a == "--rx-lanes") rx_lanes = (int)strtol(next(), nullptr, 0);
        else if (a == "--target") cfg.level_target = (float)atof(next());
        else if (a == "--mode") {
            std::string m = next();
            cfg.mode = m == "cdm4" ? FM_CDM4 : m == "cdm8" ? FM_CDM8 : FM_SDM;
        }
        else if (a == "--burst") cfg.burst = atoi(next());
        else if (a == "--pair-offset") cfg.pair_offset = (uint8_t)atoi(next());
        else if (a == "--secs") secs = atof(next());
        else if (a == "--say") say = next();
        else if (a == "--listen") cfg.tx_enable = false;
        else if (a == "--quiet") verbose = false;
        else if (a == "--amp") cfg.tx_amp = atof(next());
        else if (a == "--lead") cfg.lead_us = atoi(next());
        else if (a == "--quiet-ms") cfg.quiet_ms = atoi(next());
        else if (a == "--gap") cfg.gap_us = atoi(next());
        else if (a == "--turn") cfg.turn_ms = atoi(next());
        else if (a == "--beacon") { int b = atoi(next()); cfg.beacon_min_ms = cfg.beacon_max_ms = b > 0 ? b : 1000000; }
        else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 1; }
    }
    if (!Hw::apply_required_rx()) fprintf(stderr, "warning: quadrf-jtag RX setup failed\n");
    Hw hw;
    std::string err;
    if (!hw.open_all(err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    Link link(hw);
    link.on_chat = [](const ChatMsg &m) { printf("<< %s\n", m.text.c_str()); fflush(stdout); };
    if (verbose)
        link.on_frame = [](const FrameResult &R) {
            printf("%.3f ", Link::now());
            if (!R.hdr_ok) {
                printf("frame  hdr fail  pre %.1f dB  f_res %+.0f Hz  cfo %+.0f Hz  tone %.1f dB\n", R.pre_resid_db, R.f_res,
                       R.cfo_hz, R.tone_snr_db);
            } else {
                printf("frame  seq %2d %s mods %d%d%d%d m%d  T %.2f us  pre %5.1f  model %5.1f dB  snr %5.1f %5.1f %5.1f %5.1f"
                       "  cond %5.1f  stalls %3d  blk %d/%d\n",
                       R.hdr.seq, R.hdr.last ? "L" : " ", R.hdr.mods[0], R.hdr.mods[1], R.hdr.mods[2], R.hdr.mods[3],
                       R.hdr.mode, R.T_us, R.pre_resid_db, R.model_snr_db, R.snr_db[0], R.snr_db[1], R.snr_db[2],
                       R.snr_db[3], R.cond, R.stalls, R.blk_ok, R.blk_n);
            }
            fflush(stdout);
        };
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    if (!link.start(cfg, err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    {
        RxParams p = link.rx_params();
        p.lanes = rx_lanes;
        link.set_rx_params(p);
    }
    if (!say.empty()) link.send_chat(say);

    double t0 = 0, tlast = 0;
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        t0 = tlast = ts.tv_sec + ts.tv_nsec * 1e-9;
    }
    std::string line;
    while (g_run) {
        pollfd p{0, POLLIN, 0};
        if (poll(&p, 1, 100) > 0 && (p.revents & POLLIN)) {
            char buf[512];
            ssize_t n = read(0, buf, sizeof buf);
            if (n <= 0) p.fd = -1;
            for (ssize_t i = 0; i < n; i++) {
                if (buf[i] == '\n') { if (!line.empty()) link.send_chat(line); line.clear(); }
                else line += buf[i];
            }
        }
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double t = ts.tv_sec + ts.tv_nsec * 1e-9;
        if (t - tlast >= 2.0) {
            tlast = t;
            LinkStats s = link.stats();
            int det, dfail, ddrop, ovr;
            link.rx_counts(det, dfail, ddrop, ovr);
            printf("stat  det %d fail %d drop %d ovr %d  rx ok %d bad %d  blocks %d/%d  ber %.2e (%ld bits)  goodput %.1f kbps  phy %.1f kbps | "
                   "tx frames %d mods %d%d%d%d | peer sees snr %.1f %.1f %.1f %.1f  blocks %d/%d | "
                   "txg %d %s rxg %d  lvl %.0f peer_lvl %.0f peer_txg %d\n",
                   det, dfail, ddrop, ovr, s.frames_ok, s.frames_bad, s.blocks_ok, s.blocks_total,
                   s.fill_bits ? (double)s.fill_errs / s.fill_bits : 0.0, s.fill_bits, s.rx_goodput_bps / 1e3,
                   s.rx_phy_bps / 1e3, s.frames_sent, s.tx_mods[0], s.tx_mods[1], s.tx_mods[2], s.tx_mods[3],
                   s.peer_snr[0], s.peer_snr[1], s.peer_snr[2], s.peer_snr[3], s.peer_blocks_ok,
                   s.peer_blocks_total, s.tx_gain, s.atpc_state, s.rx_gain, s.rx_level, s.peer_level, s.peer_tx_gain);
            for (auto &m : link.chat_log())
                if (m.mine && !m.acked && m.tries) printf("      pending \"%s\" tries %d\n", m.text.c_str(), m.tries);
            fflush(stdout);
        }
        if (secs > 0 && t - t0 > secs) break;
    }
    for (auto &m : link.chat_log())
        if (m.mine) printf("sent \"%s\" %s after %d tries\n", m.text.c_str(), m.acked ? "ACKED" : "not acked", m.tries);
    link.stop();
    return 0;
}

// Turn-on transient test: the parts of tx_on named in `pre` are enabled 1 s ahead, the rest at
// t0, then the CW tone is held for `hold` s. pre: b = PA bias, m = MAX2850 TX mode + RF switch,
// k = FPGA baseband mask.
static int txstep_main(int argc, char **argv)
{
    std::string pre = argc > 2 ? argv[2] : "";
    double hold = argc > 3 ? atof(argv[3]) : 0.6;
    Hw hw;
    std::string err;
    if (!hw.open_all(err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    hw.start_tone(500020.5, 120);
    hw.lease();
    hw.tx_phases(0, 0, 0, 0);
    auto part = [&](char p) {
        if (p == 'm') { hw.write(0x23, 0x0000); hw.spi_tx(0, 0x00C | (0xF << 5)); }
        if (p == 'b') hw.spi_tx(10, 0x001);
        if (p == 'k') hw.write(0x02, 0x000F);
    };
    for (char p : pre) part(p);
    usleep(1000000);
    for (char p : std::string("mbk"))
        if (pre.find(p) == std::string::npos) part(p);
    fprintf(stderr, "t0\n");
    usleep((useconds_t)(hold * 1e6));
    hw.tx_off();
    hw.unlease();
    fprintf(stderr, "t1\n");
    usleep(400000);  // tone keeps running with TX off, as between link bursts
    hw.stop_tone();
    return 0;
}

// Per-element TX check: CW with FPGA baseband mask 0x02 = 1, 2, 4, 8 then 0xF, `hold` s each.
static int txmask_main(int argc, char **argv)
{
    double hold = argc > 2 ? atof(argv[2]) : 0.1;
    Hw hw;
    std::string err;
    if (!hw.open_all(err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    hw.start_tone(500020.5, 120);
    hw.lease();
    hw.tx_phases(0, 0, 0, 0);
    usleep(30000);
    hw.tx_on();
    for (int m : {1, 2, 4, 8, 15}) {
        hw.write(0x02, (uint16_t)m);
        usleep((useconds_t)(hold * 1e6));
    }
    hw.tx_off();
    hw.unlease();
    hw.stop_tone();
    return 0;
}

// Code-to-phase map without step detection: element k alone (mask 1 << k), alternating
// code 0 and code c every `hold` s for c = 0, 16, ..., 240. tools/an_codes.py reads it back.
static int txcodes_main(int argc, char **argv)
{
    double hold = argc > 2 ? atof(argv[2]) : 0.002;
    Hw hw;
    std::string err;
    if (!hw.open_all(err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    hw.start_tone(500020.5, 120);
    hw.lease();
    hw.tx_phases(0, 0, 0, 0);
    usleep(30000);
    hw.tx_on();
    hw.write(0x02, 0x0001);
    usleep(50000);  // marker: el0 code 0 for 50 ms
    for (int k = 0; k < 4; k++) {
        hw.write(0x02, (uint16_t)(1 << k));
        for (int c = 0; c < 256; c += 16)
            for (int h = 0; h < 2; h++) {
                uint8_t p[4] = {0, 0, 0, 0};
                if (h) p[k] = (uint8_t)c;
                hw.tx_phases(p[0], p[1], p[2], p[3]);
                usleep((useconds_t)(hold * 1e6));
            }
    }
    hw.tx_off();
    hw.unlease();
    hw.stop_tone();
    return 0;
}

// Write-timing check: element k alone, flipped by 180 deg on every one of its register writes
// (every second write), in 1024-write batches like the link; `nb` batches.
static int txtoggle_main(int argc, char **argv)
{
    int k = argc > 2 ? atoi(argv[2]) : 0;
    int nb = argc > 3 ? atoi(argv[3]) : 8;
    int mask = argc > 4 ? atoi(argv[4]) : 1 << k;
    // static codes of the other elements, "c0,c1,c2,c3"
    int base[4] = {0, 0, 0, 0};
    if (argc > 5) sscanf(argv[5], "%d,%d,%d,%d", &base[0], &base[1], &base[2], &base[3]);
    Hw hw;
    std::string err;
    if (!hw.open_all(err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    hw.start_tone(500020.5, 120);
    hw.lease();
    hw.tx_phases(0, 0, 0, 0);
    usleep(30000);
    hw.tx_on();
    hw.write(0x02, (uint16_t)mask);
    for (int i = 0; i < 200 && !hw.rx_full(); i++) usleep(1000);
    usleep(5000);
    // optional code cycle for element k ("0,64,0,128,..."), each held `hold` writes
    std::vector<int> cyc;
    int hold = argc > 7 ? atoi(argv[7]) : 2;
    if (argc > 6)
        for (const char *q = argv[6]; *q;) {
            cyc.push_back(atoi(q));
            q = strchr(q, ',');
            if (!q) break;
            q++;
        }
    std::vector<JtagReg> regs;
    for (int c = 0; c < 512; c++) {
        uint8_t p[4] = {(uint8_t)base[0], (uint8_t)base[1], (uint8_t)base[2], (uint8_t)base[3]};
        p[k] = cyc.empty() ? (uint8_t)(base[k] + (c & 1) * 128)
                           : (uint8_t)(base[k] + cyc[(size_t)(2 * c / hold) % cyc.size()]);
        regs.push_back({0x2C, (uint16_t)(p[0] << 8 | p[1])});
        regs.push_back({0x2D, (uint16_t)(p[2] << 8 | p[3])});
    }
    for (int b = 0; b < nb; b++) {
        hw.batch(regs.data(), (unsigned)regs.size());
        usleep(400);
    }
    hw.tx_off();
    hw.unlease();
    hw.stop_tone();
    return 0;
}

// Superposition check: the 255 non-zero combinations of codes {0,64,128,192} on the four elements
// (mask 15), each held 16 writes, alternating with an all-zero reference dwell so the receiver
// can normalize out CFO and drift: ref s1 ref s2 ... in 1024-write batches.
static int txstates_main(int argc, char **argv)
{
    int dw = argc > 2 ? atoi(argv[2]) : 16;
    Hw hw;
    std::string err;
    if (!hw.open_all(err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    hw.start_tone(500020.5, 120);
    hw.lease();
    hw.tx_phases(0, 0, 0, 0);
    usleep(30000);
    hw.tx_on();
    hw.write(0x02, 15);
    // until the RX ring is full the CSI IRQ thread's memcpy stretches JTAG writes to 2-3 T
    for (int i = 0; i < 200 && !hw.rx_full(); i++) usleep(1000);
    usleep(5000);
    std::vector<JtagReg> regs;
    for (int s = 1; s < 256; s++)
        for (int half = 0; half < 2; half++) {
            uint8_t p[4] = {0, 0, 0, 0};
            if (half)
                for (int k = 0; k < 4; k++) p[k] = (uint8_t)(((s >> (2 * k)) & 3) * 64);
            for (int w = 0; w < dw / 2; w++) {
                regs.push_back({0x2C, (uint16_t)(p[0] << 8 | p[1])});
                regs.push_back({0x2D, (uint16_t)(p[2] << 8 | p[3])});
            }
        }
    for (size_t o = 0; o < regs.size(); o += 1024) {
        hw.batch(regs.data() + o, (unsigned)std::min<size_t>(1024, regs.size() - o));
        usleep(400);
    }
    hw.tx_off();
    hw.unlease();
    hw.stop_tone();
    return 0;
}

// Phase-shifter calibration pattern: for each element k and code c = 0, 16, ..., 240,
// 16 writes with all elements at code 0, then 16 writes with element k at code c.
// Repeated `reps` times; tools/an_calib.py fits the per-code complex response.
static int calib_main(int argc, char **argv)
{
    int reps = argc > 2 ? atoi(argv[2]) : 4;
    Hw hw;
    std::string err;
    if (!hw.open_all(err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    hw.start_tone(500020.5, 120);
    hw.lease();
    hw.tx_phases(0, 0, 0, 0);
    usleep(30000);
    hw.tx_on();
    usleep(20000);
    std::vector<JtagReg> regs;
    for (int k = 0; k < 4; k++)
        for (int c = 0; c < 256; c += 16)
            for (int half = 0; half < 2; half++) {
                uint8_t p[4] = {0, 0, 0, 0};
                if (half) p[k] = (uint8_t)c;
                for (int i = 0; i < 8; i++) {
                    regs.push_back({0x2C, (uint16_t)(p[0] << 8 | p[1])});
                    regs.push_back({0x2D, (uint16_t)(p[2] << 8 | p[3])});
                }
            }
    for (int r = 0; r < reps; r++) {
        for (size_t o = 0; o < regs.size(); o += 1024) hw.batch(regs.data() + o, (int)std::min<size_t>(1024, regs.size() - o));
        usleep(2000);
    }
    hw.tx_off();
    hw.unlease();
    hw.stop_tone();
    fprintf(stderr, "calib: %d reps of %zu writes\n", reps, regs.size());
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "cli")) return cli_main(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "calib")) return calib_main(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "txstep")) return txstep_main(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "txmask")) return txmask_main(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "txcodes")) return txcodes_main(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "txtoggle")) return txtoggle_main(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "txstates")) return txstates_main(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "serve")) return serve_main(argc, argv);
    fprintf(stderr, "usage: quadrf-mimo cli|serve [options]\n");
    return 1;
}
