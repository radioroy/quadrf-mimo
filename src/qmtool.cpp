// Offline test tool.
//   qmtool gen   out.bin nframes MODS [cdm4|cdm8]   pattern for tools/ptx2 (uint8 [chips][4])
//   qmtool synth MODS [snr_db] [nframes]            simulated 4x4 channel through the receiver
//   qmtool rx    cap.cs8 f_if                       decode a raw 4ch cs8 capture
// MODS is four digits, one per element: 0 off (no data; the sim still radiates its fixed chip pattern), 1 BPSK, 2 QPSK, 3 8PSK (e.g. 2222, 2022).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <complex>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "frame.hpp"
#include "hw.hpp"
#include "rx.hpp"

using namespace qm;

static Header parse_hdr(const char *mods, const char *mode, int seq)
{
    Header h;
    for (int k = 0; k < 4; k++) h.mods[k] = (uint8_t)(mods[k] - '0');
    h.mode = !mode ? FM_SDM : !strcmp(mode, "cdm4") ? FM_CDM4 : !strcmp(mode, "cdm8") ? FM_CDM8 : FM_SDM;
    h.seq = seq & 15;
    return h;
}

static std::vector<uint8_t> fill_payload(const Header &h)
{
    std::vector<uint8_t> p(payload_bits(h) / 8, 0);
    int nb = payload_blocks(h);
    for (int b = 0; b < nb; b++) fill_prbs(h.seq, b, p.data() + b * BLK_BYTES);
    return p;
}

static void report(const FrameResult &R)
{
    printf("t %8.4f  f_res %+7.1f  T %.3f us  pre %5.1f dB  model %5.1f dB  cond %6.1f  snr [%5.1f %5.1f %5.1f %5.1f]  "
           "stall %d  max %.1f us  adc %.0f",
           R.t_start, R.f_res, R.T_us, R.pre_resid_db, R.model_snr_db, R.cond, R.snr_db[0], R.snr_db[1],
           R.snr_db[2], R.snr_db[3], R.stalls, R.slot_max_us, R.adc_level);
    for (int k = 0; k < 4 && R.pm_hz[k] != 0; k++) printf("%s%.1fk:%.1f", k ? " " : "  pm ", R.pm_hz[k] / 1e3, R.pm_deg[k]);
    if (getenv("QM_PRINTH")) {
        printf("  H");
        for (int r = 0; r < 4; r++)
            for (int k = 0; k < 4; k++) printf(" %.2f %.2f", R.H[r][k].real(), R.H[r][k].imag());
    }
    if (!R.hdr_ok) { printf("  HDR FAIL (margin %.1f)\n", R.hdr_margin); return; }
    auto ref = fill_payload(R.hdr);
    int nb = payload_blocks(R.hdr), ok = 0;
    long be = 0, nbits = 0;
    for (int b = 0; b < nb && (size_t)(b + 1) * BLK_BYTES <= R.payload.size(); b++) {
        uint8_t t;
        int len;
        const uint8_t *d;
        ok += parse_block(R.payload.data() + b * BLK_BYTES, t, len, d);
        if (getenv("QM_HEX")) {
            printf("\n   blk %d:", b);
            for (int i = 0; i < BLK_BYTES; i++) printf(" %02x", R.payload[b * BLK_BYTES + i]);
        }
        for (int i = 0; i < BLK_BYTES; i++) {
            be += __builtin_popcount(R.payload[b * BLK_BYTES + i] ^ ref[b * BLK_BYTES + i]);
            nbits += 8;
        }
    }
    printf("  seq %2d mods %d%d%d%d mode %d  blocks %d/%d  ber %ld/%ld", R.hdr.seq, R.hdr.mods[0], R.hdr.mods[1],
           R.hdr.mods[2], R.hdr.mods[3], R.hdr.mode, ok, nb, be, nbits);
    if (R.hdr.mode == FM_SDM && be) {
        // per-element errors and chip span, SDM bit order is chip-major then element
        int ek[4] = {0, 0, 0, 0}, c0[4] = {-1, -1, -1, -1}, c1[4] = {-1, -1, -1, -1};
        size_t bi = 0;
        for (int i = 0; i < PAY_CHIPS; i++)
            for (int k = 0; k < 4; k++)
                for (int q = 0; q < mod_bits(R.hdr.mods[k]); q++, bi++) {
                    if (bi >= R.payload.size() * 8) continue;
                    int x = ((R.payload[bi >> 3] ^ ref[bi >> 3]) >> (7 - (bi & 7))) & 1;
                    if (x) { ek[k]++; if (c0[k] < 0) c0[k] = i; c1[k] = i; }
                }
        for (int k = 0; k < 4; k++) printf("  e%d:%d[%d..%d]", k, ek[k], c0[k], c1[k]);
    }
    printf("\n");
}

static int cmd_gen(int argc, char **argv)
{
    if (argc < 5) return 1;
    int nf = atoi(argv[3]);
    FILE *f = fopen(argv[2], "wb");
    if (!f) { perror(argv[2]); return 1; }
    for (int i = 0; i < nf; i++) {
        Header h = parse_hdr(argv[4], argc > 5 ? argv[5] : nullptr, i);
        std::vector<std::array<uint8_t, 4>> ch;
        build_frame(h, fill_payload(h), ch);
        fwrite(ch.data(), 4, ch.size(), f);
    }
    fclose(f);
    return 0;
}

// Raw-rate model of the over-the-air path: CW at f_if through a per-element
// complex gain, element phases switched at the JTAG write instants (period
// T with occasional 1.2-1.5 T stalls), LO wander, AWGN, int8 clip at +-44.
// QM_P="sigma_us=1,lam=2,...": tracker parameter sweeps
static void apply_qmp(Receiver &rx)
{
    const char *e = getenv("QM_P");
    if (!e) return;
    RxParams P = rx.params();
    std::string s = e;
    size_t o = 0;
    while (o < s.size()) {
        size_t c = s.find(',', o);
        std::string kv = s.substr(o, c == std::string::npos ? std::string::npos : c - o);
        size_t q = kv.find('=');
        if (q != std::string::npos) {
            std::string k = kv.substr(0, q);
            float v = (float)atof(kv.c_str() + q + 1);
            if (k == "sigma_us") P.sigma_us = v;
            else if (k == "lam") P.lam = v;
            else if (k == "pll_kp") P.pll_kp = v;
            else if (k == "pll_ki") P.pll_ki = v;
            else if (k == "h_mu") P.h_mu = v;
            else if (k == "ls_win") P.ls_win = (int)v;
            else if (k == "late_cap") P.late_cap = v;
            else if (k == "win_lo") P.win_lo = v;
            else if (k == "win_hi") P.win_hi = v;
            else if (k == "late_mult") P.late_mult = v;
            else if (k == "refine") P.refine = (int)v;
            else if (k == "t0_us") P.t0_us = v;
            else if (k == "pre_gate_db") P.pre_gate_db = v;
            else if (k == "det_rel") P.det_rel = v;
            else if (k == "det_k") P.det_k = v;
            else if (k == "gap_us") P.gap_us = v;
            else if (k == "lpf") P.lpf = (int)v;
            else if (k == "la_margin") P.la_margin = v;
            else if (k == "redecide") P.redecide = (int)v;
            else if (k == "tim_alpha") P.tim_alpha = v;
        }
        if (c == std::string::npos) break;
        o = c + 1;
    }
    rx.set_params(P);
}

static int cmd_synth(int argc, char **argv)
{
    if (argc < 3) return 1;
    double snr_db = argc > 3 ? atof(argv[3]) : 25;
    int nf = argc > 4 ? atoi(argv[4]) : 4;
    const double f_if = 500020.5, cfo = 1300, T = 19.2e-6, gap = 370e-6;
    std::mt19937 rng(7);
    std::normal_distribution<double> g(0, 1);
    std::uniform_real_distribution<double> u(0, 1);
    cd H[4][4];
    for (int r = 0; r < 4; r++)
        for (int k = 0; k < 4; k++) H[r][k] = cd(g(rng), g(rng)) * (k == 1 ? 0.3 : 1.0);
    double hp = 0;
    for (auto &r : H)
        for (auto &x : r) hp += std::norm(x);
    double scale = 12.0 / std::sqrt(hp / 4);  // rms ~12 counts per channel, like RX gain 20 at 6 ft
    for (auto &r : H)
        for (auto &x : r) x *= scale;
    // QM_HFILE: measured H, 32 numbers re im row-major [lane][element], DDC-output counts
    if (const char *hf = getenv("QM_HFILE")) {
        FILE *fh = fopen(hf, "r");
        for (int r = 0; fh && r < 4; r++)
            for (int k = 0; k < 4; k++) {
                double re = 0, im = 0;
                if (fscanf(fh, "%lf %lf", &re, &im) == 2) H[r][k] = cd(re, im);
            }
        if (fh) fclose(fh);
    }
    const uint8_t poff = getenv("QM_POFF") ? (uint8_t)atoi(getenv("QM_POFF")) : 0;

    // write schedule
    // QM_BURST="period_ms,len_ms,lo,hi": slow JTAG bursts, writes stretched to lo..hi x T
    // (real units: ~1 ms of 25-56 us writes once per 16.7 ms DSI frame)
    double bp_ms = 0, bl_ms = 0, blo = 1.3, bhi = 2.9;
    if (const char *e = getenv("QM_BURST")) sscanf(e, "%lf,%lf,%lf,%lf", &bp_ms, &bl_ms, &blo, &bhi);
    std::vector<std::pair<double, std::array<uint8_t, 4>>> ev;
    double t = 1e-3;
    std::array<uint8_t, 4> st{{0, 0, 0, 0}};
    for (int i = 0; i < nf; i++) {
        Header h = parse_hdr(argv[2], argc > 5 ? argv[5] : nullptr, i);
        std::vector<std::array<uint8_t, 4>> ch;
        build_frame(h, fill_payload(h), ch);
        for (int c = 0; c < FRAME_CHIPS; c++)
            for (int half = 0; half < 2; half++) {
                double dt = T * (1 + 0.01 * g(rng));
                if (u(rng) < 0.01) dt *= 1.2 + 0.3 * u(rng);
                if (bp_ms > 0 && std::fmod(t * 1e3, bp_ms) < bl_ms) dt = T * (blo + (bhi - blo) * u(rng));
                t += dt;
                st[2 * half] = ch[c][2 * half];
                st[2 * half + 1] = (uint8_t)(ch[c][2 * half + 1] + poff);
                ev.emplace_back(t, st);
            }
        t += gap;
    }
    const double tend = t + 2e-3;
    const size_t N = (size_t)(tend * RX_FS);
    double sigma = 12.0 / std::pow(10, snr_db / 20) / std::sqrt(2) * std::sqrt(DECIM);  // per raw sample
    // QM_NOISE: complex rms per lane at the DDC output (counts) instead of the snr argument
    double sig_l[4] = {sigma, sigma, sigma, sigma};
    if (const char *e = getenv("QM_NOISE"))
        sscanf(e, "%lf,%lf,%lf,%lf", &sig_l[0], &sig_l[1], &sig_l[2], &sig_l[3]);
    if (getenv("QM_NOISE"))
        for (double &s : sig_l) s = s / std::sqrt(2) * std::sqrt(DECIM);
    std::vector<int8_t> raw(8 * N);
    size_t ei = 0;
    cd y[4];
    auto upd = [&](const std::array<uint8_t, 4> &s) {
        for (int r = 0; r < 4; r++) {
            y[r] = 0;
            for (int k = 0; k < 4; k++) y[r] += H[r][k] * std::polar(1.0, 2 * M_PI * s[k] / 256.0);
        }
    };
    upd({{0, 0, 0, 0}});
    // QM_PM="f_hz:deg_peak,...": sinusoidal phase spurs common to all lanes (synth reference
    // spurs on either unit), e.g. "122000:7,68000:6,350000:13"
    std::vector<std::pair<double, double>> pm;
    if (const char *e = getenv("QM_PM"))
        for (const char *q = e; *q;) {
            double f = atof(q), d = 0;
            const char *c = strchr(q, ':');
            if (c) d = atof(c + 1);
            pm.emplace_back(f, d * M_PI / 180);
            q = strchr(q, ',');
            if (!q) break;
            q++;
        }
    // QM_CFO="hz,hz_per_s": offset and linear drift (the TX reference walks after key-up)
    double cfo0 = cfo, ramp = 0;
    if (const char *e = getenv("QM_CFO")) sscanf(e, "%lf,%lf", &cfo0, &ramp);
    double lo = 0;
    for (size_t n = 0; n < N; n++) {
        double tn = n / RX_FS;
        while (ei < ev.size() && ev[ei].first <= tn) upd(ev[ei++].second);
        lo += 2 * M_PI * (f_if + cfo0 + ramp * tn + 400 * std::sin(2 * M_PI * 30 * tn)) / RX_FS;
        double pmv = 0;
        for (size_t i = 0; i < pm.size(); i++) pmv += pm[i].second * std::sin(2 * M_PI * pm[i].first * tn + i);
        cd c = std::polar(1.0, lo + pmv);
        for (int r = 0; r < 4; r++) {
            cd v = y[r] * c + cd(g(rng), g(rng)) * sig_l[r];
            raw[8 * n + 2 * r] = (int8_t)std::clamp(std::lrint(v.real()), -44L, 44L);
            raw[8 * n + 2 * r + 1] = (int8_t)std::clamp(std::lrint(v.imag()), -44L, 44L);
        }
    }
    Receiver rx(f_if);
    rx.offline = true;
    if (getenv("QM_LPF") || getenv("QM_PML")) {
        RxParams P = rx.params();
        if (getenv("QM_LPF")) P.lpf = atoi(getenv("QM_LPF"));
        if (getenv("QM_PML")) P.pm_lines = atoi(getenv("QM_PML"));
        rx.set_params(P);
    }
    apply_qmp(rx);
    auto t0 = std::chrono::steady_clock::now();
    const size_t chunk = 65536;
    const bool dump = getenv("QM_SLOTS") != nullptr;
    for (size_t o = 0; o < N; o += chunk) rx.push(raw.data() + 8 * o, std::min(chunk, N - o), dump);
    if (dump) {
        // true write intervals of frame 0 vs tracked slot lengths
        FrameResult R;
        rx.drain();
        if (rx.pop(R)) {
            int lo = atoi(getenv("QM_SLOTS")), hi = lo + 40;
            if (lo < 0) {  // QM_SLOTS=-1: centre on the first wrong decision
                for (int j = 0; j < (int)R.slot_x.size() && j < (int)ev.size(); j++) {
                    auto s = ev[j].second;
                    s[1] -= poff;
                    s[3] -= poff;
                    bool bad = false;
                    for (int k = 0; k < 4; k++)
                        if (R.hdr.mods[k] != EM_OFF && s[k] != R.slot_x[j][k]) bad = true;
                    if (bad) { lo = std::max(0, j - 20); break; }
                }
                if (lo < 0) lo = 0;
                hi = lo + 40;
                printf("first wrong decision near write %d\n", lo + 20);
            }
            double cum = 0;
            for (int j = 0; j < hi && j + 1 < (int)ev.size() && j < (int)R.slot_us.size(); j++) {
                cum += R.slot_us[j] - (ev[j + 1].first - ev[j].first) * 1e6;
                if (j >= lo) {
                    auto s = ev[j].second;
                    s[1] -= poff;
                    s[3] -= poff;
                    printf("slot %4d true %6.2f got %6.2f  drift %+6.2f us  th %+.2f  tx %3d %3d %3d %3d  dec %3d %3d %3d %3d\n", j,
                           (ev[j + 1].first - ev[j].first) * 1e6, R.slot_us[j], cum, R.theta[j], s[0], s[1], s[2], s[3],
                           R.slot_x[j][0], R.slot_x[j][1], R.slot_x[j][2], R.slot_x[j][3]);
                }
            }
            report(R);
        }
    }
    rx.drain();
    double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("synth %.1f ms of RX in %.1f ms (%.2fx realtime), det %d ok %d fail %d\n", tend * 1e3, el * 1e3,
           tend / el, rx.det_frames.load(), rx.dec_ok.load(), rx.dec_fail.load());
    printf("prof ms: ddc %.1f misc %.1f det %.1f dec %.1f\n", rx.prof_[0] * 1e3, rx.prof_[1] * 1e3,
           rx.prof_[2] * 1e3, rx.prof_[3] * 1e3);
    FrameResult R;
    while (rx.pop(R)) report(R);
    return 0;
}

static int cmd_rx(int argc, char **argv)
{
    if (argc < 4) return 1;
    FILE *f = fopen(argv[2], "rb");
    if (!f) { perror(argv[2]); return 1; }
    Receiver rx(atof(argv[3]));
    rx.offline = true;
    if (getenv("QM_WINHI") || getenv("QM_LPF") || getenv("QM_PML")) {
        RxParams P = rx.params();
        if (getenv("QM_WINHI")) P.win_hi = atof(getenv("QM_WINHI"));
        if (getenv("QM_LPF")) P.lpf = atoi(getenv("QM_LPF"));
        if (getenv("QM_PML")) P.pm_lines = atoi(getenv("QM_PML"));
        rx.set_params(P);
    }
    apply_qmp(rx);
    const bool hist = getenv("QM_HIST") != nullptr;
    int hb[40] = {};
    std::vector<int8_t> buf(8 << 16);
    size_t n, tot = 0;
    auto t0 = std::chrono::steady_clock::now();
    const char *thw = getenv("QM_THETA");  // "w0" : print theta / slot lengths from write w0
    // QM_DUMP=path: per decoded frame [int32 ns][ns x 4 complex64 slot means][ns x 4 u8 decided
    // states][ns x 4 u8 transmitted states][ns float32 slot lengths, us][ns float32 PLL phase, rad]
    FILE *dumpf = getenv("QM_DUMP") ? fopen(getenv("QM_DUMP"), "wb") : nullptr;
    auto take = [&](FrameResult &R) {
        report(R);
        if (dumpf && R.hdr_ok && !R.slot_x.empty()) {
            int32_t ns = (int32_t)R.slot_x.size();
            fwrite(&ns, 4, 1, dumpf);
            fwrite(R.slot_y.data(), sizeof(cf), (size_t)ns * 4, dumpf);
            fwrite(R.slot_x.data(), 4, (size_t)ns, dumpf);
            // transmitted state after each write, from the decoded header and the PRBS fill
            // (QM_POFF: the TX pair offset on el1/el3, 64 in the link)
            const uint8_t po = getenv("QM_POFF") ? (uint8_t)atoi(getenv("QM_POFF")) : 64;
            std::vector<std::array<uint8_t, 4>> ch, tr(ns);
            // link frames: block 0 is the feedback block, the rest PRBS fill; take block 0 as decoded
            auto pl = fill_payload(R.hdr);
            if (!getenv("QM_FILL0"))
                for (int i = 0; i < BLK_BYTES && i < (int)pl.size() && i < (int)R.payload.size(); i++) pl[i] = R.payload[i];
            build_frame(R.hdr, pl, ch);
            std::array<uint8_t, 4> st{{0, 0, 0, 0}};
            for (int j = 0; j < ns && j / 2 < (int)ch.size(); j++) {
                const int c = j / 2, hf = j & 1;
                st[2 * hf] = ch[c][2 * hf];
                st[2 * hf + 1] = (uint8_t)(ch[c][2 * hf + 1] + po);
                tr[j] = st;
            }
            fwrite(tr.data(), 4, (size_t)ns, dumpf);
            std::vector<float> su(ns, 0.f);
            for (int j = 0; j < ns && j < (int)R.slot_us.size(); j++) su[j] = R.slot_us[j];
            fwrite(su.data(), 4, (size_t)ns, dumpf);
            for (int j = 0; j < ns; j++) su[j] = j < (int)R.theta.size() ? R.theta[j] : 0.f;
            fwrite(su.data(), 4, (size_t)ns, dumpf);
        }
        if (thw && R.hdr_ok) {
            int w0 = atoi(thw);
            for (int j = w0; j < w0 + 60 && j < (int)R.theta.size() && j < (int)R.slot_us.size(); j += 2)
                printf("   w %4d  slot %5.1f %5.1f  theta %+7.3f %+7.3f\n", j, R.slot_us[j], R.slot_us[j + 1],
                       R.theta[j], R.theta[j + 1]);
        }
        for (float s : R.slot_us) hb[std::min(39, (int)(s / 2))]++;
    };
    // QM_RESET=k: push the first k chunks, reset() as the link does after its own TX, rewind
    if (getenv("QM_RESET")) {
        int k = atoi(getenv("QM_RESET"));
        for (int i = 0; i < k && (n = fread(buf.data(), 8, 1 << 16, f)) > 0; i++) rx.push(buf.data(), n, hist);
        rx.drain();
        FrameResult R;
        while (rx.pop(R)) printf("pre-reset: "), take(R);
        rx.reset();
        fseek(f, 0, SEEK_SET);
    }
    while ((n = fread(buf.data(), 8, 1 << 16, f)) > 0) {
        rx.push(buf.data(), n, hist);
        tot += n;
        FrameResult R;
        while (rx.pop(R)) take(R);
    }
    fclose(f);
    rx.drain();
    {
        FrameResult R;
        while (rx.pop(R)) take(R);
    }
    if (dumpf) fclose(dumpf);
    if (hist)
        for (int i = 0; i < 40; i++)
            if (hb[i]) printf("slot %2d-%2d us: %d\n", 2 * i, 2 * i + 2, hb[i]);
    double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("%.1f ms of RX in %.1f ms, det %d ok %d fail %d, cfo %.0f Hz\n", tot / RX_FS * 1e3, el * 1e3,
           rx.det_frames.load(), rx.dec_ok.load(), rx.dec_fail.load(), rx.cfo_hz());
    return 0;
}

// carrier frequency and power per 5 ms block of a CW capture (channel-summed lag-16 phase, +-73 kHz unambiguous)
static int cmd_cw(int argc, char **argv)
{
    if (argc < 4) return 1;
    FILE *f = fopen(argv[2], "rb");
    if (!f) { perror(argv[2]); return 1; }
    const double fif = atof(argv[3]);
    const size_t blk = (size_t)(5e-3 * RX_FS), D = 16;
    std::vector<int8_t> b(blk * 8);
    double t = 0, lo = 0;
    const double w = 2 * M_PI * fif / RX_FS;
    while (fread(b.data(), 8, blk, f) == blk) {
        // 16:1 boxcar to baseband, then lag-D autocorrelation
        std::vector<std::complex<double>> y[4];
        for (int c = 0; c < 4; c++) y[c].resize(blk / 16);
        for (size_t i = 0; i < blk / 16 * 16; i++) {
            std::complex<double> r = std::polar(1.0, -lo);
            lo += w;
            for (int c = 0; c < 4; c++)
                y[c][i / 16] += std::complex<double>(b[8 * i + 2 * c], b[8 * i + 2 * c + 1]) * r;
        }
        lo = std::fmod(lo, 2 * M_PI);
        std::complex<double> acc = 0;
        double pw = 0;
        for (int c = 0; c < 4; c++)
            for (size_t i = 0; i + D < y[c].size(); i++) {
                acc += std::conj(y[c][i]) * y[c][i + D];
                pw += std::norm(y[c][i]);
            }
        printf("%7.1f ms  f %+8.0f Hz  p %5.1f dB\n", t * 1e3, std::arg(acc) * RX_FS / 16 / (2 * M_PI * D),
               10 * std::log10(pw / (4.0 * y[0].size()) / 256 + 1e-12));
        t += blk / RX_FS;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "cw")) return cmd_cw(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "gen")) return cmd_gen(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "synth")) return cmd_synth(argc, argv);
    if (argc >= 2 && !strcmp(argv[1], "rx")) return cmd_rx(argc, argv);
    fprintf(stderr, "usage: qmtool gen|synth|rx ...\n");
    return 1;
}
