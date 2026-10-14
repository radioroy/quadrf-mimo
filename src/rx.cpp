#include "rx.hpp"
#include "fft.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>

#include "hw.hpp"

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace qm {


namespace {

constexpr double FS = RX_FS / DECIM;

double wall_s()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline cd ph2x(uint8_t p)
{
    static const std::array<cd, 256> lut = [] {
        std::array<cd, 256> t{};
        for (int i = 0; i < 256; i++) t[i] = std::polar(1.0, 2 * M_PI * i / 256.0);
        return t;
    }();
    return lut[p];
}

bool inv4(const Mat4 &A, Mat4 &out)
{
    cd M[4][8];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 8; j++) M[i][j] = j < 4 ? A[i][j] : (j - 4 == i ? 1.0 : 0.0);
    for (int c = 0; c < 4; c++) {
        int p = c;
        for (int r = c + 1; r < 4; r++)
            if (std::abs(M[r][c]) > std::abs(M[p][c])) p = r;
        if (std::abs(M[p][c]) < 1e-12) return false;
        if (p != c)
            for (int j = 0; j < 8; j++) std::swap(M[p][j], M[c][j]);
        cd d = 1.0 / M[c][c];
        for (int j = 0; j < 8; j++) M[c][j] *= d;
        for (int r = 0; r < 4; r++) {
            if (r == c) continue;
            cd f = M[r][c];
            for (int j = 0; j < 8; j++) M[r][j] -= f * M[c][j];
        }
    }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) out[i][j] = M[i][j + 4];
    return true;
}

// H = Y X^H (X X^H)^-1 over K columns; Y, X column-major [K][4]
bool ls_channel(const std::vector<std::array<cd, 4>> &Y, const std::vector<std::array<cd, 4>> &X, Mat4 &H)
{
    Mat4 G{}, C{};
    for (size_t k = 0; k < Y.size(); k++)
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                G[i][j] += X[k][i] * std::conj(X[k][j]);
                C[i][j] += Y[k][i] * std::conj(X[k][j]);
            }
    Mat4 Gi;
    if (!inv4(G, Gi)) return false;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            cd s = 0;
            for (int k = 0; k < 4; k++) s += C[i][k] * Gi[k][j];
            H[i][j] = s;
        }
    return true;
}

}  // namespace

void hermitian_eigs(const Mat4 &A, double ev[4])
{
    // Eigenvalues of a 4x4 Hermitian via the real 8x8 embedding [[Re,-Im],[Im,Re]]
    // (each eigenvalue appears twice); cyclic Jacobi.
    double M[8][8];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            M[i][j] = M[i + 4][j + 4] = A[i][j].real();
            M[i][j + 4] = -A[i][j].imag();
            M[i + 4][j] = A[i][j].imag();
        }
    for (int sweep = 0; sweep < 30; sweep++) {
        double off = 0;
        for (int p = 0; p < 8; p++)
            for (int q = p + 1; q < 8; q++) off += M[p][q] * M[p][q];
        if (off < 1e-18) break;
        for (int p = 0; p < 8; p++)
            for (int q = p + 1; q < 8; q++) {
                if (std::fabs(M[p][q]) < 1e-300) continue;
                double th = (M[q][q] - M[p][p]) / (2 * M[p][q]);
                double t = (th >= 0 ? 1 : -1) / (std::fabs(th) + std::sqrt(th * th + 1));
                double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 8; k++) {
                    double a = M[k][p], b = M[k][q];
                    M[k][p] = c * a - s * b;
                    M[k][q] = s * a + c * b;
                }
                for (int k = 0; k < 8; k++) {
                    double a = M[p][k], b = M[q][k];
                    M[p][k] = c * a - s * b;
                    M[q][k] = s * a + c * b;
                }
            }
    }
    double d[8];
    for (int i = 0; i < 8; i++) d[i] = M[i][i];
    std::sort(d, d + 8, std::greater<double>());
    for (int i = 0; i < 4; i++) ev[i] = std::max(0.0, 0.5 * (d[2 * i] + d[2 * i + 1]));
}

// ---------------------------------------------------------------- decoder

// TX synth phase spurs (common to all RX lanes) from the frame itself, without decisions.
// On a LoS rank-1 channel every element flip is a rotation/scaling of the one scalar
// sum, so a 6 deg peak spur at 68 kHz looks like data to the tracker.
// Inside a slot the element state is constant and the lane-summed lag-1 phase increment is
// just the PM derivative; write boundaries show up as large increments and are masked.
// Lines are picked from the masked increment spectrum (15..280 kHz; the lpf already nulls
// the 250..350 kHz group), then all are fitted jointly by LS on the unmasked samples.
// Output: phase to remove per sample from qa, or empty if nothing was found.
static void pm_estimate(const cf *z, int qa, int N, double fe, const RxParams &P, std::vector<float> &ph,
                        FrameResult &R)
{
    ph.clear();
    if (P.pm_lines <= 0) return;
    const int L = N - qa, D = 4;       // increments over D samples: 584 kHz rate
    const int M = L / D - 1;
    if (M < 2048) return;
    const double w_cfo = 2 * M_PI * fe * D / FS;
    std::vector<float> d(M);
    std::vector<uint8_t> ok(M, 1);
    for (int m = 0; m < M; m++) {
        cd p = 0;
        const int i = qa + m * D;
        for (int c = 0; c < 4; c++) p += cd(z[4 * (i + D) + c]) * std::conj(cd(z[4 * i + c]));
        d[m] = (float)std::remainder(std::arg(p) - w_cfo, 2 * M_PI);
    }
    std::vector<float> s(d);
    for (auto &x : s) x = std::fabs(x);
    std::nth_element(s.begin(), s.begin() + M / 2, s.end());
    const float thr = std::max(5.0f * s[M / 2], 0.05f);
    // dilate by the lpf ramp (2g samples) plus one increment
    const int dil = std::max(P.lpf, 0) / D + 2;
    std::vector<uint8_t> bad(M, 0);
    for (int m = 0; m < M; m++)
        if (std::fabs(d[m]) > thr)
            for (int k = std::max(0, m - dil); k <= std::min(M - 1, m + dil); k++) bad[k] = 1;
    int nok = 0;
    double mu = 0;
    for (int m = 0; m < M; m++) {
        ok[m] = !bad[m];
        if (ok[m]) { mu += d[m]; nok++; }
    }
    if (nok < M / 4) return;
    mu /= nok;

    int NF = 1;
    while (NF < 2 * M) NF <<= 1;
    const double fsd = FS / D;
    // 25 kHz floor: |68 - 52.7| kHz images of a spur against the write rate land below it
    const int k0 = (int)(25e3 / fsd * NF), k1 = std::min((int)(280e3 / fsd * NF), NF / 2 - 2);
    const int excl = std::max(4, (int)(1500.0 / fsd * NF));
    std::vector<std::complex<float>> X(NF);
    std::vector<float> mag(NF / 2), res(M);
    std::vector<double> wl, x;

    // joint LS of phi(n) = sum al cos(w n) + be sin(w n) (n in full-rate samples, w = wl / D)
    // on the unmasked increments phi(nD + D) - phi(nD), plus a constant for the CFO residual
    auto fit = [&]() -> bool {
        const int K = (int)wl.size(), Q = 2 * K + 1;
        std::vector<double> A(Q * Q, 0), y(Q, 0), r(Q);
        for (int m = 0; m < M; m++) {
            if (!ok[m]) continue;
            for (int k = 0; k < K; k++) {
                const double w = wl[k] / D, n0 = (double)m * D, n1 = n0 + D;
                r[2 * k] = std::cos(w * n1) - std::cos(w * n0);
                r[2 * k + 1] = std::sin(w * n1) - std::sin(w * n0);
            }
            r[Q - 1] = 1;
            for (int i = 0; i < Q; i++) {
                y[i] += r[i] * d[m];
                for (int j = 0; j <= i; j++) A[i * Q + j] += r[i] * r[j];
            }
        }
        for (int i = 0; i < Q; i++)
            for (int j = i + 1; j < Q; j++) A[i * Q + j] = A[j * Q + i];
        for (int i = 0; i < Q; i++) {
            int pv = i;
            for (int j = i + 1; j < Q; j++)
                if (std::fabs(A[j * Q + i]) > std::fabs(A[pv * Q + i])) pv = j;
            for (int j = 0; j < Q; j++) std::swap(A[i * Q + j], A[pv * Q + j]);
            std::swap(y[i], y[pv]);
            if (std::fabs(A[i * Q + i]) < 1e-12) return false;
            for (int j = i + 1; j < Q; j++) {
                const double f = A[j * Q + i] / A[i * Q + i];
                for (int k = i; k < Q; k++) A[j * Q + k] -= f * A[i * Q + k];
                y[j] -= f * y[i];
            }
        }
        x.assign(Q, 0);
        for (int i = Q - 1; i >= 0; i--) {
            double acc = y[i];
            for (int j = i + 1; j < Q; j++) acc -= A[i * Q + j] * x[j];
            x[i] = acc / A[i * Q + i];
        }
        for (int m = 0; m < M; m++) {
            double v = d[m] - x[Q - 1];
            for (int k = 0; k < K; k++) {
                const double w = wl[k] / D, n0 = (double)m * D, n1 = n0 + D;
                v -= x[2 * k] * (std::cos(w * n1) - std::cos(w * n0)) + x[2 * k + 1] * (std::sin(w * n1) - std::sin(w * n0));
            }
            res[m] = (float)v;
        }
        return true;
    };

    // pick the strongest line of the current residual, refit all, repeat; masking images of
    // a line are gone from the residual once the line itself is fitted
    for (int m = 0; m < M; m++) res[m] = d[m] - (float)mu;
    float floor_ = 0;
    for (int n = 0; n < P.pm_lines; n++) {
        std::fill(X.begin(), X.end(), 0);
        for (int m = 0; m < M; m++)
            if (ok[m]) X[m] = res[m];
        fft_inplace(X);
        for (int k = 0; k < NF / 2; k++) mag[k] = std::norm(X[k]);
        if (n == 0) {
            std::vector<float> ms(mag.begin() + k0, mag.begin() + k1);
            std::nth_element(ms.begin(), ms.begin() + ms.size() / 2, ms.end());
            floor_ = ms[ms.size() / 2];
        }
        int kb = -1;
        for (int k = k0; k < k1; k++)
            if (mag[k] > 100 * floor_ && (kb < 0 || mag[k] > mag[kb])) {
                bool near = false;
                for (double w : wl)
                    if (std::abs(k - w / (2 * M_PI) * NF) < excl) near = true;
                if (!near) kb = k;
            }
        if (kb < 0) break;
        const double a = std::log(mag[kb - 1] + 1e-20), b = std::log(mag[kb] + 1e-20), c = std::log(mag[kb + 1] + 1e-20);
        const double dk = 0.5 * (a - c) / (a - 2 * b + c);
        wl.push_back(2 * M_PI * (kb + std::clamp(dk, -0.5, 0.5)) / NF);  // rad per increment
        if (!fit()) { wl.pop_back(); break; }
    }
    if (wl.empty() || x.size() != 2 * wl.size() + 1) return;
    const int K = (int)wl.size();
    ph.assign(L, 0.f);
    for (int k = 0; k < K; k++) {
        const double w = wl[k] / D, amp = std::hypot(x[2 * k], x[2 * k + 1]);
        if (k < 4) { R.pm_hz[k] = (float)(w * FS / (2 * M_PI)); R.pm_deg[k] = (float)(amp * 180 / M_PI); }
        // recursive oscillator, renormalized
        cd o = std::polar(1.0, 0.0), st = std::polar(1.0, w);
        const cd co(x[2 * k], -x[2 * k + 1]);
        for (int i = 0; i < L; i++) {
            ph[i] += (float)(co * o).real();
            o *= st;
            if ((i & 1023) == 1023) o /= std::abs(o);
        }
    }
}

bool decode_frame(const cf *z, int N, int a, int b, const RxParams &P, const float nv_in[4],
                  bool viz, FrameResult &R)
{
    R = FrameResult{};
    // Only the last 280 us before the first write: the TX gap is >= 370 us, while the start
    // of the detected gap can include the previous frame's final sub-threshold steps.
    const int qa = std::max(a, b - (int)(280e-6 * FS)), qb = b - 2;
    if (qb - qa < 64 || b >= N) return false;

    // Residual CFO from the quiet tone. Lag products summed per channel (a plain channel sum
    // can cancel), refined over growing lags: D=4 is unambiguous to +-292 kHz, each 4x step
    // keeps the previous estimate's error inside the next lag's +-FS/2D.
    int nq = qb - qa;
    double pw = 0;
    for (int i = 0; i < nq; i++)
        for (int c = 0; c < 4; c++) pw += std::norm(z[4 * (qa + i) + c]);
    double fe = 0;
    for (int D = 4; D <= nq / 3; D *= 4) {
        cd r = 0;
        for (int i = 0; i + D < nq; i++)
            for (int c = 0; c < 4; c++) r += cd(std::conj(z[4 * (qa + i) + c]) * z[4 * (qa + i + D) + c]);
        r *= std::polar(1.0, -2 * M_PI * fe * D / FS);
        fe += std::arg(r) * FS / (2 * M_PI * D);
    }
    R.f_res = (float)fe;
    float nvt = nv_in[0] + nv_in[1] + nv_in[2] + nv_in[3];
    R.tone_snr_db = (float)(10 * std::log10(std::max(pw / nq / std::max(nvt, 1e-6f), 1e-6)));

    const int L = N - qa;
    static thread_local std::vector<float> pmph;
    pm_estimate(z, qa, N, R.f_res, P, pmph, R);

    // derotate from qa, build double cumsums
    static thread_local std::vector<std::array<cd, 4>> S1;
    static thread_local std::vector<double> S2;
    S1.resize(L + 1);
    S2.resize(L + 1);
    S1[0] = {0, 0, 0, 0};
    S2[0] = 0;
    cd rot = std::polar(1.0, -2 * M_PI * R.f_res * (qa - b) / FS);
    const cd step = std::polar(1.0, -2 * M_PI * R.f_res / FS);
    for (int i = 0; i < L; i++) {
        double e = 0;
        for (int c = 0; c < 4; c++) {
            cd v = cd(z[4 * (qa + i) + c]) * rot;
            if (!pmph.empty()) v *= std::polar(1.0, -(double)pmph[i]);
            S1[i + 1][c] = S1[i][c] + v;
            e += std::norm(v);
        }
        S2[i + 1] = S2[i] + e;
        rot *= step;
        if ((i & 1023) == 0) rot /= std::abs(rot);
    }
    const int bl = b - qa;  // local index of the first write

    // noise from first differences inside this frame's own quiet gap when it is long enough
    double nv[4], nvs = 0;
    if (bl > 256 && P.lpf <= 0) {
        std::vector<double> d(bl - 1);
        for (int c = 0; c < 4; c++) {
            for (int i = 0; i + 1 < bl; i++)
                d[i] = std::abs(cd(z[4 * (qa + i + 1) + c]) * step - cd(z[4 * (qa + i) + c]));
            std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
            double med = d[d.size() / 2] / 0.8326;
            nv[c] = std::max(med * med / 2, 1e-3);
            nvs += nv[c];
        }
    } else {
        for (int c = 0; c < 4; c++) { nv[c] = std::max(nv_in[c], 1e-3f); nvs += nv[c]; }
    }
    const double xi_p = 1.0 / (2 * nvs), xi_t = 1.0 / (2 * nvs / 4);
    const double sig = P.sigma_us * 1e-6 * FS;
    // samples either side of a boundary that sit on the channel filter's step ramp
    // (lpf + lpf+1 - 1 samples long, centred on the localized boundary)
    const int g = std::max(P.lpf, 0);
    // mean over [i0, i1) of a slot, guarded when the slot is long enough
    auto slot_mean = [&](int i0, int i1, int r) {
        if (i1 - i0 > 2 * g + 4) { i0 += g; i1 -= g; }
        else if (i1 - i0 > 4) { i0++; i1--; }
        return (S1[i1][r] - S1[i0][r]) / (double)std::max(i1 - i0, 1);
    };

    auto seg_cost = [&](int i0, int i1, const cd v[4]) {
        double s = S2[i1] - S2[i0], vv = 0;
        for (int c = 0; c < 4; c++) {
            s -= 2 * (std::conj(v[c]) * (S1[i1][c] - S1[i0][c])).real();
            vv += std::norm(v[c]);
        }
        return s + (i1 - i0) * vv;
    };

    // ---- preamble: unstructured two-sided step search with a timing prior
    const int npre = 2 * PRE_CHIPS;
    double T = P.t0_us * 1e-6 * FS;
    std::vector<int> bnd;
    bnd.reserve(NWRITES + 1);
    bnd.push_back(bl);
    {
        const int Lw = (int)(0.35 * T) + g / 2, Lp = Lw - g;
        for (int j = 0; j < npre - 1; j++) {
            int b0 = bnd.back();
            int n0 = (int)(b0 + 0.65 * T), n1 = (int)(b0 + 1.4 * T);
            if (n0 - Lw < 0 || n1 + Lw >= L) return false;
            double best = -1e300;
            int bn = n0;
            for (int n = n0; n < n1; n++) {
                double d = 0;
                for (int c = 0; c < 4; c++)
                    d += std::norm((S1[n + Lw][c] - S1[n + g][c]) - (S1[n - g][c] - S1[n - Lw][c]));
                double sc = xi_p * d / (2.0 * Lp) - 0.5 * std::pow((n - b0 - T) / sig, 2);
                if (sc > best) { best = sc; bn = n; }
            }
            bnd.push_back(bn);
            if (j >= 8) {
                T = double(bnd.back() - bnd.front()) / (bnd.size() - 1);
            } else if (j >= 3) {
                std::vector<int> d(bnd.size() - 1);
                for (size_t k = 1; k < bnd.size(); k++) d[k - 1] = bnd[k] - bnd[k - 1];
                std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
                T = d[d.size() / 2];
            }
        }
    }

    // Writes that flip only a weak element give little or no step, and the free search above
    // then latches onto noise. Fit a line through the boundaries (two passes, dropping points
    // more than 2 us off) and pull outliers onto it; the known-state passes below re-place them.
    {
        const double tol = 2e-6 * FS;
        std::vector<char> use(bnd.size(), 1);
        double t0 = bnd[0], tt = T;
        for (int pass = 0; pass < 2; pass++) {
            double sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0;
            for (size_t j = 0; j < bnd.size(); j++)
                if (use[j]) { sx += j; sy += bnd[j]; sxx += (double)j * j; sxy += (double)j * bnd[j]; n++; }
            if (n < 8) break;
            tt = (n * sxy - sx * sy) / (n * sxx - sx * sx);
            t0 = (sy - tt * sx) / n;
            for (size_t j = 0; j < bnd.size(); j++) use[j] = std::fabs(bnd[j] - (t0 + tt * j)) < tol;
        }
        if (tt > 0.8 * P.t0_us * 1e-6 * FS && tt < 1.25 * P.t0_us * 1e-6 * FS) {
            T = tt;
            for (size_t j = 1; j < bnd.size(); j++)
                if (!use[j]) bnd[j] = (int)std::lround(t0 + tt * j);
        }
    }

    // slot j (between write j and j+1) holds the state after write j
    const auto &pre = preamble_states();
    auto state_after_write = [&](int w) {
        int c = w / 2;
        std::array<uint8_t, 4> s;
        if (w % 2 == 0) {
            s = {pre[c][0], pre[c][1], c > 0 ? pre[c - 1][2] : (uint8_t)0, c > 0 ? pre[c - 1][3] : (uint8_t)0};
        } else {
            s = pre[c];
        }
        return s;
    };
    std::vector<std::array<cd, 4>> Xp(npre + 1);  // column 0 = quiet (all phases 0)
    Xp[0] = {1.0, 1.0, 1.0, 1.0};
    for (int w = 0; w < npre; w++) {
        auto s = state_after_write(w);
        for (int k = 0; k < 4; k++) Xp[w + 1][k] = ph2x(s[k]);
    }
    std::array<cd, 4> yq{};
    for (int c = 0; c < 4; c++) yq[c] = (S1[bl - 2 - g][c] - S1[0][c]) / (double)(bl - 2 - g);

    Mat4 H{};
    std::vector<std::array<cd, 4>> Y, Xs;
    auto pre_ls = [&]() {
        Y.assign(1, yq);
        Xs.assign(1, Xp[0]);
        for (size_t j = 0; j + 1 < bnd.size(); j++) {
            std::array<cd, 4> m;
            for (int c = 0; c < 4; c++) m[c] = slot_mean(bnd[j], bnd[j + 1], c);
            Y.push_back(m);
            Xs.push_back(Xp[j + 1]);
        }
        return ls_channel(Y, Xs, H);
    };
    if (!pre_ls()) return false;
    for (int it = 0; it < P.refine; it++) {
        const int Lw = (int)(0.35 * T) + g / 2;
        std::vector<std::array<cd, 4>> V(npre + 1);
        for (int j = 0; j <= npre; j++)
            for (int r = 0; r < 4; r++) {
                cd s = 0;
                for (int k = 0; k < 4; k++) s += H[r][k] * Xp[j][k];
                V[j][r] = s;
            }
        // A slow JTAG write inside the preamble is 2-3 T. A search capped at 1.4 T then pins
        // at its late edge and leaves the last boundary tens of us early, which the tracker
        // inherits at the first header write. Searching wide every time instead lets weak
        // single-element steps lose to the next, stronger write, so widen only when pinned.
        std::vector<int> nb{bnd[0]};
        for (size_t j = 1; j < bnd.size(); j++) {
            int b0 = nb.back();
            double e0 = b0 + T;
            auto search = [&](int n0, int n1, bool late) {
                double best = 1e300;
                int bn = n0;
                for (int n = n0; n < n1; n++) {
                    const double dn = n - e0;
                    const double pq = !late || dn < 0 ? 0.5 * (dn / sig) * (dn / sig)
                                                      : std::min(0.5 * (dn / (P.late_mult * sig)) * (dn / (P.late_mult * sig)),
                                                                 (double)P.late_cap);
                    double cst = xi_p * (seg_cost(n - Lw, n - g, V[j].data()) + seg_cost(n + g, n + Lw, V[j + 1].data())) + pq;
                    if (cst < best) { best = cst; bn = n; }
                }
                return bn;
            };
            const int n1 = (int)(b0 + 1.4 * T);
            int bn = search((int)(b0 + 0.65 * T), n1, false);
            if (bn >= n1 - 2) bn = search((int)(b0 + 0.65 * T), std::min((int)(b0 + P.win_hi * T), L - Lw - 1), true);
            nb.push_back(bn);
        }
        bnd = nb;
        {
            std::vector<int> d(bnd.size() - 1);
            for (size_t k = 1; k < bnd.size(); k++) d[k - 1] = bnd[k] - bnd[k - 1];
            std::vector<int> ds = d;
            std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
            const double med = ds[ds.size() / 2];
            double s = 0;
            int n = 0;
            for (int v : d)
                if (std::fabs(v - med) < 0.15 * med) { s += v; n++; }
            T = n ? s / n : med;
        }
        if (!pre_ls()) return false;
    }
    {
        double r = 0, t = 0;
        for (size_t k = 0; k < Y.size(); k++)
            for (int i = 0; i < 4; i++) {
                cd p = 0;
                for (int j = 0; j < 4; j++) p += H[i][j] * Xs[k][j];
                r += std::norm(Y[k][i] - p);
                t += std::norm(Y[k][i]);
            }
        R.pre_resid_db = (float)(10 * std::log10(std::max(r / t, 1e-9)));
    }
    // A real preamble fits the known states to better than -6 dB even at 3 dB per-sample SNR;
    // false starts inside a frame sit near 0 dB. Also keeps CRC-8 false headers out.
    if (R.pre_resid_db > P.pre_gate_db) {
        if (getenv("QM_TRK")) {
            fprintf(stderr, "pre gate %.1f dB, bl %d, slots us:", R.pre_resid_db, (int)bnd.size());
            for (size_t j = 1; j < bnd.size(); j++) fprintf(stderr, " %.1f", (bnd[j] - bnd[j - 1]) / FS * 1e6);
            fprintf(stderr, "\n");
        }
        R.T_us = (float)(T / FS * 1e6);
        R.H = H;
        return true;
    }

    // ---- tracker over header + payload writes
    std::array<uint8_t, 4> cur = pre[PRE_CHIPS - 1];
    std::vector<std::array<uint8_t, 4>> slot_state(npre);
    for (int w = 0; w < npre; w++) slot_state[w] = state_after_write(w);
    std::vector<float> theta(npre, 0.f);
    // PLL state: phase th at sample tpos plus frequency wf in rad/sample. The TX reference
    // walks up to ~15 kHz (2.6 ppm at 5.8 GHz) in the ~200 ms after tx_on, i.e. several kHz
    // across one frame, and stalled slots are up to 3T long, so phase is propagated by
    // elapsed samples rather than by slot count.
    double th = 0, wf = 0, tpos = bnd.back();
    // Smoothed boundary track: outside the slow bursts the JTAG writes are regular to well
    // under a us, while a single boundary estimate is off by 1-3 us on weak steps. Predicting
    // from bp + T hands that error to the next search; a QPSK write then lands at the
    // window edge and the pair decision goes wrong.
    double tq = bnd.back();
    Mat4 Ht = H;
    std::vector<std::array<cd, 4>> lsY, lsX;
    lsY.reserve(NWRITES);
    lsX.reserve(NWRITES);
    std::vector<double> hdr_llr(HDR_CHIPS, 0.0);
    Header hdr;
    bool hdr_ok = false;
    int la_tried = 0, la_flip = 0;

    std::vector<uint8_t> alph[2];
    std::string trk;
    if (getenv("QM_TRK")) {
        char ln[32];
        trk = "pre slots us:";
        for (size_t j = 1; j < bnd.size(); j++) {
            snprintf(ln, sizeof ln, " %.1f", (bnd[j] - bnd[j - 1]) / FS * 1e6);
            trk += ln;
        }
        snprintf(ln, sizeof ln, "  T %.2f\n", T / FS * 1e6);
        trk += ln;
    }

    auto alph_for = [&](int c, int el, std::vector<uint8_t> &a) {
        a.clear();
        if (c < PAY_CHIP0) {
            a.push_back(elem_phase(EM_BPSK, 0, c));
            a.push_back(elem_phase(EM_BPSK, 1, c));
        } else if (hdr.mode == FM_SDM) {
            const int m = hdr.mods[el];
            for (int q = 0; q < mod_order(m); q++) a.push_back(elem_phase(m, q, c));
        } else {
            a.push_back(elem_phase(EM_BPSK, 0, c));
            if (hdr.mods[el] != EM_OFF) a.push_back(elem_phase(EM_BPSK, 1, c));
        }
    };
    // Best localization cost of write w given the previous boundary bp_ and state cur_, same
    // cost as the main search below. Used one write ahead to settle near-ties between an
    // early boundary and a late one.
    auto next_cost = [&](int w, int bp_, const std::array<uint8_t, 4> &cur_) {
        const int c = w / 2, ea = 2 * (w % 2), eb = ea + 1;
        std::vector<uint8_t> A0, A1;
        alph_for(c, ea, A0);
        alph_for(c, eb, A1);
        const bool hw = c < PAY_CHIP0;
        const int K = hw ? 2 : (int)(A0.size() * A1.size());
        const double e0 = bp_ + T;
        const int Lw = std::max(g + 4, (int)(0.4 * T) + g / 2), Lp = Lw - g;
        const int r1 = std::max(std::max(1, g) + 4, (int)(0.85 * T) - g);
        const int Lr = std::max(4, (int)(0.7 * T) - 1);
        const int n0 = (int)(bp_ + P.win_lo * T);
        const int n1 = std::min((int)(bp_ + P.win_hi * T), L - std::max(Lr, r1) - 2);
        if (n1 <= n0 || n1 - n0 > 192) return 0.0;
        const cd rr = std::polar(1.0, th + wf * (e0 - tpos));
        cd vpre[4], ga[4], gb[4];
        double vpre2 = 0;
        for (int r = 0; r < 4; r++) {
            cd s = 0;
            for (int k = 0; k < 4; k++) s += Ht[r][k] * ph2x(cur_[k]);
            vpre[r] = rr * s;
            vpre2 += std::norm(vpre[r]);
            ga[r] = rr * Ht[r][ea];
            gb[r] = rr * Ht[r][eb];
        }
        const cd xa0 = ph2x(cur_[ea]), xb0 = ph2x(cur_[eb]);
        double best = 1e300;
        for (int n = n0; n < n1; n++) {
            double C0 = (S2[n + Lw] - S2[n + g]) + (S2[n - g] - S2[n - Lw]) + 2 * Lp * vpre2;
            cd ua = 0, ub = 0;
            for (int r = 0; r < 4; r++) {
                C0 -= 2 * (std::conj(vpre[r]) * ((S1[n + Lw][r] - S1[n + g][r]) + (S1[n - g][r] - S1[n - Lw][r]))).real();
                cd u = (S1[n + Lw][r] - S1[n + g][r]) - (double)Lp * vpre[r];
                ua += std::conj(ga[r]) * u;
                ub += std::conj(gb[r]) * u;
            }
            const double dn = n - e0;
            const double pq = dn < 0 ? 0.5 * (dn / sig) * (dn / sig)
                                     : std::min(0.5 * (dn / (P.late_mult * sig)) * (dn / (P.late_mult * sig)),
                                                (double)P.late_cap);
            for (int h = 0; h < K; h++) {
                const uint8_t pa = hw ? A0[h] : A0[h / A1.size()], pb = hw ? A0[h] : A1[h % A1.size()];
                const cd da = ph2x(pa) - xa0, db = ph2x(pb) - xb0;
                double dv2 = 0;
                for (int r = 0; r < 4; r++) dv2 += std::norm(ga[r] * da + gb[r] * db);
                const double cst = xi_t * (C0 + Lp * dv2 - 2 * (std::conj(da) * ua + std::conj(db) * ub).real()) + P.lam * pq;
                best = std::min(best, cst);
            }
        }
        return best;
    };

    for (int w = npre; w < NWRITES - 2; w++) {
        const int c = w / 2, half = w % 2, ea = 2 * half, eb = ea + 1;
        if (w == npre + 2 * HDR_CHIPS) {
            uint32_t word = 0;
            float marg = 1e30f;
            for (int i = 0; i < HDR_CHIPS; i++) {
                if (hdr_llr[i] < 0) word |= 1u << i;
                marg = std::min(marg, (float)std::fabs(hdr_llr[i]));
            }
            R.hdr_margin = marg;
            hdr_ok = Header::unpack(word, hdr);
            if (!hdr_ok && getenv("QM_TRK")) fprintf(stderr, "%shdr word %06x\n", trk.c_str(), word);
            if (!hdr_ok) break;
        }
        for (int s = 0; s < 2; s++) {
            int el = s == 0 ? ea : eb;
            alph[s].clear();
            if (c < PAY_CHIP0) {
                alph[s].push_back(elem_phase(EM_BPSK, 0, c));
                alph[s].push_back(elem_phase(EM_BPSK, 1, c));
            } else if (hdr.mode == FM_SDM) {
                int m = hdr.mods[el];
                for (int q = 0; q < mod_order(m); q++) alph[s].push_back(elem_phase(m, q, c));
            } else {
                alph[s].push_back(elem_phase(EM_BPSK, 0, c));
                if (hdr.mods[el] != EM_OFF) alph[s].push_back(elem_phase(EM_BPSK, 1, c));
            }
        }
        const bool hdr_w = c < PAY_CHIP0;
        const int K = hdr_w ? 2 : (int)(alph[0].size() * alph[1].size());
        // Disabled elements are keyed down and marked off in the header. With both elements of a pair off
        // its register write moves nothing on air; inside a slow JTAG burst that write can
        // run 3 T, and the next live write then falls outside a one-write window.
        auto pair_dead = [&](int wq) {
            const int e = 2 * (wq % 2);
            return hdr_ok && wq / 2 >= PAY_CHIP0 && hdr.mode == FM_SDM && hdr.mods[e] == EM_OFF &&
                   hdr.mods[e + 1] == EM_OFF;
        };
        const bool dead = pair_dead(w);
        const bool after_dead = !dead && w > npre && pair_dead(w - 1);

        const int bp = bnd.back();
        const double e0 = tq + T;
        if (dead) {
            const int nb = std::max(bp + 1, (int)std::lround(e0));
            theta.push_back((float)th);
            bnd.push_back(nb);
            tq = e0;
            for (int s = 0; s < 2; s++) cur[ea + s] = alph[s][0];
            slot_state.push_back(cur);
            continue;
        }
        // live write after a dead one: search from the last live boundary over two writes
        const int bw = after_dead ? bnd[bnd.size() - 2] : bp;
        // JTAG writes come in slow bursts (25-56 us, ~1 ms long, once per DSI frame), so the
        // boundary search reaches 3.2 T. Each candidate is judged on its own local windows:
        // Lw either side for localization, then Lr after it for the symbol decision; writes
        // never come sooner than ~0.85 T, so neither window crosses into another slot.
        // With the channel filter on, both windows give up g samples next to the candidate,
        // and the decision window ends g before the earliest next write (0.85 T).
        const int Lw = std::max(g + 4, (int)(0.4 * T) + g / 2), Lp = Lw - g;
        const int r0 = std::max(1, g), r1 = std::max(r0 + 4, (int)(0.85 * T) - g);
        const int Lr = std::max(4, (int)(0.7 * T) - 1);
        const int n0 = (int)(bw + (after_dead ? 2 * P.win_lo : P.win_lo) * T);
        const int n1 = std::min((int)(bw + (after_dead ? 2 * P.win_hi : P.win_hi) * T), L - std::max(Lr, r1) - 2);
        if (n1 <= n0 || n1 - n0 > 320) break;
        const int Nn = n1 - n0;

        const cd rr = std::polar(1.0, th + wf * (e0 - tpos));
        cd vpre[4];
        for (int r = 0; r < 4; r++) {
            cd s = 0;
            for (int k = 0; k < 4; k++) s += Ht[r][k] * ph2x(cur[k]);
            vpre[r] = rr * s;
        }
        cd ga[4], gb[4];
        for (int r = 0; r < 4; r++) { ga[r] = rr * Ht[r][ea]; gb[r] = rr * Ht[r][eb]; }

        // Localization: full residual over 2 Lw samples around each candidate n, v_pre before
        // and v_h = v_pre + dv after. Every candidate gets the same number of samples, so the
        // costs compare directly, and a late candidate pays for a left side that has moved on.
        // Split as C0(n) (no-change residual) + [-2 Re(dv^H u(n)) + Lw |dv|^2] with
        // u(n) = S1[n, n+Lw) - Lw v_pre, projected onto the two switched columns.
        alignas(16) float c0[320], uar[320], uai[320], ubr[320], ubi[320], sc[320];
        double vpre2 = 0;
        for (int r = 0; r < 4; r++) vpre2 += std::norm(vpre[r]);
        for (int i = 0; i < Nn; i++) {
            const int n = n0 + i;
            double C0 = (S2[n + Lw] - S2[n + g]) + (S2[n - g] - S2[n - Lw]) + 2 * Lp * vpre2;
            cd ua = 0, ub = 0;
            for (int r = 0; r < 4; r++) {
                C0 -= 2 * (std::conj(vpre[r]) * ((S1[n + Lw][r] - S1[n + g][r]) + (S1[n - g][r] - S1[n - Lw][r]))).real();
                cd u = (S1[n + Lw][r] - S1[n + g][r]) - (double)Lp * vpre[r];
                ua += std::conj(ga[r]) * u;
                ub += std::conj(gb[r]) * u;
            }
            // Across a dead write the window spans several chips' steps, and in BPSK the state
            // two chips on equals v_pre half the time, so a later step fits Lw just as well.
            // Make it explain the span back to the last live boundary, but only charge what's
            // above noise + 10% of signal power: keyed-off elements still leak (-18..-50 dB),
            // their writes move the span a little, and a skipped chip moves it by ~200%.
            const int xa = bw + g + 1, xb = n - Lw;
            if (after_dead && xb > xa) {
                const cd rot = std::polar(1.0, wf * (0.5 * (xa + xb) - e0));
                double ext = (S2[xb] - S2[xa]) + (xb - xa) * vpre2;
                for (int r = 0; r < 4; r++) ext -= 2 * (std::conj(rot * vpre[r]) * (S1[xb][r] - S1[xa][r])).real();
                C0 += std::max(0.0, ext - (xb - xa) * (2.0 / xi_t + 0.1 * vpre2));
            }
            // quadratic on the early side; late side wider and capped so a clear step
            // inside a slow burst still wins, while a weak one defaults to e0
            const double dn = n - e0;
            const double pq = dn < 0 ? 0.5 * (dn / sig) * (dn / sig)
                                     : std::min(0.5 * (dn / (P.late_mult * sig)) * (dn / (P.late_mult * sig)),
                                                (double)P.late_cap);
            c0[i] = (float)(xi_t * C0 + P.lam * pq);
            uar[i] = (float)(-2 * xi_t * ua.real());
            uai[i] = (float)(-2 * xi_t * ua.imag());
            ubr[i] = (float)(-2 * xi_t * ub.real());
            ubi[i] = (float)(-2 * xi_t * ub.imag());
        }
        const cd xa0 = ph2x(cur[ea]), xb0 = ph2x(cur[eb]);
        auto hyp = [&](int h, uint8_t &pa, uint8_t &pb) {
            if (hdr_w) pa = pb = alph[0][h];
            else { pa = alph[0][h / alph[1].size()]; pb = alph[1][h % alph[1].size()]; }
        };
        float best = 1e30f;
        int bi = 0, bh_loc = 0;
        float loc_c[64];
        int loc_n[64];
        for (int h = 0; h < K; h++) {
            uint8_t pa, pb;
            hyp(h, pa, pb);
            const cd da = ph2x(pa) - xa0, db = ph2x(pb) - xb0;
            double dv2 = 0;
            for (int r = 0; r < 4; r++) dv2 += std::norm(ga[r] * da + gb[r] * db);
            const float kd = (float)(xi_t * Lp * dv2);
            const float dar = (float)da.real(), dai = (float)da.imag(), dbr = (float)db.real(), dbi = (float)db.imag();
            for (int i = 0; i < Nn; i++) sc[i] = c0[i] + kd + dar * uar[i] + dai * uai[i] + dbr * ubr[i] + dbi * ubi[i];
            int im = 0;
            for (int i = 1; i < Nn; i++)
                if (sc[i] < sc[im]) im = i;
            if (h < 64) { loc_c[h] = sc[im]; loc_n[h] = n0 + im; }
            if (sc[im] < best) { best = sc[im]; bi = im; bh_loc = h; }
        }
        int nb = n0 + bi;
        // QPSK pair steps that nearly cancel on all four lanes (two weak, nearly collinear
        // H columns) make a boundary at ~T with no real change cost about as little as the
        // true, late write inside a slow JTAG burst. Taking the wrong one shifts every later
        // write onto the other pair. If a boundary at a clearly different time is within
        // la_margin, let the next write decide: after a phantom it has to explain the other
        // pair's change and fits badly.
        const int c1 = (w + 1) / 2;
        if (P.la_margin > 0 && K <= 64 && w + 1 < NWRITES - 2 && (c1 < PAY_CHIP0 || hdr_ok) && !pair_dead(w + 1)) {
            int alt = -1;
            for (int h = 0; h < K; h++)
                if (std::abs(loc_n[h] - nb) > 0.4 * T && (alt < 0 || loc_c[h] < loc_c[alt])) alt = h;
            if (alt >= 0 && loc_c[alt] - best < P.la_margin) {
                std::array<uint8_t, 4> ca = cur, cb = cur;
                hyp(bh_loc, ca[ea], ca[eb]);
                hyp(alt, cb[ea], cb[eb]);
                const double ja = best + next_cost(w + 1, nb, ca);
                const double jb = loc_c[alt] + next_cost(w + 1, loc_n[alt], cb);
                la_tried++;
                if (jb < ja) { nb = loc_n[alt]; la_flip++; }
            }
        }
        if (after_dead) bnd.back() = (bw + nb) / 2;

        // Symbol decision: ML on [nb+1, nb+1+Lr) given v_pre (skip the sample the boxcar
        // decimator smears across the boundary).
        const int da0 = g > 0 ? nb + r0 : nb + 1, da1 = g > 0 ? nb + r1 : nb + 1 + Lr;
        const double Ld = da1 - da0;
        cd ua = 0, ub = 0;
        for (int r = 0; r < 4; r++) {
            cd u = (S1[da1][r] - S1[da0][r]) - Ld * vpre[r];
            ua += std::conj(ga[r]) * u;
            ub += std::conj(gb[r]) * u;
        }
        double bestc = 1e300, cost_b[2] = {1e300, 1e300};
        int bh = 0;
        for (int h = 0; h < K; h++) {
            uint8_t pa, pb;
            hyp(h, pa, pb);
            const cd da = ph2x(pa) - xa0, db = ph2x(pb) - xb0;
            double dv2 = 0;
            for (int r = 0; r < 4; r++) dv2 += std::norm(ga[r] * da + gb[r] * db);
            double cst = xi_t * (Ld * dv2 - 2 * (std::conj(da) * ua + std::conj(db) * ub).real());
            if (cst < bestc) { bestc = cst; bh = h; }
            if (hdr_w) cost_b[h] = cst;
        }
        if (hdr_w) hdr_llr[c - PRE_CHIPS] += cost_b[1] - cost_b[0];
        uint8_t pa, pb;
        hyp(bh, pa, pb);
        static const char *trkw = getenv("QM_TRKW");  // "lo,hi": per-hypothesis costs for writes lo..hi
        if (trkw) {
            int lo = 0, hi = -1;
            sscanf(trkw, "%d,%d", &lo, &hi);
            if (w >= lo && w <= hi) {
                char ln[96];
                snprintf(ln, sizeof ln, "w %d el%d/%d cur %d %d slot %.1f us pick %d %d  costs:", w, ea, eb, cur[ea],
                         cur[eb], (nb - bp) / FS * 1e6, pa, pb);
                trk += ln;
                for (int h = 0; h < K; h++) {
                    uint8_t qa, qb;
                    hyp(h, qa, qb);
                    const cd da = ph2x(qa) - xa0, db = ph2x(qb) - xb0;
                    double dv2 = 0;
                    for (int r = 0; r < 4; r++) dv2 += std::norm(ga[r] * da + gb[r] * db);
                    double cst = xi_t * (Ld * dv2 - 2 * (std::conj(da) * ua + std::conj(db) * ub).real());
                    snprintf(ln, sizeof ln, " %d/%d:%.0f", qa, qb, cst - bestc);
                    trk += ln;
                }
                trk += "\n    loc (cost@us):";
                for (int h = 0; h < K && h < 64; h++) {
                    uint8_t qa, qb;
                    hyp(h, qa, qb);
                    snprintf(ln, sizeof ln, " %d/%d:%.0f@%.1f", qa, qb, loc_c[h] - best, (loc_n[h] - bp) / FS * 1e6);
                    trk += ln;
                }
                snprintf(ln, sizeof ln, "  [n0 %.1f n1 %.1f us]\n", (n0 - bp) / FS * 1e6, (n1 - bp) / FS * 1e6);
                trk += ln;
            }
        }
        static const bool dbg = getenv("QM_TRK") != nullptr;
        if (dbg && hdr_w) {
            char ln[160];
            snprintf(ln, sizeof ln, "w %d  slot %.1f  n %d..%d  best %.1f  h %d  cost %.1f %.1f  th %.3f wf %.5f\n", w,
                     (nb - bp) / FS * 1e6, n0 - bp, n1 - bp, best, bh, cost_b[0], cost_b[1], th, wf);
            trk += ln;
        }

        // common-phase PLL and LMS channel update from the slot that just ended
        cd m[4], pred[4], xs[4];
        for (int k = 0; k < 4; k++) xs[k] = ph2x(cur[k]);
        const double nlen = std::max(nb - bp, 1);
        const double nc = 0.5 * (bp + nb);
        const double php = th + wf * (nc - tpos);
        cd dot = 0;
        for (int r = 0; r < 4; r++) {
            m[r] = g > 0 ? slot_mean(bp, nb, r) : (S1[nb][r] - S1[bp][r]) / nlen;
            cd s = 0;
            for (int k = 0; k < 4; k++) s += Ht[r][k] * xs[k];
            pred[r] = s;
            dot += std::conj(s) * m[r] * std::polar(1.0, -php);
        }
        double err = std::arg(dot);
        th = php + P.pll_kp * err;
        wf += P.pll_ki * err / T;
        tpos = nc;
        const cd dr = std::polar(1.0, -th);
        for (int r = 0; r < 4; r++) {
            cd res = m[r] * dr - pred[r];
            for (int k = 0; k < 4; k++) Ht[r][k] += (double)P.h_mu * res * std::conj(xs[k]);
        }
        // LMS alone lets H wander in the weak directions of an ill-conditioned channel (cond
        // 180-440 between q2 and q3). With QPSK one 90 deg misdecision then gets absorbed into
        // a rotated H column and the tracker never recovers; a windowed LS fit pins H to the
        // data. BPSK would need a 180 deg slip, which is why it never showed this.
        if (P.ls_win >= 16) {
            std::array<cd, 4> yd, xd;
            for (int r = 0; r < 4; r++) yd[r] = m[r] * dr;
            for (int k = 0; k < 4; k++) xd[k] = xs[k];
            lsY.push_back(yd);
            lsX.push_back(xd);
            if ((int)lsY.size() >= P.ls_win && lsY.size() % 8 == 0) {
                std::vector<std::array<cd, 4>> Yw(lsY.end() - P.ls_win, lsY.end()), Xw(lsX.end() - P.ls_win, lsX.end());
                Mat4 Hn;
                if (ls_channel(Yw, Xw, Hn)) Ht = Hn;
            }
        }
        theta.push_back((float)th);
        bnd.push_back(nb);
        {
            const double pred = tq + T;
            // a late write is a JTAG stall or burst, not timing drift: re-anchor there
            tq = nb - pred > 0.25 * T ? nb : pred + P.tim_alpha * (nb - pred);
        }
        cur[ea] = pa;
        cur[eb] = pb;
        slot_state.push_back(cur);
    }
    if (getenv("QM_TRKW")) fprintf(stderr, "---- frame  lookahead %d flips %d\n%s", la_tried, la_flip, trk.c_str());
    R.T_us = (float)(T / FS * 1e6);
    R.writes = (int)bnd.size();
    R.hdr_ok = hdr_ok;
    R.hdr = hdr;
    R.t_start = 0;
    if (!hdr_ok) {
        R.H = H;
        return true;
    }

    // ---- post-processing over the full frame with the decided states
    const int ns = (int)bnd.size() - 1;
    std::vector<std::array<cd, 4>> Yr(ns), Ym(ns), Xm(ns);
    std::vector<cd> rotj(ns);
    for (int j = 0; j < ns; j++) {
        for (int r = 0; r < 4; r++) Yr[j][r] = slot_mean(bnd[j], bnd[j + 1], r);
        for (int k = 0; k < 4; k++) Xm[j][k] = ph2x(slot_state[j][k]);
        rotj[j] = std::polar(1.0, -(double)theta[j]);
    }
    // The tracker's LMS soaks part of the LO wander into H, so its theta is not the true
    // common phase. Alternate: LS H with known states, then per-slot phase against H x_j
    // smoothed over +-6 slots (~0.25 ms, LO wander is a few kHz at most).
    Mat4 Hd = H;
    std::vector<cd> ph(ns);
    for (int it = 0; it < 3; it++) {
        for (int j = 0; j < ns; j++)
            for (int r = 0; r < 4; r++) Ym[j][r] = Yr[j][r] * rotj[j];
        if (!ls_channel(Ym, Xm, Hd)) { Hd = H; break; }
        for (int j = 0; j < ns; j++) {
            cd s = 0;
            for (int r = 0; r < 4; r++) {
                cd p = 0;
                for (int k = 0; k < 4; k++) p += Hd[r][k] * Xm[j][k];
                s += std::conj(Yr[j][r]) * p;
            }
            ph[j] = s;
        }
        cd acc = 0;
        const int hw = 6;
        for (int j = 0; j < std::min(hw, ns); j++) acc += ph[j];
        for (int j = 0; j < ns; j++) {
            if (j + hw < ns) acc += ph[j + hw];
            if (j - hw - 1 >= 0) acc -= ph[j - hw - 1];
            rotj[j] = std::abs(acc) > 0 ? acc / std::abs(acc) : 1.0;
        }
    }
    double tu = 0;
    for (int j = 0; j < ns; j++) {
        double a = -std::arg(rotj[j]);
        tu = j == 0 ? a : tu + std::remainder(a - tu, 2 * M_PI);
        theta[j] = (float)tu;
        for (int r = 0; r < 4; r++) Ym[j][r] = Yr[j][r] * rotj[j];
    }
    // Re-decide every payload pair with the timing now fixed. A pair's new state holds for two
    // slots (until the same pair is written again), and the tracker only looked at ~0.7 T of
    // the first one. Using both full slots against the frame-wide H roughly doubles the
    // energy per decision, which is what QPSK on the weak, nearly collinear columns needs.
    const int last_w = std::min(2 * (FRAME_CHIPS - 1) - 1, ns - 2);
    for (int it = 0; it < P.redecide; it++) {
        std::vector<uint8_t> A0, A1;
        for (int w = 2 * PAY_CHIP0; w <= last_w; w++) {
            const int c = w / 2, ea = 2 * (w % 2), eb = ea + 1;
            alph_for(c, ea, A0);
            alph_for(c, eb, A1);
            // slots w-1 (old pair state, known), w and w+1 (new pair state) with one free
            // common phase: the frame-wide phase track is smoothed over +-6 slots and misses
            // faster LO jitter that the tracker's step metric never sees
            cd rest[3][4];
            double wt[3], yy = 0;
            for (int s = 0; s < 3; s++) {
                const int j = w - 1 + s;
                wt[s] = std::max(1, bnd[j + 1] - bnd[j] - 2 * g);
                for (int r = 0; r < 4; r++) {
                    cd p = 0;
                    for (int k = 0; k < 4; k++)
                        if (s == 0 || (k != ea && k != eb)) p += Hd[r][k] * Xm[j][k];
                    rest[s][r] = p;
                    yy += wt[s] * std::norm(Ym[j][r]);
                }
            }
            double bc = 1e300;
            uint8_t ba = slot_state[w][ea], bb = slot_state[w][eb];
            for (uint8_t pa : A0)
                for (uint8_t pb : A1) {
                    const cd xa = ph2x(pa), xb = ph2x(pb);
                    double pp = 0;
                    cd yp = 0;
                    for (int s = 0; s < 3; s++)
                        for (int r = 0; r < 4; r++) {
                            const cd p = s == 0 ? rest[0][r] : rest[s][r] + Hd[r][ea] * xa + Hd[r][eb] * xb;
                            pp += wt[s] * std::norm(p);
                            yp += wt[s] * std::conj(p) * Ym[w - 1 + s][r];
                        }
                    const double cst = yy + pp - 2 * std::abs(yp);
                    if (cst < bc) { bc = cst; ba = pa; bb = pb; }
                }
            for (int s = 0; s < 2; s++) {
                slot_state[w + s][ea] = ba;
                slot_state[w + s][eb] = bb;
                Xm[w + s][ea] = ph2x(ba);
                Xm[w + s][eb] = ph2x(bb);
            }
        }
        Mat4 Hn;
        if (ls_channel(Ym, Xm, Hn)) Hd = Hn;
    }
    R.H = Hd;
    {
        double r = 0, t = 0;
        for (int j = 0; j < ns; j++)
            for (int i = 0; i < 4; i++) {
                cd p = 0;
                for (int k = 0; k < 4; k++) p += Hd[i][k] * Xm[j][k];
                r += std::norm(Ym[j][i] - p);
                t += std::norm(p);
            }
        R.model_snr_db = (float)(10 * std::log10(t / std::max(r, 1e-12)));
    }
    // singular values over the elements the peer has on (off ones are keyed down, their
    // column is noise and would read as a near-zero singular value)
    Mat4 G{};
    int nact = 0;
    for (int k = 0; k < 4; k++) nact += hdr.mods[k] != EM_OFF;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            if (hdr.mods[i] == EM_OFF || hdr.mods[j] == EM_OFF) continue;
            for (int k = 0; k < 4; k++) G[i][j] += std::conj(Hd[k][i]) * Hd[k][j];
        }
    double ev[4];
    hermitian_eigs(G, ev);
    for (int i = 0; i < 4; i++) R.sv[i] = i < nact ? (float)std::sqrt(ev[i]) : 0.f;
    R.cond = nact && R.sv[nact - 1] > 1e-9f ? R.sv[0] / R.sv[nact - 1] : 1e9f;

    // SIC + MRC estimate of each element per payload chip (two slots per chip)
    double hn[4];
    for (int k = 0; k < 4; k++) {
        hn[k] = 0;
        for (int r = 0; r < 4; r++) hn[k] += std::norm(Hd[r][k]);
    }
    auto est = [&](int j, int k) {
        cd s = 0;
        for (int r = 0; r < 4; r++) {
            cd y = Ym[j][r];
            for (int i = 0; i < 4; i++)
                if (i != k) y -= Hd[r][i] * Xm[j][i];
            s += std::conj(Hd[r][k]) * y;
        }
        return s / std::max(hn[k], 1e-9);
    };
    const int last_chip = std::min(FRAME_CHIPS - 2, (ns - 3) / 2);
    std::vector<std::array<cd, 4>> u(PAY_CHIPS);
    double evm[4] = {0, 0, 0, 0};
    int nev = 0;
    for (int i = 0; i < PAY_CHIPS; i++) {
        int c = PAY_CHIP0 + i;
        if (c > last_chip) break;
        for (int k = 0; k < 4; k++) {
            int j0 = k < 2 ? 2 * c : 2 * c + 1;
            int m = hdr.mode == FM_SDM ? (int)hdr.mods[k] : (int)EM_BPSK;
            cd x = 0.5 * (est(j0, k) + est(j0 + 1, k));
            x *= std::conj(ph2x(elem_phase(m, 0, c)));
            u[i][k] = x;
            int M = rot_order(m);
            double a = std::arg(x) / (2 * M_PI / M);
            cd ideal = std::polar(1.0, 2 * M_PI * std::round(a) / M);
            evm[k] += std::norm(x - ideal);
        }
        nev++;
    }
    for (int k = 0; k < 4; k++) R.snr_db[k] = nev ? (float)(-10 * std::log10(std::max(evm[k] / nev, 1e-6))) : 0;

    // payload bits
    std::vector<uint8_t> bits;
    bits.reserve(payload_bits(hdr));
    if (hdr.mode == FM_SDM) {
        int prev[4] = {0, 0, 0, 0};
        for (int i = 0; i < PAY_CHIPS; i++) {
            int c = PAY_CHIP0 + i;
            int j = 2 * c + 1;  // slot after write 2c+1 holds all four elements of chip c
            for (int k = 0; k < 4; k++) {
                int m = hdr.mods[k];
                if (m == EM_OFF) continue;
                int M = mod_order(m);
                int s = 0;
                if (j < (int)slot_state.size()) s = (((int)slot_state[j][k] - elem_phase(m, 0, c)) & 255) / (256 / M);
                int g = gray((s - prev[k] + M) % M);
                prev[k] = s;
                for (int q = mod_bits(m) - 1; q >= 0; q--) bits.push_back((g >> q) & 1);
            }
        }
    } else {
        int sf = cdm_sf(hdr.mode), nsym = PAY_CHIPS / sf;
        int prev[4] = {0, 0, 0, 0};
        for (int s = 0; s < nsym; s++)
            for (int k = 0; k < 4; k++) {
                if (hdr.mods[k] == EM_OFF) continue;
                double acc = 0;
                for (int jj = 0; jj < sf; jj++) {
                    int i = s * sf + jj, c = PAY_CHIP0 + i;
                    double sgn = (cdm_code_bit(k, sf, jj) ^ pn_bit(c)) ? -1.0 : 1.0;
                    acc += u[i][k].real() * sgn;
                }
                int b = acc < 0;
                bits.push_back(b ^ prev[k]);
                prev[k] = b;
            }
    }
    R.payload.assign(bits.size() / 8, 0);
    for (size_t i = 0; i < R.payload.size() * 8; i++) R.payload[i >> 3] |= bits[i] << (7 - (i & 7));

    // timing stats
    std::vector<float> sl(ns);
    double s1 = 0, s2 = 0, mx = 0;
    for (int j = 0; j < ns; j++) {
        sl[j] = (float)((bnd[j + 1] - bnd[j]) / FS * 1e6);
        s1 += sl[j];
        s2 += sl[j] * sl[j];
        mx = std::max(mx, (double)sl[j]);
        if (sl[j] > 1.25 * R.T_us) R.stalls++;
    }
    R.slot_mean_us = (float)(s1 / ns);
    R.slot_std_us = (float)std::sqrt(std::max(0.0, s2 / ns - R.slot_mean_us * R.slot_mean_us));
    R.slot_max_us = (float)mx;

    if (viz) {
        for (int k = 0; k < 4; k++) {
            R.constel[k].resize(nev);
            for (int i = 0; i < nev; i++) R.constel[k][i] = cf(u[i][k]);
        }
        R.slot_us = sl;
        R.theta = theta;
        R.slot_y.resize((size_t)ns * 4);
        for (int j = 0; j < ns; j++)
            for (int r = 0; r < 4; r++) R.slot_y[4 * j + r] = cf(Ym[j][r]);
        R.slot_x.assign(slot_state.begin(), slot_state.begin() + ns);
    }
    return true;
}

// ---------------------------------------------------------------- streaming

Receiver::Receiver(double f_if) : f_if_(f_if)
{
    const char *e = getenv("QM_DECODERS");
    const int nw = std::clamp(e ? atoi(e) : 3, 1, 4);
    for (int i = 0; i < nw; i++) th_.emplace_back(&Receiver::worker, this);
}

void Receiver::reset()
{
    epoch_++;
    buf_.clear();
    pk_.clear();
    drain();
    base_ = det_pos_ = 0;
    last_hit_ = -1;
    quiet_s_ = 0;
    busy_until_ = 0;
    cand_.clear();
    // sample indices restart at 0: re-measure the noise floor on the first new block, the
    // old one may include our own TX tail
    nv_upd_ = INT64_MIN / 2;
    // viz_free_t_ stays: it is wall time, and the first block of every epoch is the quiet
    // turnaround before the peer keys up, not a sign the peer is gone
    viz_next_ = 0;
}

void Receiver::push(const int8_t *raw, size_t ninst, bool viz)
{
    auto tp0 = std::chrono::steady_clock::now();
    const size_t nblk = ninst / DECIM;
    const double w = 2 * M_PI * (f_if_ + cfo_) / RX_FS;
    float w0r[DECIM], w0i[DECIM];
    for (int k = 0; k < DECIM; k++) { w0r[k] = (float)std::cos(-w * k); w0i[k] = (float)std::sin(-w * k); }
    const cd step = std::polar(1.0, -w * DECIM);
    cd p = std::polar(1.0, ph_);
    const size_t old = buf_.size();
    buf_.resize(old + 4 * nblk);
    pk_.resize(old + 4 * nblk);
    cf *out = buf_.data() + old;
    uint8_t *pko = pk_.data() + old;
    for (size_t bk = 0; bk < nblk; bk++) {
        const int8_t *x = raw + (size_t)8 * DECIM * bk;
        float acc[8];
#if defined(__aarch64__)
        float32x4_t a01 = vdupq_n_f32(0), a23 = vdupq_n_f32(0);
        int8x8_t pkv = vdup_n_s8(0);
        for (int k = 0; k < DECIM; k++) {
            const int8x8_t raw8 = vld1_s8(x + 8 * k);
            pkv = vmax_s8(pkv, vqabs_s8(raw8));
            int16x8_t v16 = vmovl_s8(raw8);
            float32x4_t lo = vcvtq_f32_s32(vmovl_s16(vget_low_s16(v16)));
            float32x4_t hi = vcvtq_f32_s32(vmovl_s16(vget_high_s16(v16)));
            float32x4_t wr = vdupq_n_f32(w0r[k]);
            const float wv[4] = {-w0i[k], w0i[k], -w0i[k], w0i[k]};
            float32x4_t wi = vld1q_f32(wv);
            a01 = vfmaq_f32(a01, lo, wr);
            a01 = vfmaq_f32(a01, vrev64q_f32(lo), wi);
            a23 = vfmaq_f32(a23, hi, wr);
            a23 = vfmaq_f32(a23, vrev64q_f32(hi), wi);
        }
        vst1q_f32(acc, a01);
        vst1q_f32(acc + 4, a23);
        int8_t pk8[8];
        vst1_s8(pk8, pkv);
        for (int c = 0; c < 4; c++) pko[4 * bk + c] = (uint8_t)std::max(pk8[2 * c], pk8[2 * c + 1]);
#else
        for (int i = 0; i < 8; i++) acc[i] = 0;
        int pkm[4] = {0, 0, 0, 0};
        for (int k = 0; k < DECIM; k++)
            for (int c = 0; c < 4; c++) {
                float I = x[8 * k + 2 * c], Q = x[8 * k + 2 * c + 1];
                acc[2 * c] += I * w0r[k] - Q * w0i[k];
                acc[2 * c + 1] += I * w0i[k] + Q * w0r[k];
                pkm[c] = std::max({pkm[c], std::abs((int)x[8 * k + 2 * c]), std::abs((int)x[8 * k + 2 * c + 1])});
            }
        for (int c = 0; c < 4; c++) pko[4 * bk + c] = (uint8_t)std::min(pkm[c], 127);
#endif
        const cf pf((float)p.real() / DECIM, (float)p.imag() / DECIM);
        for (int c = 0; c < 4; c++) out[4 * bk + c] = cf(acc[2 * c], acc[2 * c + 1]) * pf;
        p *= step;
    }
    ph_ = std::arg(p);
    if (const int lanes = params().lanes; (lanes & 0xF) != 0xF)
        for (size_t bk = 0; bk < nblk; bk++)
            for (int c = 0; c < 4; c++)
                if (!(lanes >> c & 1)) { out[4 * bk + c] = 0; pko[4 * bk + c] = 0; }
    {
        uint32_t nc = 0;
        for (size_t bk = 0; bk < nblk; bk++)
            nc += std::max({pko[4 * bk], pko[4 * bk + 1], pko[4 * bk + 2], pko[4 * bk + 3]}) >= CLIP_PK;
        clip_blk_ += nc;
        all_blk_ += (uint32_t)nblk;
    }

    // Noise floor per channel from first differences (robust to the write steps), taken
    // before the channel filter: differences of the low-passed stream measure the tone's own
    // CFO slope and the slow phase wander, not the noise.
    {
        const int64_t nend = base_ + (int64_t)(buf_.size() / 4);
        const int nn = (int)std::min<size_t>(nblk, 2048);
        if (nend - nv_upd_ > 8192 && nn >= 1024) {
            nv_upd_ = nend;
            const cf *z = out + 4 * (nblk - nn);
            std::vector<float> d(nn - 1);
            float tot = 0;
            for (int c = 0; c < 4; c++) {
                for (int i = 0; i + 1 < nn; i++) d[i] = std::abs(z[4 * (i + 1) + c] - z[4 * i + c]);
                std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
                float med = d[d.size() / 2] / 0.8326f;
                nv_[c] = med * med / 2;
                tot += nv_[c];
            }
            nv_tot_ = tot;
        }
    }

    const int lpf = params().lpf;
    if (lpf != lf_) {
        lf_ = lpf;
        li1_ = li2_ = 0;
        for (int c = 0; c < 4; c++) ls1_[c] = ls2_[c] = 0;
        lh1_.assign((size_t)std::max(lpf, 1) * 4, cf(0));
        lh2_.assign((size_t)std::max(lpf + 1, 1) * 4, cf(0));
    }
    if (lpf > 0) {
        // the sums run in double and subtract exactly the float values they added, so they
        // do not drift over hours of samples
        const double g1 = 1.0 / lpf, g2 = 1.0 / (lpf + 1);
        for (size_t bk = 0; bk < nblk; bk++) {
            cf *o = out + 4 * bk;
            cf *h1 = lh1_.data() + 4 * li1_, *h2 = lh2_.data() + 4 * li2_;
            for (int c = 0; c < 4; c++) {
                ls1_[c] += cd(o[c]) - cd(h1[c]);
                h1[c] = o[c];
                const cf y1 = cf(ls1_[c] * g1);
                ls2_[c] += cd(y1) - cd(h2[c]);
                h2[c] = y1;
                o[c] = cf(ls2_[c] * g2);
            }
            if (++li1_ == lpf) li1_ = 0;
            if (++li2_ == lpf + 1) li2_ = 0;
        }
    }
    auto tp1 = std::chrono::steady_clock::now();

    // Stage snapshots trigger like a scope in normal mode: only when the DDC output holds a
    // signal well above the noise, so half-duplex gaps (the peer is silent about half the
    // time) do not show up as frames of pure noise. Free-runs after 1 s without signal.
    const int64_t vend = base_ + (int64_t)(buf_.size() / 4);
    bool vtrig = false;
    if (viz && vend >= viz_next_) {
        const size_t nb = std::min<size_t>(nblk, 4096);
        float mx = 0, nmx = 0;
        for (size_t i = nblk - nb; i < nblk; i++)
            for (int c = 0; c < 4; c++) mx = std::max(mx, std::norm(out[4 * i + c]));
        for (int c = 0; c < 4; c++) nmx = std::max(nmx, nv_[c]);
        vtrig = mx > 40 * nmx && mx > 4;
    }
    if (viz && vend >= viz_next_ && (vtrig || wall_s() >= viz_free_t_)) {
        std::lock_guard<std::mutex> l(viz_m_);
        size_t nr = std::min<size_t>(ninst, 8192);
        // same end of the chunk as the DDC window below
        viz_.raw.assign(raw + 8 * (ninst - nr), raw + 8 * ninst);
        viz_.trig = vtrig;
        viz_.lanes = params().lanes;
        viz_free_t_ = wall_s() + 1.0;
        const int8_t *rp = raw + 8 * (ninst - nr);
        for (int c = 0; c < 4; c++) {
            double s = 0, pk = 0;
            for (size_t i = 0; i < nr; i++)
                for (int q = 0; q < 2; q++) {
                    double v = rp[8 * i + 2 * c + q];
                    s += v * v;
                    pk = std::max(pk, std::fabs(v));
                }
            viz_.raw_rms[c] = (float)std::sqrt(s / (2 * nr));
            viz_.raw_peak[c] = (float)pk;
        }
        const size_t nbb = buf_.size() / 4;
        const size_t nb = std::min<size_t>(nbb, 4096);
        viz_.bb.assign(buf_.data() + 4 * (nbb - nb), buf_.data() + 4 * nbb);
        viz_.nco_hz = f_if_ + cfo_;
        viz_fresh_ = true;
        viz_next_ = base_ + (int64_t)(buf_.size() / 4) + (int64_t)(0.1 * FS);
    }

    const int64_t end = base_ + (int64_t)(buf_.size() / 4);

    auto tp2 = std::chrono::steady_clock::now();
    detect(det_pos_);
    auto tp3 = std::chrono::steady_clock::now();
    try_decode(viz);
    auto tp4 = std::chrono::steady_clock::now();
    using dsec = std::chrono::duration<double>;
    prof_[0] += dsec(tp1 - tp0).count();
    prof_[1] += dsec(tp2 - tp1).count();
    prof_[2] += dsec(tp3 - tp2).count();
    prof_[3] += dsec(tp4 - tp3).count();

    // keep ~3 frames of history behind the oldest pending candidate
    int64_t keep_from = end - (int64_t)(0.07 * FS);
    if (!cand_.empty()) keep_from = std::min(keep_from, cand_.front().first - 64);
    keep_from = std::min(keep_from, det_pos_ - 64);
    if (keep_from > base_ + (int64_t)(0.02 * FS)) {
        size_t drop = (size_t)(keep_from - base_);
        buf_.erase(buf_.begin(), buf_.begin() + 4 * drop);
        pk_.erase(pk_.begin(), pk_.begin() + 4 * drop);
        base_ = keep_from;
    }
}

void Receiver::detect(int64_t from)
{
    RxParams P = params();
    const int W = 8;
    const int64_t end = base_ + (int64_t)(buf_.size() / 4);
    const float thr = P.det_k * nv_tot_ * 2.0f / W;
    const int64_t q = (int64_t)(P.gap_us * 1e-6 * FS);
    int64_t n = std::max(from, base_ + W);
    if (last_hit_ < 0) last_hit_ = n;  // stream start counts as activity
    // running W-sample sums before / after n, in units of W x mean
    cf pre[4] = {}, post[4] = {};
    if (n + W < end) {
        const cf *z = buf_.data() + 4 * (n - base_);
        for (int k = 1; k <= W; k++)
            for (int c = 0; c < 4; c++) pre[c] += z[-4 * k + c];
        for (int k = 0; k < W; k++)
            for (int c = 0; c < 4; c++) post[c] += z[4 * k + c];
    }
    const float thr_s = thr * W * W;
    for (; n + W < end; n++) {
        const cf *z = buf_.data() + 4 * (n - base_);
        float pp = 0;
        cf x = 0;
        for (int c = 0; c < 4; c++) {
            pp += std::norm(pre[c]) + std::norm(post[c]);
            x += std::conj(pre[c]) * post[c];
        }
        float e = pp - 2 * std::abs(x);
        for (int c = 0; c < 4; c++) {
            pre[c] += z[c] - z[-4 * W + c];
            post[c] += z[4 * W + c] - z[c];
        }
        if (e > thr_s && e > P.det_rel * pp) {
            if (last_hit_ >= 0 && n - last_hit_ > q && n >= busy_until_.load()) {
                cand_.emplace_back(last_hit_ + 4, n);
                det_frames++;
            }
            last_hit_ = n;
        }
    }
    det_pos_ = n;
    quiet_s_ = (double)(n - last_hit_) / FS;
}

void Receiver::seed_periods(const RxParams &P, std::vector<float> &out) const
{
    out.clear();
    auto add = [&](float t) {
        if (t <= 0) return;
        for (float u : out)
            if (std::fabs(t - u) < 0.04f * u) return;
        out.push_back(t);
    };
    add(peer_T_us_ > 0 ? peer_T_us_.load() : P.t0_us);
    add(peer_next_us_);
    // nothing decoded for 2 s: the peer may be at any rate (it restarted, or we did)
    if (wall_s() - last_ok_wall_ > 2.0)
        for (float t : RATE_TABLE_US) add(t);
}

void Receiver::try_decode(bool viz)
{
    RxParams P = params();
    const int64_t end = base_ + (int64_t)(buf_.size() / 4);
    std::vector<float> seeds;
    seed_periods(P, seeds);
    const float t_hi = *std::max_element(seeds.begin(), seeds.end());
    const float t_lo = *std::min_element(seeds.begin(), seeds.end());
    // wait long enough for the slowest candidate; the quiet check ends the wait early
    const int64_t need = (int64_t)(1.15 * NWRITES * t_hi * 1e-6 * FS) + 256;
    const int64_t q = (int64_t)(P.gap_us * 1e-6 * FS);
    while (!cand_.empty()) {
        auto [a, b] = cand_.front();
        if (b < busy_until_.load() || a < base_) { cand_.pop_front(); continue; }
        // ready once a full stretched frame is buffered, or once the frame has gone quiet
        const bool ended = last_hit_ > b + (int64_t)(0.93 * NWRITES * t_lo * 1e-6 * FS) && det_pos_ - last_hit_ > q;
        if (end < b + need && !ended) break;
        cand_.pop_front();
        Job j;
        const int64_t n = std::min<int64_t>(end - a, b - a + need);
        j.z.assign(buf_.begin() + 4 * (a - base_), buf_.begin() + 4 * (a - base_ + n));
        j.a = a;
        j.b = b;
        j.nco = cfo_.load();
        j.epoch = epoch_.load();
        std::copy(nv_, nv_ + 4, j.nv);
        {
            // ADC level over the frame: p99 of the per-block peaks, per lane
            const int64_t f0 = b - base_, f1 = std::min<int64_t>(a - base_ + n, f0 + (int64_t)(NWRITES * seeds[0] * 1e-6 * FS));
            std::vector<uint8_t> v;
            v.reserve((size_t)std::max<int64_t>(f1 - f0, 0));
            for (int c = 0; c < 4; c++) {
                v.clear();
                for (int64_t i = f0; i < f1; i++) v.push_back(pk_[4 * i + c]);
                if (v.empty()) { j.adc[c] = 0; continue; }
                const size_t k = std::min(v.size() - 1, (size_t)(0.99 * v.size()));
                std::nth_element(v.begin(), v.begin() + k, v.end());
                j.adc[c] = v[k];
            }
        }
        j.P = P;
        j.viz = viz;
        if (viz) {
            // detector output around the frame start, same statistic as detect()
            const int W = 8;
            const int64_t i0 = std::max<int64_t>(a - 64, base_ + W), i1 = std::min<int64_t>(b + 900, end - W);
            j.step_b = (int)(b - i0);
            for (int64_t m = i0; m < i1; m++) {
                const cf *z = buf_.data() + 4 * (m - base_);
                cf pre[4] = {}, post[4] = {};
                for (int kk = 1; kk <= W; kk++)
                    for (int c = 0; c < 4; c++) pre[c] += z[-4 * kk + c];
                for (int kk = 0; kk < W; kk++)
                    for (int c = 0; c < 4; c++) post[c] += z[4 * kk + c];
                float pp = 0;
                cf x = 0;
                for (int c = 0; c < 4; c++) {
                    pre[c] /= (float)W;
                    post[c] /= (float)W;
                    pp += std::norm(pre[c]) + std::norm(post[c]);
                    x += std::conj(pre[c]) * post[c];
                }
                j.step_e.push_back(pp - 2 * std::abs(x));
            }
            j.step_thr = P.det_k * nv_tot_ * 2.0f / W;
        }
        std::unique_lock<std::mutex> l(jm_);
        if (offline) jcv_.wait(l, [&] { return jobs_.size() < 6; });
        // decoder overloaded: the oldest frame is the least useful
        if (jobs_.size() >= 6) { jobs_.pop_front(); dec_drop++; }
        jobs_.push_back(std::move(j));
        jcv_.notify_one();
    }
}

void Receiver::worker()
{
    for (;;) {
        Job j;
        {
            std::unique_lock<std::mutex> l(jm_);
            jcv_.wait(l, [&] { return stop_ || (!hold_ && !jobs_.empty()); });
            if (stop_) return;
            j = std::move(jobs_.front());
            jobs_.pop_front();
            busy_++;
        }
        if (j.b >= busy_until_.load()) run_job(j);
        std::lock_guard<std::mutex> l(jm_);
        busy_--;
        jcv_.notify_all();
    }
}

void Receiver::hold(bool on, int max_ms)
{
    std::unique_lock<std::mutex> l(jm_);
    hold_ = on;
    if (on && max_ms > 0) jcv_.wait_for(l, std::chrono::milliseconds(max_ms), [&] { return busy_ == 0; });
    if (!on) jcv_.notify_all();
}

void Receiver::run_job(Job &j)
{
    FrameResult R;
    const int N = (int)(j.z.size() / 4);
    // Seed with the most likely period, and only on a failed header try the others: a
    // rate change shows up as one frame decoded on the second seed, a false start costs a
    // preamble search per seed (the preamble gate returns before the tracker).
    std::vector<float> seeds;
    seed_periods(j.P, seeds);
    bool ok = false;
    for (size_t k = 0; k < seeds.size(); k++) {
        RxParams P = j.P;
        P.t0_us = seeds[k];
        FrameResult Rk;
        const bool okk = decode_frame(j.z.data(), N, 0, (int)(j.b - j.a), P, j.nv, j.viz, Rk);
        if (k == 0 || (okk && Rk.hdr_ok) || (okk && !ok)) { R = std::move(Rk); ok = okk; }
        if (ok && R.hdr_ok) break;
    }
    static const bool dbg = getenv("QM_DEBUG") != nullptr;
    if (dbg) fprintf(stderr, "dec: a %ld b %ld (%.4f s) N %d ok %d hdr %d\n", (long)j.a, (long)j.b, j.b / FS, N, ok,
                     R.hdr_ok);
    if (!ok) { dec_fail++; return; }
    R.t_start = j.b / FS;
    R.epoch = j.epoch;
    std::copy(j.adc, j.adc + 4, R.adc_lane);
    R.adc_level = *std::max_element(j.adc, j.adc + 4);
    R.cfo_hz = (float)(j.nco + R.f_res);
    // The 16-tap boxcar is only -0.3 dB at 20 kHz offset and the decoder removes the
    // residual per frame, so the NCO is re-tuned only for gross offsets (unit-to-unit CFO
    // here is ~16 kHz). Any re-tune lands as a frequency step inside frames that are already
    // buffered, and with several jobs in flight an incremental update overshoots, so the
    // new value is absolute: this frame's own NCO plus its residual. Only CRC-valid headers
    // may move it; a false start on our own TX tail can show a strong tone tens of kHz off.
    if (R.hdr_ok && R.tone_snr_db > 10 && std::fabs(R.f_res) > 40e3 && std::fabs(R.f_res) < 250e3)
        cfo_.store(j.nco + j.P.cfo_alpha * R.f_res);
    if (R.hdr_ok && dec_ok % 4 == 0) {
        const int64_t i0 = j.b - j.a, nf = std::min<int64_t>(N - i0, (int64_t)(NWRITES * R.T_us * 1e-6 * FS));
        if (nf > PSD_N) {
            std::vector<cf> y((size_t)nf * 4);
            const double w = -2 * M_PI * R.f_res / FS;
            for (int64_t i = 0; i < nf; i++) {
                const cf r = std::polar(1.0f, (float)(w * i));
                for (int c = 0; c < 4; c++) y[4 * i + c] = j.z[4 * (i0 + i) + c] * r;
            }
            int segs = 0;
            for (int c = 0; c < 4; c++) segs += welch_acc(y.data() + c, (size_t)nf, 4, PSD_N, R.psd);
            const float norm = 1.0f / (segs * (float)PSD_N * (3.0f * PSD_N / 8));
            for (auto &v : R.psd) v *= norm;
        }
    }
    if (R.hdr_ok) {
        dec_ok++;
        busy_until_.store(j.b + (int64_t)(0.9 * NWRITES * R.T_us * 1e-6 * FS));
        peer_T_us_ = R.T_us;
        last_ok_wall_ = wall_s();
    } else {
        dec_fail++;
    }
    if (j.viz) {
        R.step_e = std::move(j.step_e);
        R.step_thr = j.step_thr;
        R.step_b = j.step_b;
    }
    std::lock_guard<std::mutex> l(out_m_);
    out_.push_back(std::move(R));
    if (out_.size() > 16) out_.pop_front();
}

void Receiver::drain()
{
    std::unique_lock<std::mutex> l(jm_);
    jcv_.wait(l, [&] { return jobs_.empty() && busy_ == 0; });
}

Receiver::~Receiver()
{
    {
        std::lock_guard<std::mutex> l(jm_);
        stop_ = true;
    }
    jcv_.notify_all();
    for (auto &t : th_)
        if (t.joinable()) t.join();
}

bool Receiver::pop(FrameResult &o)
{
    std::lock_guard<std::mutex> l(out_m_);
    if (out_.empty()) return false;
    o = std::move(out_.front());
    out_.pop_front();
    return true;
}

bool Receiver::viz_raw(VizRaw &v)
{
    std::lock_guard<std::mutex> l(viz_m_);
    if (!viz_fresh_) return false;
    v = viz_;
    viz_fresh_ = false;
    return true;
}

}  // namespace qm
