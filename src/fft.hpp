// In-place radix-2 complex FFT, for viz and occupied-bandwidth estimates only.
#pragma once
#include <cmath>
#include <complex>
#include <utility>
#include <vector>

namespace qm {

inline void fft_inplace(std::vector<std::complex<float>> &a)
{
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = -2 * M_PI / (double)len;
        const std::complex<float> wl((float)std::cos(ang), (float)std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            std::complex<float> w(1, 0);
            for (size_t k = 0; k < len / 2; k++) {
                auto u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

// Welch PSD with a Hann window and 50 % overlap, fftshifted (bin n/2 = DC).
// x is strided: sample i is x[i * stride]. Output accumulates into psd (size n).
inline int welch_acc(const std::complex<float> *x, size_t len, size_t stride, size_t n, std::vector<float> &psd)
{
    psd.resize(n, 0.0f);
    std::vector<std::complex<float>> b(n);
    std::vector<float> w(n);
    for (size_t i = 0; i < n; i++) w[i] = 0.5f - 0.5f * (float)std::cos(2 * M_PI * i / n);
    int segs = 0;
    for (size_t s = 0; s + n <= len; s += n / 2, segs++) {
        for (size_t i = 0; i < n; i++) b[i] = x[(s + i) * stride] * w[i];
        fft_inplace(b);
        for (size_t i = 0; i < n; i++) psd[(i + n / 2) % n] += std::norm(b[i]);
    }
    return segs;
}

}  // namespace qm
