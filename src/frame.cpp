#include "frame.hpp"

#include <cstring>

namespace qm {

uint8_t crc8(const uint8_t *d, size_t n)
{
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++) c = (uint8_t)(c & 0x80 ? (c << 1) ^ 0x07 : c << 1);
    }
    return c;
}

uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) c = (uint16_t)(c & 0x8000 ? (c << 1) ^ 0x1021 : c << 1);
    }
    return c;
}

uint32_t Header::pack() const
{
    uint32_t w = (mode & 3u);
    for (int k = 0; k < 4; k++) w |= (uint32_t)(mods[k] & 3u) << (2 + 2 * k);
    w |= (last ? 1u : 0u) << 10;
    w |= (uint32_t)(seq & 0xF) << 12;
    uint8_t b[2] = {(uint8_t)(w & 0xFF), (uint8_t)(w >> 8)};
    return w | (uint32_t)crc8(b, 2) << 16;
}

bool Header::unpack(uint32_t w, Header &h)
{
    uint8_t b[2] = {(uint8_t)(w & 0xFF), (uint8_t)((w >> 8) & 0xFF)};
    if (crc8(b, 2) != ((w >> 16) & 0xFF)) return false;
    h.mode = w & 3;
    if (h.mode > FM_CDM8) return false;
    for (int k = 0; k < 4; k++) h.mods[k] = (w >> (2 + 2 * k)) & 3;
    h.last = (w >> 10) & 1;
    h.seq = (w >> 12) & 0xF;
    return true;
}

int cdm_sf(int mode) { return mode == FM_CDM4 ? 4 : mode == FM_CDM8 ? 8 : 1; }

int payload_bits(const Header &h)
{
    if (h.mode == FM_SDM) {
        int b = 0;
        for (int k = 0; k < 4; k++) b += mod_bits(h.mods[k]);
        return b * PAY_CHIPS;
    }
    int nd = 0;
    for (int k = 0; k < 4; k++) nd += h.mods[k] != EM_OFF;
    return nd * (PAY_CHIPS / cdm_sf(h.mode));
}

const std::array<std::array<uint8_t, 4>, PRE_CHIPS> &preamble_states()
{
    // Which element of each pair flips per chip; fixed PN (xorshift, seed 11).
    static std::array<std::array<uint8_t, 4>, PRE_CHIPS> st = [] {
        std::array<std::array<uint8_t, 4>, PRE_CHIPS> s{};
        uint32_t x = 0x2545F491u ^ 11u;
        std::array<uint8_t, 4> cur{{0, 0, 0, 0}};
        // Chip 0 flips both elements of each pair: the frame-start detector keys on write 0,
        // and a single flip of a weak element there would slip the whole preamble by a write.
        s[0] = cur = {{128, 128, 128, 128}};
        // Later chips: one element of each pair flips 180 deg and the other steps +90, so a
        // write still makes a step when either element of its pair is faded (q3 el3 arrives
        // 21 dB down at q2; with single flips half the 0x2D boundaries vanished).
        for (int c = 1; c < PRE_CHIPS; c++) {
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            const int a = (x >> 7) & 1, b = 2 + ((x >> 19) & 1);
            cur[a] ^= 128;
            cur[a ^ 1] += 64;
            cur[b] ^= 128;
            cur[b ^ 1] += 64;
            s[c] = cur;
        }
        return s;
    }();
    return st;
}

uint8_t pn_bit(int c)
{
    uint32_t x = 0x9E3779B9u * (uint32_t)(c + 1);
    x ^= x >> 15; x *= 0x85EBCA6Bu; x ^= x >> 13;
    return (x >> 11) & 1;
}

uint8_t cdm_code_bit(int el, int sf, int j)
{
    // Walsh rows: element k uses row r_k of the SF x SF Sylvester Hadamard matrix.
    static const int rows4[4] = {0, 1, 2, 3}, rows8[4] = {1, 2, 4, 7};
    int r = sf == 4 ? rows4[el] : rows8[el];
    return (uint8_t)(__builtin_popcount(r & j) & 1);
}

void build_frame(const Header &h, const std::vector<uint8_t> &payload,
                 std::vector<std::array<uint8_t, 4>> &out)
{
    out.assign(FRAME_CHIPS, {{0, 0, 0, 0}});
    const auto &pre = preamble_states();
    for (int c = 0; c < PRE_CHIPS; c++) out[c] = pre[c];

    uint32_t hw = h.pack();
    for (int i = 0; i < HDR_CHIPS; i++) {
        int c = PRE_CHIPS + i;
        int b = (hw >> i) & 1;
        for (int k = 0; k < 4; k++) out[c][k] = elem_phase(EM_BPSK, b, c);
    }

    size_t nbits = payload.size() * 8, bi = 0;
    auto take = [&](int n) {
        int v = 0;
        for (int i = 0; i < n; i++) {
            int b = bi < nbits ? (payload[bi >> 3] >> (7 - (bi & 7))) & 1 : 0;
            v = (v << 1) | b;
            bi++;
        }
        return v;
    };

    // Differential along chips per element: a common-phase slip of 2 pi/M maps every
    // valid state onto another valid state, so absolute symbols would fail in bursts.
    if (h.mode == FM_SDM) {
        int acc[4] = {0, 0, 0, 0};
        for (int i = 0; i < PAY_CHIPS; i++) {
            int c = PAY_CHIP0 + i;
            for (int k = 0; k < 4; k++) {
                int m = h.mods[k];
                if (m != EM_OFF) acc[k] = (acc[k] + gray_inv(take(mod_bits(m)))) % mod_order(m);
                out[c][k] = elem_phase(m, m == EM_OFF ? 0 : acc[k], c);
            }
        }
    } else {
        int sf = cdm_sf(h.mode), nsym = PAY_CHIPS / sf;
        for (int i = 0; i < PAY_CHIPS; i++) {
            int c = PAY_CHIP0 + i;
            for (int k = 0; k < 4; k++) out[c][k] = elem_phase(EM_BPSK, 0, c);
        }
        int acc[4] = {0, 0, 0, 0};
        for (int s = 0; s < nsym; s++) {
            for (int k = 0; k < 4; k++) {
                if (h.mods[k] == EM_OFF) continue;
                int d = acc[k] ^= take(1);
                for (int j = 0; j < sf; j++) {
                    int c = PAY_CHIP0 + s * sf + j;
                    int b = d ^ cdm_code_bit(k, sf, j) ^ pn_bit(c);
                    out[c][k] = elem_phase(EM_BPSK, b, c);
                }
            }
        }
    }
    out[FRAME_CHIPS - 1] = {{0, 0, 0, 0}};
}

void make_block(uint8_t type, const uint8_t *data, int len, uint8_t *out)
{
    memset(out, 0, BLK_BYTES);
    if (len > BLK_DATA) len = BLK_DATA;
    out[0] = type;
    out[1] = (uint8_t)len;
    if (len > 0) memcpy(out + 2, data, len);
    uint16_t c = crc16(out, BLK_BYTES - 2);
    out[BLK_BYTES - 2] = (uint8_t)(c >> 8);
    out[BLK_BYTES - 1] = (uint8_t)c;
}

bool parse_block(const uint8_t *in, uint8_t &type, int &len, const uint8_t *&data)
{
    uint16_t c = crc16(in, BLK_BYTES - 2);
    if (in[BLK_BYTES - 2] != (uint8_t)(c >> 8) || in[BLK_BYTES - 1] != (uint8_t)c) return false;
    type = in[0];
    len = in[1];
    if (len > BLK_DATA) return false;
    data = in + 2;
    return true;
}

void fill_prbs(uint8_t seq, int blk, uint8_t *out)
{
    uint32_t x = 0xA5A5F00Du ^ ((uint32_t)seq << 8) ^ (uint32_t)blk * 0x9E3779B9u;
    if (!x) x = 1;
    uint8_t d[BLK_DATA];
    for (int i = 0; i < BLK_DATA; i++) {
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        d[i] = (uint8_t)(x >> 24);
    }
    make_block(BT_FILL, d, BLK_DATA, out);
}

}  // namespace qm
