// Phase-coded MIMO frame: one JTAG batch of 16 lead-in writes + 504 chips = 1024
// register writes (the fpga-csi batch ioctl rejects count > 1024).
//
// The lead-in rewrites the held tail state, so RF does not move. On q3 the first
// writes of a batch come out irregular, and without the lead-in they landed on the
// preamble.
//
// Chip c writes 0x2C (el0 hi byte, el1 lo byte) then 0x2D (el2, el3), so the
// array sees write 2c commit el0/el1 and write 2c+1 commit el2/el3. Phases are
// 8 bit, 256 = 360 deg.
//
//   chips   0..23   preamble: chip 0 flips all four; after that one element of
//                   each pair flips 180 deg and the other steps +90 deg, so a write
//                   still steps when one element of its pair is faded
//   chips  24..47   header: BPSK, same bit on all four elements
//   chips  48..502  payload: per-element off/BPSK/QPSK/8PSK (SDM) or
//                   Walsh x PN spread BPSK (CDM)
//   chip   503      tail: all phases 0, held through the inter-frame gap
//
// Data chips carry a pi/M rotation per chip so every write moves its elements.
#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace qm {

constexpr int FRAME_CHIPS = 504;
constexpr int PRE_CHIPS = 24;
constexpr int HDR_CHIPS = 24;
constexpr int PAY_CHIP0 = PRE_CHIPS + HDR_CHIPS;
constexpr int PAY_CHIPS = FRAME_CHIPS - 1 - PAY_CHIP0;  // 455
constexpr int NWRITES = 2 * FRAME_CHIPS;
constexpr int LEAD_WRITES = 1024 - NWRITES;
constexpr int BLK_BYTES = 24;
constexpr int BLK_DATA = 20;

enum ElemMod : uint8_t { EM_OFF = 0, EM_BPSK = 1, EM_QPSK = 2, EM_8PSK = 3 };
enum FrameMode : uint8_t { FM_SDM = 0, FM_CDM4 = 1, FM_CDM8 = 2 };
enum BlockType : uint8_t { BT_FILL = 0, BT_CHAT = 1, BT_ACK = 2, BT_FEEDBACK = 3, BT_ID = 5 };

inline int mod_bits(int m) { return m; }          // off 0, BPSK 1, QPSK 2, 8PSK 3
inline int mod_order(int m) { return 1 << m; }    // alphabet size, off = 1
inline int rot_order(int m) { return m == EM_OFF ? 2 : 1 << m; }

struct Header {
    uint8_t mode = FM_SDM;
    std::array<uint8_t, 4> mods{{EM_QPSK, EM_QPSK, EM_QPSK, EM_QPSK}};
    bool last = false;
    uint8_t seq = 0;  // 4 bits
    uint32_t pack() const;           // 24 bits incl. CRC-8
    static bool unpack(uint32_t w, Header &h);
};

int cdm_sf(int mode);
int payload_bits(const Header &h);
inline int payload_blocks(const Header &h) { return payload_bits(h) / 8 / BLK_BYTES; }

// Element phase byte for symbol s of element modulation m at chip c.
inline uint8_t elem_phase(int m, int s, int c)
{
    int M = rot_order(m);
    return (uint8_t)(((256 / M) * s + (128 / M) * (c + 1)) & 0xFF);
}

const std::array<std::array<uint8_t, 4>, PRE_CHIPS> &preamble_states();
uint8_t cdm_code_bit(int el, int sf, int j);      // Walsh row for element
uint8_t pn_bit(int c);                            // chip scrambler shared by all elements

// payload bytes -> per-chip element phases [FRAME_CHIPS][4]
void build_frame(const Header &h, const std::vector<uint8_t> &payload,
                 std::vector<std::array<uint8_t, 4>> &out);

inline int gray(int s) { return s ^ (s >> 1); }
inline int gray_inv(int g) { int s = 0; for (; g; g >>= 1) s ^= g; return s; }

uint8_t crc8(const uint8_t *d, size_t n);
uint16_t crc16(const uint8_t *d, size_t n);

// 24-byte block: [type][len][data 20][crc16]
void make_block(uint8_t type, const uint8_t *data, int len, uint8_t *out);
bool parse_block(const uint8_t *in, uint8_t &type, int &len, const uint8_t *&data);
void fill_prbs(uint8_t seq, int blk, uint8_t *out24);

}  // namespace qm
