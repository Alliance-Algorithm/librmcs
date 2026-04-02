#pragma once

/*********************************************************************
* Filename:   sha256.hpp
* Original:   Brad Conte (brad AT bradconte.com)
*             https://github.com/B-Con/crypto-algorithms (public domain)
* Modified:   Refactored to header-only C++ with namespace, std types,
*             and named helper functions.
* Disclaimer: This code is presented "as is" without any guarantees.
* Details:    Header-only adaptation of the SHA-256 hashing algorithm.
              SHA-256 is one of the three algorithms in the SHA2
              specification. The others, SHA-384 and SHA-512, are not
              offered in this implementation.
              Algorithm specification can be found here:
               * http://csrc.nist.gov/publications/fips/fips180-2/fips180-2withchangenotice.pdf
              This implementation uses big-endian (network) byte order:
              sha256_transform reads message words as big-endian, and
              sha256_final stores the bit length and digest in big-endian
              order per the SHA-256 specification.
*********************************************************************/

#include <array>
#include <cstddef>
#include <cstdint>

namespace librmcs::firmware::crypto {

inline constexpr size_t kSha256DigestSize = 32U;

struct Sha256Ctx {
    std::array<uint8_t, 64> data{};
    uint32_t datalen = 0U;
    uint64_t bitlen = 0U;
    std::array<uint32_t, 8> state{};
};

namespace detail {

inline constexpr uint32_t kChunkBytes = 64U;
inline constexpr uint32_t kPaddingBytes = 56U;
inline constexpr uint32_t kBitLengthIncrement = 512U;

inline constexpr std::array<uint32_t, 64> kRoundConstants = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U,
    0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU,
    0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU,
    0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
    0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U,
    0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U,
    0xc67178f2U,
};

inline uint32_t rotate_right(uint32_t value, uint32_t bits) {
    return (value >> bits) | (value << (32U - bits));
}

inline uint32_t choose(uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ ((~x) & z); }

inline uint32_t majority(uint32_t x, uint32_t y, uint32_t z) { return (x & y) ^ (x & z) ^ (y & z); }

inline uint32_t big_sigma0(uint32_t x) {
    return rotate_right(x, 2U) ^ rotate_right(x, 13U) ^ rotate_right(x, 22U);
}

inline uint32_t big_sigma1(uint32_t x) {
    return rotate_right(x, 6U) ^ rotate_right(x, 11U) ^ rotate_right(x, 25U);
}

inline uint32_t small_sigma0(uint32_t x) {
    return rotate_right(x, 7U) ^ rotate_right(x, 18U) ^ (x >> 3U);
}

inline uint32_t small_sigma1(uint32_t x) {
    return rotate_right(x, 17U) ^ rotate_right(x, 19U) ^ (x >> 10U);
}

inline void sha256_transform(Sha256Ctx* ctx, const uint8_t* data) {
    std::array<uint32_t, 64> message_schedule;

    for (uint32_t index = 0U; index < 16U; ++index) {
        const uint32_t data_offset = index * 4U;
        message_schedule[index] = (static_cast<uint32_t>(data[data_offset]) << 24U)
                                | (static_cast<uint32_t>(data[data_offset + 1U]) << 16U)
                                | (static_cast<uint32_t>(data[data_offset + 2U]) << 8U)
                                | static_cast<uint32_t>(data[data_offset + 3U]);
    }

    for (uint32_t index = 16U; index < 64U; ++index) {
        message_schedule[index] =
            small_sigma1(message_schedule[index - 2U]) + message_schedule[index - 7U]
            + small_sigma0(message_schedule[index - 15U]) + message_schedule[index - 16U];
    }

    uint32_t a = ctx->state[0];
    uint32_t b = ctx->state[1];
    uint32_t c = ctx->state[2];
    uint32_t d = ctx->state[3];
    uint32_t e = ctx->state[4];
    uint32_t f = ctx->state[5];
    uint32_t g = ctx->state[6];
    uint32_t h = ctx->state[7];

    for (uint32_t index = 0U; index < 64U; ++index) {
        const uint32_t temporary_1 =
            h + big_sigma1(e) + choose(e, f, g) + kRoundConstants[index] + message_schedule[index];
        const uint32_t temporary_2 = big_sigma0(a) + majority(a, b, c);
        h = g;
        g = f;
        f = e;
        e = d + temporary_1;
        d = c;
        c = b;
        b = a;
        a = temporary_1 + temporary_2;
    }

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
    ctx->state[4] += e;
    ctx->state[5] += f;
    ctx->state[6] += g;
    ctx->state[7] += h;
}

} // namespace detail

inline void sha256_init(Sha256Ctx* ctx) {
    ctx->datalen = 0U;
    ctx->bitlen = 0U;
    ctx->state[0] = 0x6a09e667U;
    ctx->state[1] = 0xbb67ae85U;
    ctx->state[2] = 0x3c6ef372U;
    ctx->state[3] = 0xa54ff53aU;
    ctx->state[4] = 0x510e527fU;
    ctx->state[5] = 0x9b05688cU;
    ctx->state[6] = 0x1f83d9abU;
    ctx->state[7] = 0x5be0cd19U;
}

inline void sha256_update(Sha256Ctx* ctx, const uint8_t* data, size_t len) {
    for (size_t index = 0U; index < len; ++index) {
        ctx->data[ctx->datalen] = data[index];
        ++ctx->datalen;

        if (ctx->datalen == detail::kChunkBytes) {
            detail::sha256_transform(ctx, ctx->data.data());
            ctx->bitlen += detail::kBitLengthIncrement;
            ctx->datalen = 0U;
        }
    }
}

inline void sha256_final(Sha256Ctx* ctx, uint8_t* hash) {
    uint32_t data_index = ctx->datalen;

    if (ctx->datalen < detail::kPaddingBytes) {
        ctx->data[data_index++] = 0x80U;
        while (data_index < detail::kPaddingBytes) {
            ctx->data[data_index++] = 0x00U;
        }
    } else {
        ctx->data[data_index++] = 0x80U;
        while (data_index < detail::kChunkBytes) {
            ctx->data[data_index++] = 0x00U;
        }
        detail::sha256_transform(ctx, ctx->data.data());

        for (uint32_t index = 0U; index < detail::kPaddingBytes; ++index) {
            ctx->data[index] = 0x00U;
        }
    }

    ctx->bitlen += static_cast<uint64_t>(ctx->datalen) * 8U;
    ctx->data[63] = static_cast<uint8_t>(ctx->bitlen);
    ctx->data[62] = static_cast<uint8_t>(ctx->bitlen >> 8U);
    ctx->data[61] = static_cast<uint8_t>(ctx->bitlen >> 16U);
    ctx->data[60] = static_cast<uint8_t>(ctx->bitlen >> 24U);
    ctx->data[59] = static_cast<uint8_t>(ctx->bitlen >> 32U);
    ctx->data[58] = static_cast<uint8_t>(ctx->bitlen >> 40U);
    ctx->data[57] = static_cast<uint8_t>(ctx->bitlen >> 48U);
    ctx->data[56] = static_cast<uint8_t>(ctx->bitlen >> 56U);
    detail::sha256_transform(ctx, ctx->data.data());

    for (uint32_t index = 0U; index < 4U; ++index) {
        const uint32_t shift = 24U - (index * 8U);
        hash[index] = static_cast<uint8_t>((ctx->state[0] >> shift) & 0xFFU);
        hash[index + 4U] = static_cast<uint8_t>((ctx->state[1] >> shift) & 0xFFU);
        hash[index + 8U] = static_cast<uint8_t>((ctx->state[2] >> shift) & 0xFFU);
        hash[index + 12U] = static_cast<uint8_t>((ctx->state[3] >> shift) & 0xFFU);
        hash[index + 16U] = static_cast<uint8_t>((ctx->state[4] >> shift) & 0xFFU);
        hash[index + 20U] = static_cast<uint8_t>((ctx->state[5] >> shift) & 0xFFU);
        hash[index + 24U] = static_cast<uint8_t>((ctx->state[6] >> shift) & 0xFFU);
        hash[index + 28U] = static_cast<uint8_t>((ctx->state[7] >> shift) & 0xFFU);
    }
}

} // namespace librmcs::firmware::crypto
