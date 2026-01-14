#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace librmcs::core::utility {

struct BitfieldMemberTag {};

template <std::size_t BitWidth>
using DefaultValueTypeT = std::conditional_t<
    (BitWidth == 1), bool,
    std::conditional_t<
        (BitWidth <= 8), std::uint8_t,
        std::conditional_t<
            (BitWidth <= 16), std::uint16_t,
            std::conditional_t<
                (BitWidth <= 32), std::uint32_t,
                std::conditional_t<(BitWidth <= 64), std::uint64_t, void>>>>>;

/**
 * @brief Describes a single field within a Bitfield buffer.
 *
 * If ValueT is a signed integral type and BitWidth is smaller than sizeof(ValueT) * 8,
 * Bitfield::get performs two's-complement sign extension after extraction.
 */
template <std::size_t Index, std::size_t BitWidth, typename ValueT = DefaultValueTypeT<BitWidth>>
requires(
    std::is_trivial_v<ValueT> && sizeof(ValueT) <= sizeof(std::uint64_t)
    && (std::is_integral_v<ValueT> || std::is_enum_v<ValueT>) && (sizeof(ValueT) * 8) >= BitWidth)
struct BitfieldMember : BitfieldMemberTag {
    static_assert(BitWidth > 0 && BitWidth <= 64);

    static constexpr std::size_t index = Index;
    static constexpr std::size_t bit_width = BitWidth;

    using ValueType = ValueT;
};

template <typename T>
concept is_bitfield_member = std::derived_from<T, BitfieldMemberTag>;

/**
 * @brief Fixed-layout little-endian bitfield over a byte buffer.
 *
 * Bitfield provides compile-time defined bitfield access on top of a
 * std::byte buffer, with a layout that is independent of CPU endianness
 * and compiler-specific struct bitfield rules.
 *
 * Bit indexing follows a fixed little-endian bitstream convention:
 *   - bit 0   is the least significant bit of data[0]
 *   - bit 7   is the most significant bit of data[0]
 *   - bit 8   is the least significant bit of data[1]
 *   - and so on: bit N belongs to data[N / 8], at position (N % 8)
 *
 * This mapping is the same on all architectures, regardless of whether
 * the target CPU is little-endian or big-endian.
 *
 * Fields are described by BitfieldMember<Index, BitCount> types, which
 * define the starting bit index and width. Access is performed via:
 *   - Bitfield<SizeInBytes>::get<Member>(...) to read a field
 *   - Bitfield<SizeInBytes>::set<Member>(...) to write a field
 *
 * The implementation:
 *   - operates only on std::byte and integer/enum types (bitwise extraction)
 *   - supports signed integral types via two's-complement sign extension
 *   - does not use or rely on C++ struct bitfields
 *   - enforces bounds and width constraints at compile time where possible
 *   - does not perform any implicit initialization or runtime bounds checks
 *
 * Notes on usage and safety:
 *   - This type is intended to be zero‑overhead. The `data` buffer is
 *     intentionally left uninitialized; the user is responsible for
 *     initializing it (e.g. `Bitfield<...> bf{};`) before reading.
 *   - Pointer‑based `get`/`set` overloads assume that the caller provides
 *     a buffer of at least `SizeInBytes` bytes. No runtime bounds checking
 *     is performed; out‑of‑bounds access is a caller ub.
 *
 * This type is intended for protocol headers, binary payloads, and other
 * data formats that explicitly define a little-endian bit layout.
 *
 * Usage example:
 *     struct MyBitfield : Bitfield<4> {
 *         using Id = BitfieldMember<0, 4>;
 *         using Flag = BitfieldMember<4, 1>;
 *         using Length = BitfieldMember<5, 27>;
 *     } bf{};
 *     bf.set<MyBitfield::Id>(0xA);
 *     bf.set<MyBitfield::Flag>(true);
 *     bf.set<MyBitfield::Length>(512);
 *     assert(bf.get<MyBitfield::Id>() == 0xA);
 *     assert(bf.get<MyBitfield::Flag>() == true);
 *     assert(bf.get<MyBitfield::Length>() == 512);
 *
 * @tparam SizeInBytes size of the underlying buffer in bytes
 */
template <std::size_t SizeInBytes>
struct Bitfield {
private:
    template <is_bitfield_member Member>
    static consteval bool check_member() {
        constexpr std::size_t first_bit = Member::index;
        constexpr std::size_t bit_width = Member::bit_width;
        constexpr std::size_t last_bit = first_bit + bit_width;
        constexpr std::size_t first_byte = first_bit / 8;
        constexpr std::size_t last_byte = (last_bit + 7) / 8;
        constexpr std::size_t span_bytes = last_byte - first_byte;

        return bit_width > 0                         //
            && first_bit < size_in_bits              //
            && bit_width <= size_in_bits - first_bit //
            && span_bytes > 0                        //
            && span_bytes <= sizeof(std::uint64_t);  //
    }

    template <std::size_t SpanBytes>
    using BestWordForSpan = std::conditional_t<
        (SpanBytes <= 1), std::uint8_t,
        std::conditional_t<
            (SpanBytes <= 2), std::uint16_t,
            std::conditional_t<(SpanBytes <= 4), std::uint32_t, std::uint64_t>>>;

public:
    static_assert(SizeInBytes > 0);
    static_assert(SizeInBytes <= std::numeric_limits<std::size_t>::max() / 8);

    static constexpr std::size_t size_in_bytes = SizeInBytes;
    static constexpr std::size_t size_in_bits = SizeInBytes * 8;

    template <is_bitfield_member Member>
    requires(check_member<Member>())
    [[nodiscard]] static constexpr auto get(const std::byte* src) noexcept ->
        typename Member::ValueType {
        return read_bits<Member>(src);
    }

    template <is_bitfield_member Member>
    requires(check_member<Member>())
    static constexpr void set(typename Member::ValueType value, std::byte* dst) noexcept {
        write_bits<Member>(value, dst);
    }

    template <is_bitfield_member Member>
    requires(check_member<Member>())
    [[nodiscard]] constexpr auto get() const noexcept -> typename Member::ValueType {
        return get<Member>(data);
    }

    template <is_bitfield_member Member>
    requires(check_member<Member>()) constexpr void set(typename Member::ValueType value) noexcept {
        set<Member>(value, data);
    }

    // Buffer is intentionally left uninitialized for zero-cost construction
    std::byte data[size_in_bytes];

    struct Ref {
        constexpr explicit Ref(std::byte* ptr) noexcept
            : ptr_(ptr) {}

        template <is_bitfield_member Member>
        requires(check_member<Member>())
        [[nodiscard]] constexpr auto get() const noexcept -> typename Member::ValueType {
            return Bitfield::template get<Member>(ptr_);
        }

        template <is_bitfield_member Member>
        requires(check_member<Member>())
        constexpr void set(typename Member::ValueType value) noexcept {
            Bitfield::template set<Member>(value, ptr_);
        }

    private:
        std::byte* ptr_;
    };

    struct CRef {
        constexpr explicit CRef(const std::byte* ptr) noexcept
            : ptr_(ptr) {}

        template <is_bitfield_member Member>
        requires(check_member<Member>())
        [[nodiscard]] constexpr auto get() const noexcept -> typename Member::ValueType {
            return Bitfield::template get<Member>(ptr_);
        }

    private:
        const std::byte* ptr_;
    };

private:
    template <is_bitfield_member Member>
    [[nodiscard]] static constexpr auto read_bits(const std::byte* src) noexcept ->
        typename Member::ValueType {

        constexpr std::size_t first_bit = Member::index;
        constexpr std::size_t bit_width = Member::bit_width;
        constexpr std::size_t last_bit = first_bit + bit_width;
        constexpr std::size_t first_byte = first_bit / 8;
        constexpr std::size_t last_byte = (last_bit + 7) / 8;
        constexpr std::size_t span_bytes = last_byte - first_byte;

        using Word = BestWordForSpan<span_bytes>;

        Word accum = 0;

        for (std::size_t i = 0; i < span_bytes; ++i) {
            auto b = std::to_integer<std::uint8_t>(src[first_byte + i]);
            accum |= (Word(b) << (8 * i));
        }

        constexpr std::size_t inner_offset = first_bit - first_byte * 8;

        constexpr Word full_mask =
            (bit_width == sizeof(Word) * 8) ? ~Word(0) : ((Word(1) << bit_width) - 1);

        Word value = (accum >> inner_offset) & full_mask;

        using ValueType = typename Member::ValueType;
        if constexpr (std::is_integral_v<ValueType> && std::is_signed_v<ValueType>) {
            // Use the "shift-left-then-shift-right" idiom for sign extension.
            // This relies on C++20's mandatory arithmetic right shift and defined left-shift
            // overflow.
            static_assert(__cplusplus >= 202002L);

            using ComputeType =
                std::conditional_t<(sizeof(Word) < sizeof(int)), int, std::make_signed_t<Word>>;
            constexpr int shift_amount = sizeof(ComputeType) * 8 - bit_width;
            return static_cast<ValueType>(
                (static_cast<ComputeType>(value) << shift_amount) >> shift_amount);
        } else {
            return static_cast<ValueType>(value);
        }
    }

    template <is_bitfield_member Member>
    static constexpr void write_bits(typename Member::ValueType value, std::byte* dst) noexcept {

        constexpr std::size_t first_bit = Member::index;
        constexpr std::size_t bit_width = Member::bit_width;
        constexpr std::size_t last_bit = first_bit + bit_width;
        constexpr std::size_t first_byte = first_bit / 8;
        constexpr std::size_t last_byte = (last_bit + 7) / 8;
        constexpr std::size_t span_bytes = last_byte - first_byte;

        using Word = BestWordForSpan<span_bytes>;

        Word accum = 0;

        for (std::size_t i = 0; i < span_bytes; ++i) {
            auto b = std::to_integer<std::uint8_t>(dst[first_byte + i]);
            accum |= (Word(b) << (8 * i));
        }

        constexpr std::size_t inner_offset = first_bit - first_byte * 8;

        constexpr Word full_mask =
            (bit_width == sizeof(Word) * 8) ? ~Word(0) : ((Word(1) << bit_width) - 1);

        accum &= ~(full_mask << inner_offset);

        Word v = (Word(value) & full_mask) << inner_offset;
        accum |= v;

        for (std::size_t i = 0; i < span_bytes; ++i) {
            std::uint8_t b = static_cast<std::uint8_t>((accum >> (8 * i)) & 0xFF);
            dst[first_byte + i] = std::byte{b};
        }
    }
};

} // namespace librmcs::core::utility
