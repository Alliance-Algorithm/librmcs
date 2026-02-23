#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <type_traits>

#include <board.h>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/src/spi/spi.hpp"

namespace librmcs::firmware::spi::bmi088 {

template <typename TraitsT>
class Bmi088Base : private SpiModule {
protected:
    static constexpr std::size_t kDummyBytes = TraitsT::kDummyBytes;
    static_assert(kDummyBytes >= 1);

    using RegisterAddressType = TraitsT::RegisterAddress;
    static_assert(
        std::is_scoped_enum_v<RegisterAddressType>
        && std::is_same_v<std::underlying_type_t<RegisterAddressType>, uint8_t>);

    static constexpr int kMaxRetries = 3;

    struct [[gnu::packed]] Data {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    explicit Bmi088Base(Spi::Lazy* spi, ChipSelectPin chip_select_pin)
        : SpiModule(chip_select_pin)
        , spi_(spi->init()) {}

    uint8_t read_register(RegisterAddressType addr) {
        spi_.transmit_receive(*this, prepare_tx_buffer_read(static_cast<uint8_t>(addr), 1));
        return static_cast<uint8_t>(spi_.rx_buffer[kDummyBytes]);
    }

    void write_register(RegisterAddressType addr, uint8_t val) {
        spi_.transmit_receive(*this, prepare_tx_buffer_write(static_cast<uint8_t>(addr), val));
    }

    bool read_and_confirm(RegisterAddressType addr, uint8_t expected) {
        for (int i = kMaxRetries; i-- > 0;) {
            if (read_register(addr) == expected)
                return true;
            board_delay_ms(1);
        }
        return false;
    }

    bool write_and_confirm(RegisterAddressType addr, uint8_t val) {
        for (int i = kMaxRetries; i-- > 0;) {
            write_register(addr, val);
            board_delay_ms(1);
            if (read_register(addr) == val)
                return true;
        }
        return false;
    }

    bool read_async(RegisterAddressType addr, std::size_t size) {
        if (!spi_.try_lock())
            return false;

        spi_.transmit_receive_async(
            *this, prepare_tx_buffer_read(static_cast<uint8_t>(addr), size));
        return true;
    }

    Data& parse_rx_data(std::byte* rx_buffer, std::size_t size) {
        core::utility::assert_debug(size == sizeof(Data) + kDummyBytes);
        return *std::launder(reinterpret_cast<Data*>(rx_buffer + kDummyBytes));
    }

    Spi& spi_;

private:
    std::size_t prepare_tx_buffer_read(uint8_t addr, std::size_t read_size) {
        core::utility::assert_debug(read_size + kDummyBytes <= Spi::kMaxTransferSize);

        spi_.tx_buffer[0] = static_cast<std::byte>(0x80 | addr);
        std::memset(&spi_.tx_buffer[1], 0, read_size + kDummyBytes - 1);
        return read_size + kDummyBytes;
    }

    std::size_t prepare_tx_buffer_write(uint8_t addr, uint8_t val) {
        static_assert(2 <= Spi::kMaxTransferSize);

        spi_.tx_buffer[0] = static_cast<std::byte>(addr);
        spi_.tx_buffer[1] = static_cast<std::byte>(val);
        return 2;
    }
};

} // namespace librmcs::firmware::spi::bmi088
