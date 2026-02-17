#pragma once

#include <chrono> // IWYU pragma: keep (https://github.com/llvm/llvm-project/issues/68213)
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <type_traits>

#include <main.h>

#include "core/src/utility/assert.hpp"
#include "firmware/c_board/src/spi/spi.hpp"
#include "firmware/c_board/src/timer/delay.hpp"

namespace librmcs::firmware::spi::bmi088 {

template <typename TraitsT>
class Bmi088Base : private SpiModule {
protected:
    static constexpr size_t kDummyBytes = TraitsT::kDummyBytes;
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

    Bmi088Base(Spi::Lazy* spi, GPIO_TypeDef* chip_select_port, uint16_t chip_select_pin)
        : SpiModule(chip_select_port, chip_select_pin)
        , spi_(spi->init()) {}

    uint8_t read_register(RegisterAddressType addr) {
        spi_.transmit_receive(*this, prepare_tx_buffer_read(static_cast<uint8_t>(addr), 1));
        return spi_.rx_buffer[kDummyBytes];
    }

    void write_register(RegisterAddressType addr, uint8_t val) {
        spi_.transmit_receive(*this, prepare_tx_buffer_write(static_cast<uint8_t>(addr), val));
    }

    bool read_and_confirm(RegisterAddressType addr, uint8_t expected) {
        using namespace std::chrono_literals;

        for (int i = kMaxRetries; i-- > 0;) {
            if (read_register(addr) == expected)
                return true;
            timer::delay(1ms);
        }
        return false;
    }

    bool write_and_confirm(RegisterAddressType addr, uint8_t val) {
        using namespace std::chrono_literals;

        for (int i = kMaxRetries; i-- > 0;) {
            write_register(addr, val);
            timer::delay(1ms);
            if (read_register(addr) == val)
                return true;
        }
        return false;
    }

    bool read_async(RegisterAddressType addr, size_t size) {
        if (!spi_.try_lock())
            return false;

        spi_.transmit_receive_async(
            *this, prepare_tx_buffer_read(static_cast<uint8_t>(addr), size));
        return true;
    }

    Data& parse_rx_data(uint8_t* rx_buffer, size_t size) {
        core::utility::assert_debug(size == sizeof(Data) + kDummyBytes);
        return *std::launder(reinterpret_cast<Data*>(rx_buffer + kDummyBytes));
    }

    Spi& spi_;

private:
    size_t prepare_tx_buffer_read(uint8_t addr, size_t read_size) {
        core::utility::assert_debug(read_size + kDummyBytes <= Spi::kMaxTransferSize);

        spi_.tx_buffer[0] = 0x80 | addr;
        std::memset(&spi_.tx_buffer[1], 0, read_size + kDummyBytes - 1);
        return read_size + kDummyBytes;
    }

    size_t prepare_tx_buffer_write(uint8_t addr, uint8_t val) {
        static_assert(2 <= Spi::kMaxTransferSize);

        spi_.tx_buffer[0] = addr;
        spi_.tx_buffer[1] = val;
        return 2;
    }
};

} // namespace librmcs::firmware::spi::bmi088
