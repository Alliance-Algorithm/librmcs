#pragma once

#include <cstdint>
#include <stdexcept>

#include <librmcs/data/datas.hpp>
#include <librmcs/protocol/handler.hpp>
#include <librmcs/protocol/i2c.hpp>

namespace librmcs::agent::detail {

class SingleI2c0DataCallback : public data::DataCallback {
public:
    bool i2c_receive_callback(data::DataId id, const data::I2cDataView& data) final {
        switch (id) {
        case data::DataId::kI2c0: i2c0_receive_callback(data); return true;
        default: return false;
        }
    }

    void i2c_error_callback(data::DataId id, const data::I2cErrorView& data) final {
        switch (id) {
        case data::DataId::kI2c0: i2c0_error_callback(data); break;
        default: break;
        }
    }

protected:
    virtual void i2c0_receive_callback(const data::I2cDataView& data) { (void)data; }

    virtual void i2c0_error_callback(const data::I2cErrorView& data) { (void)data; }

    void i2c0_error_from_slave_address(uint8_t slave_address) {
        data::DataCallback::i2c_error_from_slave_address(data::DataId::kI2c0, slave_address);
    }
};

template <typename Derived>
class I2c0PacketBuilderMixin {
public:
    static constexpr uint16_t kI2cMaxDataLength = librmcs::protocol::kI2cMaxDataLength;

    Derived& i2c0_write(const data::I2cDataView& data) {
        if (data.payload.empty() || data.payload.size() > kI2cMaxDataLength
            || data.slave_address > 0x7FU) [[unlikely]]
            throw std::invalid_argument{"I2C0 write failed: Invalid I2C data"};
        if (!builder_.write_i2c(data::DataId::kI2c0, data)) [[unlikely]]
            throw std::runtime_error{"I2C0 write failed: Transmit buffer unavailable"};
        return derived();
    }

    Derived& i2c0_read(const data::I2cReadConfigView& data) {
        if (data.read_length == 0 || data.read_length > kI2cMaxDataLength
            || data.slave_address > 0x7FU) [[unlikely]]
            throw std::invalid_argument{"I2C0 read failed: Invalid I2C read config"};
        if (!builder_.write_i2c_read_config(data::DataId::kI2c0, data)) [[unlikely]]
            throw std::runtime_error{"I2C0 read failed: Transmit buffer unavailable"};
        return derived();
    }

protected:
    host::protocol::Handler::PacketBuilder builder_;

private:
    friend Derived;

    explicit I2c0PacketBuilderMixin(host::protocol::Handler& handler) noexcept
        : builder_(handler.start_transmit()) {}

    Derived& derived() noexcept { return static_cast<Derived&>(*this); }
};

} // namespace librmcs::agent::detail
