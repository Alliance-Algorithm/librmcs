#pragma once

#include "../utility/cross_os.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace librmcs::device {

class Ch040 {
public:
    PACKED_STRUCT(Quaterion {
        uint8_t label;
        float w;
        float x;
        float y;
        float z;
    };)

    PACKED_STRUCT(Header {
        uint8_t header;
        uint8_t type;
        uint16_t length;
    };)

    PACKED_STRUCT(Package {
        Header header;
        uint16_t crc;
        Quaterion q;
    };)

    explicit Ch040() = default;

    bool store_status(const uint8_t* uart_data, size_t uart_data_length) {
        if (uart_data_length != sizeof(Package))
            return false;

        const auto& package = *reinterpret_cast<const Package*>(uart_data);
        const auto& head = package.header;

        if (head.header != 0x5a || head.type != 0xa5 || head.length != 17)
            return false;

        if (!crc_check(uart_data, head.length, package.crc))
            return false;

        w_ = package.q.w;
        x_ = package.q.x;
        y_ = package.q.y;
        z_ = package.q.z;

        return true;
    }

    double w() const { return w_.load(std::memory_order::relaxed); }
    double x() const { return x_.load(std::memory_order::relaxed); }
    double y() const { return y_.load(std::memory_order::relaxed); }
    double z() const { return z_.load(std::memory_order::relaxed); }

protected:
    std::atomic<double> w_;
    std::atomic<double> x_;
    std::atomic<double> y_;
    std::atomic<double> z_;

    static void crc16_update(uint16_t* crc_src, const uint8_t* bytes, uint32_t len) {
        uint32_t crc = *crc_src;
        for (std::size_t byte_index = 0; byte_index < len; ++byte_index) {
            auto byte = static_cast<uint32_t>(bytes[byte_index]);
            crc ^= byte << 8;
            for (std::size_t crc_index = 0; crc_index < 8; ++crc_index) {
                uint32_t temp = crc << 1;
                if (crc & 0x8000)
                    temp ^= 0x1021;
                crc = temp;
            }
        }
        *crc_src = crc;
    }

    static bool crc_check(const uint8_t* bytes, uint32_t data_length, uint16_t data_crc) {
        uint16_t crc = 0;
        crc16_update(&crc, bytes, 4);
        crc16_update(&crc, bytes + 6, 17);

        return (crc == data_crc);
    }
};

} // namespace librmcs::device
