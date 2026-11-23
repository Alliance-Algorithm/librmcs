#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include <class/vendor/vendor_device.h>
#include <common/tusb_types.h>
#include <device/usbd.h>

#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/assert.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/session.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_safe_buffer.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::usb {

class Vendor : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<Vendor>;

    core::protocol::Serializer& serializer() { return serializer_; }

    void deactivate_session() { session_.deactivate(); }

    void handle_downlink(std::span<const std::byte> buffer, bool finished) {
        session_.feed(buffer, finished);
    }

    void finish_downlink_transfer() { session_.finish_transfer(); }

    bool try_transmit() {
        session_.expire();

        if (!session_.established()) {
            return false;
        }

        if (!tud_vendor_n_write_available(0))
            return false;

        if (!transmitting_batch_) {
            transmitting_batch_ = transmit_buffer_.pop_batch();
        }
        if (!transmitting_batch_)
            return false;

        const auto data = transmitting_batch_->data();

        const std::size_t max_packet_size = (tud_speed_get() == TUSB_SPEED_HIGH) ? 512 : 64;
        const auto target_size = std::min(data.size() - transmitted_size_, max_packet_size);

        if (target_size) {
            const auto* src = reinterpret_cast<const uint8_t*>(data.data() + transmitted_size_);
            core::utility::assert_debug(tud_vendor_n_write(0, src, target_size) == target_size);
        } else {
            core::utility::assert_debug(tud_vendor_n_write_zlp(0));
        }

        transmitted_size_ += target_size;
        if (transmitted_size_ == data.size() && target_size < max_packet_size) {
            transmit_buffer_.release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
            transmitted_size_ = 0;
        }

        return true;
    }

private:
    void reset_transmit() {
        if (transmitting_batch_) {
            transmit_buffer_.release_batch(transmitting_batch_);
            transmitting_batch_ = nullptr;
            transmitted_size_ = 0;
        }
        transmit_buffer_.clear();
    }

    utility::InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};
    Session session_{
        serializer_, [](void* context) { static_cast<Vendor*>(context)->reset_transmit(); }, this};

    const utility::InterruptSafeBuffer::Batch* transmitting_batch_ = nullptr;
    size_t transmitted_size_ = 0;
};

inline constinit Vendor::Lazy vendor;

} // namespace librmcs::firmware::usb
