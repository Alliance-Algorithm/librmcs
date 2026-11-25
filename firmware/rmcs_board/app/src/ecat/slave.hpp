#pragma once

#include <cstdint>

#include "core/src/protocol/ethercat/process_data.hpp"
#include "core/src/protocol/serializer.hpp"
#include "core/src/utility/immovable.hpp"
#include "firmware/rmcs_board/app/src/session.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_safe_buffer.hpp"
#include "firmware/rmcs_board/app/src/utility/lazy.hpp"

namespace librmcs::firmware::ecat {

class Slave : private core::utility::Immovable {
public:
    void start();
    void poll();
    core::protocol::Serializer& serializer() { return serializer_; }

    void receive_process_data();
    void transmit_process_data();

private:
    void reset_session();

    core::protocol::ethercat::ProcessDataChannel<2> process_data_;
    utility::InterruptSafeBuffer transmit_buffer_;
    core::protocol::Serializer serializer_{transmit_buffer_};
    Session session_{
        serializer_, [](void* context) { static_cast<Slave*>(context)->transmit_buffer_.clear(); },
        this};
    uint32_t last_outputs_quarter_us_ = 0;
    uint32_t retired_epoch_ = 0;
    bool ready_ = false;
    bool was_operational_ = false;
};

inline constinit utility::Lazy<Slave> slave;

} // namespace librmcs::firmware::ecat
