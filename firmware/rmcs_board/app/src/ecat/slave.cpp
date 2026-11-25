#include "firmware/rmcs_board/app/src/ecat/slave.hpp"

#include <cstdint>
#include <span>

extern "C" {
#include <ecat_slv.h>
#include <esc.h>
}

#include "core/src/protocol/ethercat/process_data.hpp"
#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/ecat/esc.hpp"
#include "firmware/rmcs_board/app/src/timer/timer.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_safe_buffer.hpp"

namespace librmcs::firmware::ecat {

using core::protocol::ethercat::ProcessData;

extern "C" {
extern uint8_t rxpdo[ProcessData::kSize];
extern uint8_t txpdo[ProcessData::kSize];
}

void Slave::start() {
    if (!Esc::start())
        return; // USB DFU remains available on boards without a usable PHY.

    esc_cfg_t config{};
    config.esc_hw_eep_handler = Esc::service_eeprom;
    config.safeoutput_override = [] { slave->reset_session(); };
    ecat_slv_init(&config);
    ready_ = true;
}

void Slave::poll() {
    if (!ready_)
        return;

    const auto previous_state = ESCvar.ALstatus;
    ecat_slv_poll();
    const bool operational = ESCvar.ALstatus == ESCop;
    const auto now = timer::Timer::timestamp_quarter_us();
    if (operational && !was_operational_)
        last_outputs_quarter_us_ = now;

    // SOES' optional software watchdog counts iterations, not time.
    constexpr uint32_t pd_timeout_quarter_us = 100'000 * 4;
    if (operational && now - last_outputs_quarter_us_ >= pd_timeout_quarter_us)
        ESC_ALstatusgotoerror(ESCsafeop | ESCerror, ALERR_WATCHDOG);
    was_operational_ = ESCvar.ALstatus == ESCop;

    const bool new_outputs = (ESCvar.ALevent & ESCREG_ALEVENT_SM2) != 0;
    if (session_.expire())
        reset_session();
    DIG_process(DIG_PROCESS_OUTPUTS_FLAG);
    if (new_outputs || ESCvar.ALstatus != previous_state)
        DIG_process(DIG_PROCESS_INPUTS_FLAG);
}

void Slave::receive_process_data() {
    if (ESCvar.ALstatus != ESCop)
        return;

    last_outputs_quarter_us_ = timer::Timer::timestamp_quarter_us();
    const auto frame = ProcessData::decode(std::as_bytes(std::span{rxpdo}));
    if (!frame)
        return;

    if (process_data_.epoch() == 0) {
        if (frame->epoch == retired_epoch_ || frame->sequence > 1 || frame->acknowledgement != 0)
            return;
        process_data_.reset(frame->epoch);
    }

    const auto payload = process_data_.receive(*frame);
    if (payload && !payload->empty())
        session_.feed(*payload, true);
}

void Slave::transmit_process_data() {
    if (session_.established() && !process_data_.busy()) {
        if (const auto* batch = transmit_buffer_.pop_batch()) {
            core::utility::assert_always(process_data_.enqueue(batch->data()));
            utility::InterruptSafeBuffer::release_batch(batch);
        }
    }
    process_data_.write(std::as_writable_bytes(std::span{txpdo}));
}

void Slave::reset_session() {
    if (process_data_.epoch() != 0)
        retired_epoch_ = process_data_.epoch();
    process_data_.reset(0);
    session_.deactivate();
    session_.finish_transfer();
    transmit_buffer_.clear();
}

extern "C" {
void cb_set_outputs() { slave->receive_process_data(); }
void cb_get_inputs() { slave->transmit_process_data(); }
}

} // namespace librmcs::firmware::ecat
