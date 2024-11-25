#include <cmath>

#include <bit>

#include <librmcs/client/cboard.hpp>
#include <librmcs/utility/endian_promise.hpp>

class MyRobot : public librmcs::client::CBoard {
public:
    explicit MyRobot(int32_t usb_pid = -1)
        : CBoard(usb_pid)
        , transmit_buffer_(*this, 16) {}

private:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;

        if (can_id == 0x201) {
            using librmcs::utility::be_int16_t;

            struct {
                be_int16_t angle;
                be_int16_t velocity;
                be_int16_t current;
                uint8_t temperature;
                uint8_t unused;
            } feedback = std::bit_cast<decltype(feedback)>(can_data);

            constexpr double control_velocity = 2000.0;
            constexpr double kp = 3.0;

            const double err = control_velocity - static_cast<double>(feedback.velocity);
            const double control_current = std::round(std::clamp(kp * err, -16384.0, 16384.0));

            be_int16_t control_currents[4];
            control_currents[0] = static_cast<int16_t>(control_current);
            control_currents[1] = 0;
            control_currents[2] = 0;
            control_currents[3] = 0;

            LOG_INFO(
                "velocity=%d, control_current=%d", (int)feedback.velocity,
                (int)control_currents[0]);

            transmit_buffer_.add_can1_transmission(
                0x200, std::bit_cast<uint64_t>(control_currents));
            transmit_buffer_.trigger_transmission();
        } else {
            LOG_INFO("unhandled CAN1 device: 0x%x", can_id);
        }
    }

    TransmitBuffer transmit_buffer_;
};

int main() {
    MyRobot my_robot{};
    my_robot.handle_events();
}