#include <librmcs/client/cboard.hpp>
#include <librmcs/device/dji_motor.hpp>
#include <librmcs/utility/pid_calculator.hpp>

class MyRobot : public librmcs::client::CBoard {
public:
    explicit MyRobot(int32_t usb_pid = -1)
        : CBoard(usb_pid)
        , motor_(librmcs::device::DjiMotor::Config{librmcs::device::DjiMotor::Type::M3508})
        , pid_calculator_(librmcs::utility::PidCalculator{0.1, 0.0003, 0.0}
                              .set_integral_min(-1000.0)
                              .set_integral_max(1000.0))
        , transmit_buffer_(*this, 16) {}

private:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;

        if (can_id == 0x201) {
            motor_.store_status(can_data);
            motor_.update_status();

            constexpr double control_velocity = 6.28;

            const double err = control_velocity - motor_.velocity();
            const double control_torque = pid_calculator_.update(err);

            LOG_INFO(
                "velocity=%.2f, torque=%.2f/%.2f", motor_.velocity(), control_torque,
                motor_.max_torque());

            uint16_t control_commands[4];
            control_commands[0] = motor_.generate_command(control_torque);
            control_commands[1] = 0;
            control_commands[2] = 0;
            control_commands[3] = 0;

            transmit_buffer_.add_can1_transmission(
                0x200, std::bit_cast<uint64_t>(control_commands));
            transmit_buffer_.trigger_transmission();
        } else {
            LOG_INFO("unhandled CAN1 device: 0x%x", can_id);
        }
    }

    librmcs::device::DjiMotor motor_;
    librmcs::utility::PidCalculator pid_calculator_;

    TransmitBuffer transmit_buffer_;
};

int main() {
    MyRobot my_robot{};
    my_robot.handle_events();
}