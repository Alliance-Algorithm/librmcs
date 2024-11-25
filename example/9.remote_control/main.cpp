#include <csignal>

#include <atomic>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <librmcs/device/dji_motor.hpp>
#include <librmcs/device/dr16.hpp>
#include <librmcs/utility/pid_calculator.hpp>

class MyRobot : public librmcs::client::CBoard {
public:
    explicit MyRobot(int32_t usb_pid = -1)
        : CBoard(usb_pid)
        , motor_(librmcs::device::DjiMotor::Config{librmcs::device::DjiMotor::Type::M3508})
        , pid_calculator_(librmcs::utility::PidCalculator{0.1, 0.0003, 0.0}
                              .set_integral_min(-1000.0)
                              .set_integral_max(1000.0))
        , transmit_buffer_(*this, 16)
        , event_thread_([this]() { handle_events(); }) {}

    ~MyRobot() {
        stop_handling_events();
        event_thread_.join();

        transmit_buffer_.add_can1_transmission(0x200, 0);
        transmit_buffer_.trigger_transmission();

        LOG_INFO("Correctly deconstructed");
    }

    void update() {
        dr16_.update_status();
        const double control_velocity = 6.28 * dr16_.joystick_right().x;

        motor_.update_status();
        const double err = control_velocity - motor_.velocity();
        const double control_torque = pid_calculator_.update(err);

        LOG_INFO(
            "[%ld] target=%.2f velocity=%.2f, torque=%.2f/%.2f", update_count++, control_velocity,
            motor_.velocity(), control_torque, motor_.max_torque());

        uint16_t control_commands[4];
        control_commands[0] = motor_.generate_command(control_torque);
        control_commands[1] = 0;
        control_commands[2] = 0;
        control_commands[3] = 0;

        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(control_commands));
        transmit_buffer_.trigger_transmission();
    }

private:
    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;

        if (can_id == 0x201) {
            motor_.store_status(can_data);
        } else {
            LOG_INFO("unhandled CAN1 device: 0x%x", can_id);
        }
    }

    long update_count = 0;

    librmcs::device::Dr16 dr16_;

    librmcs::device::DjiMotor motor_;
    librmcs::utility::PidCalculator pid_calculator_;

    TransmitBuffer transmit_buffer_;

    std::thread event_thread_;
};

int main() {
    static std::atomic<bool> running = true;
    std::signal(SIGINT, [](int sig) {
        (void)sig;
        running = false;
    });

    MyRobot my_robot{};

    using namespace std::chrono_literals;
    constexpr double update_rate = 1000.0;
    constexpr auto update_period =
        std::chrono::round<std::chrono::steady_clock::duration>(1.0s / update_rate);

    auto next_iteration_time = std::chrono::steady_clock::now();
    while (running.load(std::memory_order::relaxed)) {
        my_robot.update();
        next_iteration_time += update_period;
        std::this_thread::sleep_until(next_iteration_time);
    }
}