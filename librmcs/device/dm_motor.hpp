#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>

#include "../utility/cross_os.hpp"

namespace librmcs::device {
class DmMotor {
public:
    enum class Type : uint8_t { DM8009 };
    struct Config {
        explicit Config(Type motor_type) {
            this->encoder_zero_point = 0;
            this->motor_type = motor_type;
            this->reversed = false;
            this->multi_turn_angle_enabled = false;
        }
        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point;
        bool reversed;
        bool multi_turn_angle_enabled;
    };

    DmMotor()
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {}

    explicit DmMotor(const Config& config)
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {
        configure(config);
    }

    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;

    void configure(const Config& config) {

        double reduction_ratio;
        switch (config.motor_type) {
        case Type::DM8009:
            raw_angle_max_ = 65535;
            reduction_ratio = 1.0;
            max_velocity_ = 45.0;
            max_torque_ = 54.0;

            max_angle_ = std::numbers::pi;
            max_kp_ = 500.0;
            max_kd_ = 5.0;
            break;
        }

        encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += raw_angle_max_;

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

        const double sign = config.reversed ? -1.0 : 1.0;

        status_angle_to_angle_coefficient_ = sign / raw_angle_max_ * 2 * std::numbers::pi;
        angle_to_command_angle_coefficient_ = sign * reduction_ratio * rad_to_deg_;

        status_velocity_to_velocity_coefficient_ = sign / reduction_ratio;
        velocity_to_command_velocity_coefficient_ = sign * reduction_ratio;

        status_torque_to_torque_coefficient_ = sign * reduction_ratio;
        torque_to_command_torque_coefficient_ = 1 / status_torque_to_torque_coefficient_;
    }

    void store_status(uint64_t can_data) { can_data_.store(can_data, std::memory_order::relaxed); }

    void update_status() {
        uint64_t feedback = can_data_.load(std::memory_order_relaxed);

        uint8_t frame[8];
        std::memcpy(frame, &feedback, sizeof(feedback));

        id_ = (frame[0]) & 0xff;
        motor_state_ = (frame[0]) >> 4;
        uint16_t encoder = (frame[1] << 8) | frame[2];
        uint16_t velocity = (frame[3] << 4) | (frame[4] >> 4);
        uint16_t torque = ((frame[4] & 0xF) << 8) | frame[5];

        mos_temperature_ = static_cast<double>(frame[6]);
        rotor_temperature_ = static_cast<double>(frame[7]);

        const auto raw_angle = static_cast<int>(encoder);

        position_ = uint_to_double(encoder, -max_angle_, max_angle_, 16);

        auto calibrated_raw_angle = encoder - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += raw_angle_max_;
        if (!multi_turn_angle_enabled_) {
            angle_ = status_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle_ < 0)
                angle_ += 2 * std::numbers::pi;
        } else {
            // Calculates the minimal difference between two angles and normalizes it to the range
            // (-raw_angle_max_/2, raw_angle_max_/2].
            // This implementation leverages bitwise operations for efficiency, which is valid only
            // when raw_angle_max_ is a power of 2.
            auto diff = (calibrated_raw_angle - multi_turn_encoder_count_) & (raw_angle_max_ - 1);
            if (diff > (raw_angle_max_ >> 1))
                diff -= raw_angle_max_;

            multi_turn_encoder_count_ += diff;
            angle_ =
                status_angle_to_angle_coefficient_ * static_cast<double>(multi_turn_encoder_count_);
        }
        last_raw_angle_ = raw_angle;

        angle_value_ = status_angle_to_angle_coefficient_ * static_cast<double>(encoder);
        if (angle_value_ < 0)
            angle_value_ += 2 * std::numbers::pi;

        velocity_ = status_velocity_to_velocity_coefficient_
                  * uint_to_double(velocity, -max_velocity_, max_velocity_, 12);

        torque_ = status_torque_to_torque_coefficient_
                * uint_to_double(torque, -max_torque_, max_torque_, 12);
    }

    int64_t calibrate_zero_point() {
        angle_multi_turn_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    uint8_t error_state() const { return motor_state_; }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }

    double angle_value() const { return angle_value_; }
    double position() const { return position_; }

    double mos_temperature() const { return mos_temperature_; }
    double rotor_temperature() const { return rotor_temperature_; }

    constexpr static uint64_t generate_enable_command() {
        PACKED_STRUCT({
            uint8_t message[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        } command alignas(uint64_t){};)
        return std::bit_cast<uint64_t>(command);
    }

    constexpr static uint64_t generate_clear_error_command() {
        PACKED_STRUCT({
            uint8_t message[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
        } command alignas(uint64_t){};)
        return std::bit_cast<uint64_t>(command);
    }

    uint64_t generate_mit_command(
        double control_angle, double control_velocity, double control_torque, double control_kp,
        double control_kd) const {
        uint64_t result;

        control_angle = std::clamp(control_angle, -max_angle_, max_angle_);
        control_velocity = std::clamp(control_velocity, -max_velocity_, max_velocity_);
        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);

        uint16_t angle = double_to_uint(control_angle, -max_angle_, max_angle_, 16);
        uint16_t velocity = double_to_uint(control_velocity, -max_velocity_, max_velocity_, 12);
        uint16_t torque = double_to_uint(control_torque, -max_torque_, max_torque_, 12);
        uint16_t kp = double_to_uint(control_kp, 0, max_kp_, 12);
        uint16_t kd = double_to_uint(control_kd, 0, max_kd_, 12);
        uint8_t tx_buff[8]{};

        tx_buff[0] = (angle >> 8);
        tx_buff[1] = angle;
        tx_buff[2] = (velocity >> 4);
        tx_buff[3] = ((velocity & 0xF) << 4) | (kp >> 8);
        tx_buff[4] = kp;
        tx_buff[5] = (kd >> 4);
        tx_buff[6] = ((kd & 0xF) << 4) | (torque >> 8);
        tx_buff[7] = torque;

        std::copy(tx_buff, tx_buff + 8, reinterpret_cast<uint8_t*>(&result));

        return result;
    }

    /// @brief Disable the motor, but do not clear the motor's running state. Sending control
    /// commands again can control the motor actions.
    uint64_t generate_disable_command() const {
        // Note: instead of sending a real disable message here, a torque control message with
        // torque set to 0 is sent, because the disable message does not cause the motor to feedback
        // its status.
        return generate_mit_command(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    uint64_t generate_torque_command(double control_torque) const {
        if (std::isnan(control_torque)) {
            return generate_disable_command();
        }
        return generate_mit_command(0.0, 0.0, control_torque, 0.0, 0.0);
    }

    uint64_t generate_angle_command(
        double control_angle, double control_kp = 65, double control_kd = 2) const {
        if (std::isnan(control_angle)) {
            return generate_disable_command();
        }
        return generate_mit_command(control_angle, 0.0, 0.0, control_kp, control_kd);
    }

private:
    int to_command_torque(double torque) const {
        double command_torque = torque_to_command_torque_coefficient_ * torque;
        return double_to_uint(command_torque, -max_torque_, max_torque_, 12);
    }

    int to_command_velocity(double velocity) const {
        double command_velocity = velocity_to_command_velocity_coefficient_ * velocity;
        return double_to_uint(command_velocity, -max_velocity_, max_velocity_, 12);
    }

    static double uint_to_double(int x_int, double x_min, double x_max, int bits) {
        double span = x_max - x_min;
        double offset = x_min;
        return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
    }

    static int double_to_uint(double x_double, double x_min, double x_max, int bits) {
        double span = x_max - x_min;
        double offset = x_min;
        return (int)((x_double - offset) * ((double)((1 << bits) - 1)) / span);
    }

    std::atomic<uint64_t> can_data_ = 0;

    // Constants
    static constexpr double deg_to_rad_ = std::numbers::pi / 180;
    static constexpr double rad_to_deg_ = 180 / std::numbers::pi;

    static constexpr int raw_current_max_ = 2048;
    int raw_angle_max_;
    int encoder_zero_point_;

    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double status_angle_to_angle_coefficient_, angle_to_command_angle_coefficient_;
    double status_velocity_to_velocity_coefficient_, velocity_to_command_velocity_coefficient_;
    double status_torque_to_torque_coefficient_, torque_to_command_torque_coefficient_;

    int64_t multi_turn_encoder_count_ = 0;
    int last_raw_angle_ = 0;

    uint8_t id_;
    uint8_t motor_state_;
    double angle_;
    double velocity_;
    double torque_;

    double angle_value_ = 0.0;
    double position_ = 0.0;

    double max_angle_;
    double max_torque_;
    double max_velocity_;

    double max_kp_;
    double max_kd_;

    double mos_temperature_;
    double rotor_temperature_;
};
} // namespace librmcs::device