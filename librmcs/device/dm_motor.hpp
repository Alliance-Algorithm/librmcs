#pragma once

#include <cmath>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstdint>
#include <cstdio>
namespace librmcs::device {

class DMMotor {
public:
    enum class Type : uint8_t { J6006, J4310};

    struct Config {
        explicit Config(Type motor_type) {
            this->position_zero_point = 0;
            this->motor_type = motor_type;
            switch (motor_type) {
                case Type::J6006: 
                    max_position = 3.141593;
                    max_velocity_ = 45;
                    max_torque_ = 12;
                    break;
                case Type::J4310: 
                    max_position = 3.141593;
                    max_velocity_ = 30;
                    max_torque_ = 10;
                    break;
            }
            this->reversed = false;
            this->multi_turn_angle_enabled = false;
        }

        Config& set_position_zero_point(int value) { return position_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& set_max_position() { return reversed = true, *this; }
        Config& set_max_velocity_() { return reversed = true, *this; }
        Config& set_max_torque_() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int position_zero_point;
        double reduction_ratio;
        double max_position;
        double max_velocity_;
        double max_torque_;
        bool reversed;
        bool multi_turn_angle_enabled;
    };

    explicit DMMotor(const Config& config) {
        angle_ = 0.0;
        velocity_ = 0.0;
        torque_ = 0.0;

        configure(config);
    }
    DMMotor(const DMMotor&) = delete;
    DMMotor& operator=(const DMMotor&) = delete;

    void configure(const Config& config) {
        position_zero_point_ = config.position_zero_point;

        double sign = config.reversed ? -1 : 1;

        raw_angle_to_angle_coefficient_ = 
            sign * 2 * config.max_position / static_cast<double>((1 << 16) - 1);
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;
        max_position_ = config.max_position;

        raw_velocity_to_velocity_coefficient_ = 
            sign * 2 * config.max_velocity_ / static_cast<double>((1 << 12) - 1);
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;
        max_velocity_ = config.max_velocity_;
        
        raw_torque_to_torque_coefficient_ = 
            sign * 2 * config.max_torque_ / static_cast<double>((1 << 12) - 1);
        torque_to_raw_torque_coefficient_ = 1 / raw_torque_to_torque_coefficient_;
        max_torque_ = config.max_torque_;

        last_angle = 0;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_ = 0;
    }

    void store_status(uint64_t can_data) { can_data_.store(can_data, std::memory_order_relaxed); }

    void update_status() {
        auto feedback = std::bit_cast<DMMotorFeedback>(can_data_.load(std::memory_order::relaxed));
        can_id_ = feedback.ID_and_ERR & 0xf0;
        error_ = feedback.ID_and_ERR & 0x0f;
        // Angle unit: rad
        uint16_t raw_angle_ = (feedback.position_part2 << 8) | (feedback.position_part1);
        double raw_angle = raw_angle_to_angle_coefficient_ * static_cast<double>(raw_angle_) - max_position_;
        double angle = raw_angle - position_zero_point_;
        if (angle < -max_position_)
            angle += 2 * max_position_;
        if (!multi_turn_angle_enabled_) {
            angle_ = angle;
        } else {
            auto diff = angle - last_angle;
            if (diff < -max_position_)
                angle_multi_turn_++;
            else if (diff > max_position_)
                angle_multi_turn_--;
            angle_ = angle + 2 * max_position_ * static_cast<double>(angle_multi_turn_);
        }
        last_angle = angle;

        // Velocity unit: rad/s
        uint16_t raw_velocity = (feedback.velocity_part2 << 4) | (feedback.velocity_part1_and_torque_part2 >> 4);
        velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(raw_velocity) - max_velocity_;

        // Torque unit: N*m
        uint16_t raw_torque = ((feedback.velocity_part1_and_torque_part2 & 0x0f) << 8) | (feedback.torque_part1);
        torque_ = raw_torque_to_torque_coefficient_ * static_cast<double>(raw_torque) - max_torque_;
    }

    uint64_t generate_control_command(double control_torque) const {
        if (std::isnan(control_torque)) {
            return 0;
        }

        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        uint8_t control_frame[8] = {0};
        uint16_t torque = static_cast<uint16_t>(torque_to_raw_torque_coefficient_ * (control_torque + max_torque_));
        control_frame[6] = torque >> 8;
        control_frame[7] = torque;
        return std::bit_cast<uint64_t>(control_frame);
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }

    uint16_t can_id() const { return can_id_; }
    uint8_t error() const { return error_; }
    static uint64_t enable_command() { return 0xFCFFFFFFFFFFFFFF; }
    static uint64_t disable_command() { return 0xFDFFFFFFFFFFFFFF; }

private:
    struct alignas(uint64_t) DMMotorFeedback {
        uint8_t ID_and_ERR;
        uint8_t position_part2;
        uint8_t position_part1;
        uint8_t velocity_part2;
        uint8_t velocity_part1_and_torque_part2;
        uint8_t torque_part1;
        uint8_t temperature_MOS;
        uint8_t temperature_Rotor;
    };

    std::atomic<uint64_t> can_data_ = 0;

    double position_zero_point_, last_angle;

    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_torque_to_torque_coefficient_, torque_to_raw_torque_coefficient_;

    double max_position_, max_velocity_, max_torque_;

    double angle_;
    double velocity_;
    double torque_;

    uint16_t can_id_;
    uint8_t error_;
};

} // namespace librmcs::device