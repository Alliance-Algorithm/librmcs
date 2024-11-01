#pragma once

#include <cmath>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "../utility/endian_promise.hpp"

namespace librmcs::device {

class LKMotor {
public:
    enum class Type : uint8_t { MG6012E};

    struct Config {
        explicit Config(Type motor_type) {
            this->encoder_zero_point = 0;
            this->motor_type = motor_type;
            switch (motor_type) {
                case Type::MG6012E: reduction_ratio = 1.0; break;
            }
            this->reversed = false;
            this->multi_turn_angle_enabled = false;
        }

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point;
        double reduction_ratio;
        bool reversed;
        bool multi_turn_angle_enabled;
    };

    explicit LKMotor(const Config& config) {
        angle_ = 0.0;
        velocity_ = 0.0;
        torque_ = 0.0;

        configure(config);
    }
    LKMotor(const LKMotor&) = delete;
    LKMotor& operator=(const LKMotor&) = delete;

    void configure(const Config& config) {
        encoder_zero_point_ = config.encoder_zero_point;

        double sign = config.reversed ? -1 : 1;

        double torque_constant, raw_current_max, current_max;
        switch (config.motor_type) {
        case Type::MG6012E:
            raw_angle_max_ = 65535; 
            torque_constant = 1.714286;
            raw_current_max = 2048.0;
            current_max = 33.0;
            break;
        default: throw std::runtime_error{"Unknown motor type"};
        }

        raw_angle_to_angle_coefficient_ = sign / config.reduction_ratio / raw_angle_max_ * 2 * M_PI;
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ = sign / config.reduction_ratio / 360 * 2 * M_PI;
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;
        
        

        raw_current_to_torque_coefficient_ =
            sign * config.reduction_ratio * torque_constant / raw_current_max * current_max;
        torque_to_raw_current_coefficient_ = 1 / raw_current_to_torque_coefficient_;

        max_torque_ = 1 * config.reduction_ratio * torque_constant * current_max;

        last_raw_angle_ = 0;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_ = 0;
    }

    void store_status(uint64_t can_data) { can_data_.store(can_data, std::memory_order_relaxed); }

    void update_status() {
        auto feedback = std::bit_cast<LKMotorFeedback>(can_data_.load(std::memory_order::relaxed));
        
        temperature_ = feedback.temperature;
        
        // Angle unit: rad
        int raw_angle = feedback.encoder;
        int angle = raw_angle - encoder_zero_point_;
        if (angle < 0)
            angle += raw_angle_max_;
        if (!multi_turn_angle_enabled_) {
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle);
        } else {
            auto diff = (angle - angle_multi_turn_) % raw_angle_max_;
            if (diff <= -raw_angle_max_ / 2)
                diff += raw_angle_max_;
            else if (diff > raw_angle_max_ / 2)
                diff -= raw_angle_max_;
            angle_multi_turn_ += diff;
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }
        // Velocity unit: rad/s
        int16_t raw_velocity = feedback.velocity;
        velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(raw_velocity) - max_velocity_;

        // Torque unit: N*m
        uint16_t raw_torque = feedback.current;
        torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(raw_torque);
    }

    uint64_t generate_control_command(double control_torque) const {
        if (std::isnan(control_torque)) {
            return 0;
        }
        uint8_t control_frame[8] = {0};
        control_frame[0] = 0xA1;

        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        double current = std::round(torque_to_raw_current_coefficient_ * control_torque);
        int16_t control_current = static_cast<int16_t>(current);
        control_frame[4] = control_current;
        control_frame[5] = control_current >> 8;
        return std::bit_cast<uint64_t>(control_frame);
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    int8_t temperature() const { return temperature_; }

    uint16_t can_id() const { return can_id_; }
    uint8_t error() const { return error_; }
    static uint64_t enable_command() { return 0x0000000000000088; }
    static uint64_t disable_command() { return 0x0000000000000081; }

private:
    struct alignas(uint64_t) LKMotorFeedback {
        uint8_t command;
        int8_t temperature;
        int16_t current;
        int16_t velocity;
        uint16_t encoder;
    };

    std::atomic<uint64_t> can_data_ = 0;

    int raw_angle_max_;
    int encoder_zero_point_, last_raw_angle_;

    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_current_to_torque_coefficient_, torque_to_raw_current_coefficient_;

    double max_position_, max_velocity_, max_torque_;

    double angle_;
    double velocity_;
    double torque_;
    int8_t temperature_;

    uint16_t can_id_;
    uint8_t error_;
};

} // namespace librmcs::device