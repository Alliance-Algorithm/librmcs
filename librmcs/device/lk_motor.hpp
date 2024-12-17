#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstring>
#include <numbers>
#include <stdexcept>

namespace librmcs::device {

class LkMotor {
public:
    enum class Type : uint8_t {
        UNKNOWN = 0,
        MG6012E = 1,
        MG5010E = 2,
    };

    struct Config {
        uint16_t raw_encoder_max;
        int16_t raw_current_max;
        double real_current_max;

        bool reversed;
        double torque_constant;
        double encoder_reduction_ratio;
        double velocity_reduction_ratio;
        bool multi_turn_angle_enabled;

        int encoder_zero_point;

        explicit Config(Type type) {
            encoder_zero_point = 0;
            reversed = false;
            multi_turn_angle_enabled = false;

            switch (type) {
            case Type::MG5010E:
                raw_encoder_max = 65535;
                raw_current_max = 2048;
                real_current_max = 33.0;
                torque_constant = 1;
                encoder_reduction_ratio = 1.0;
                velocity_reduction_ratio = 10.0;
                break;
            case Type::MG6012E:
                raw_encoder_max = 65535;
                raw_current_max = 2048;
                real_current_max = 33.0;
                torque_constant = 1.714286;
                encoder_reduction_ratio = 1.0;
                velocity_reduction_ratio = 1.0;
                break;
            default: throw std::runtime_error{"Unknown motor type"};
            }
        }

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return encoder_reduction_ratio = value, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
        Config& reverse() { return reversed = true, *this; }
    };

    explicit LkMotor(const Config& config) {
        encoder_zero_point_ = 0;
        last_encoder_count_ = 0;
        multi_turn_angle_enabled_ = false;

        encoder_to_angle_coefficient_ = 0;
        angle_to_encoder_coefficient_ = 0;
        raw_to_velocity_coefficient_ = 0;
        velocity_to_raw_coefficient_ = 0;
        raw_current_to_torque_coefficient_ = 0;
        torque_to_raw_current_coefficient_ = 0;

        configure(config);
    }

    void configure(const Config& config) {
        encoder_zero_point_ = config.encoder_zero_point;
        raw_encoder_max_ = config.raw_encoder_max;
        torque_constant_ = config.torque_constant;
        raw_current_max_ = config.raw_current_max;
        real_current_max_ = config.real_current_max;
        velocity_reduction_ratio_ = config.velocity_reduction_ratio;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

        multi_turn_encoder_count_ = 0;
        last_encoder_count_ = 0;

        const auto direction = config.reversed ? -1 : 1;
        const auto encoder_ratio = config.encoder_reduction_ratio;
        const auto velocity_ratio = config.velocity_reduction_ratio;

        encoder_to_angle_coefficient_ =
            direction / encoder_ratio / raw_encoder_max_ * 2 * std::numbers::pi;
        angle_to_encoder_coefficient_ = 1 / encoder_to_angle_coefficient_;

        raw_to_velocity_coefficient_ = direction / velocity_ratio / 360 * 2 * std::numbers::pi;
        velocity_to_raw_coefficient_ = 1 / raw_to_velocity_coefficient_;

        raw_current_to_torque_coefficient_ =
            direction * torque_constant_ / raw_current_max_ * real_current_max_;
        torque_to_raw_current_coefficient_ = 1 / raw_current_to_torque_coefficient_;

        torque_max_ = torque_constant_ * real_current_max_;
    }

    void store_status(uint64_t can_data) { can_buffer_.store(can_data, std::memory_order_relaxed); }

    void update_status() {
        struct alignas(uint64_t) {
            uint8_t command;
            int8_t temperature;
            int16_t current;
            int16_t velocity;
            uint16_t encoder;
        } feedback = std::bit_cast<decltype(feedback)>(can_buffer_.load(std::memory_order_relaxed));

        temperature_ = feedback.temperature;

        // Angle unit: rad
        const auto encoder_count = feedback.encoder;
        auto absolute_encoder = encoder_count - encoder_zero_point_;
        if (absolute_encoder < 0)
            absolute_encoder += raw_encoder_max_;

        if (!multi_turn_angle_enabled_)
            angle_ = encoder_to_angle_coefficient_ * static_cast<double>(absolute_encoder);
        else {
            auto diff = (absolute_encoder - multi_turn_encoder_count_) % raw_encoder_max_;
            if (diff <= -raw_encoder_max_ / 2)
                diff += raw_encoder_max_;
            else if (diff > raw_encoder_max_ / 2)
                diff -= raw_encoder_max_;
            multi_turn_encoder_count_ += diff;
            angle_ = encoder_to_angle_coefficient_ * static_cast<double>(multi_turn_encoder_count_);
        }

        // Velocity unit: rad/s
        const double raw_velocity = feedback.velocity;
        velocity_ = raw_to_velocity_coefficient_ * raw_velocity;

        // Torque unit: N*m
        torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(feedback.current);

        last_encoder_count_ = encoder_count;
    }

    int64_t calibrate_zero_point() {
        multi_turn_encoder_count_ = 0;
        encoder_zero_point_ = last_encoder_count_;
        return encoder_zero_point_;
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return torque_max_; }
    int8_t temperature() const { return temperature_; }

    /// @brief 将电机从开启状态（上电后默认状态）切换到关闭状态，清除电机转动圈数及之
    /// 前接收的控制指令，LED由常亮转为慢闪。此时电机仍然可以回复控制命令，但不会执行动作。
    static inline uint64_t generate_shutdown_command() {
        struct __attribute__((packed)) {
            uint8_t id;
            uint8_t null[7];
        } command{.id = 0x80};
        return std::bit_cast<uint64_t>(command);
    }

    /// @brief 停止电机，但不清除电机运行状态。再次发送控制指令即可控制电机动作。
    static inline uint64_t generate_pause_command() {
        struct __attribute__((packed)) {
            uint8_t id;
            uint8_t null[7];
        } command{.id = 0x81};
        return std::bit_cast<uint64_t>(command);
    }

    /// @brief 将电机从关闭状态切换到开启状态，LED
    /// 由慢闪转为常亮。此时再发送控制指令即可控制电机动作。
    static inline uint64_t generate_enable_command() {
        struct __attribute__((packed)) {
            uint8_t id;
            uint8_t null[7];
        } command{.id = 0x88};
        return std::bit_cast<uint64_t>(command);
    }

    /// @brief 该命令读取当前电机的温度、电机转矩电流（MF、MG）/
    /// 电机输出功率（MS）、转速、编码器位置。
    static inline uint64_t generate_status_request() {
        struct __attribute__((packed)) {
            uint8_t id;
            uint8_t null[7];
        } request{.id = 0x9C};
        return std::bit_cast<uint64_t>(request);
    }

    /// @brief 主机发送该命令以控制电机的转矩电流输出
    /// @note 电机在收到命令后回复主机。电机回复数据和读取电机状态 2 命令相同（仅命令字节
    /// DATA[0]不同，这里为 0xA1）。
    uint64_t generate_torque_command(double torque) const {
        if (std::isnan(torque))
            return 0;

        const auto current = std::round(
            torque_to_raw_current_coefficient_ * std::clamp(torque, -torque_max_, torque_max_));
        const int16_t current_bit = static_cast<int16_t>(current);

        /// @param current 数值范围-2048~ 2048，对应 MF 电机实际转矩电流范围-16.5A~16.5A，对应 MG
        /// 电机实际转矩电流范围-33A~33A，母线电流和电机的实际扭矩因不同电机而异。
        struct __attribute__((packed)) {
            uint8_t id;
            uint8_t null0[3];
            int16_t current;
            uint8_t null1[2];
        } command{.id = 0xA1, .current = current_bit};

        return std::bit_cast<uint64_t>(command);
    }

    /// @brief 主机发送该命令以控制电机的速度，同时带有力矩限制。
    /// @note 电机在收到命令后回复主机。电机回复数据和读取电机状态 2 命令相同（仅命令字节
    /// DATA[0]不同，这里为 0xA2/0xAD）。
    uint64_t generate_velocity_command(double velocity, double torque = 0) const {
        if (std::isnan(velocity) || std::isnan(torque))
            return 0;

        const auto torque_bit = static_cast<int16_t>(torque * torque_to_raw_current_coefficient_);
        const auto velocity_bit =
            static_cast<int32_t>(velocity * 18000 / std::numbers::pi * velocity_reduction_ratio_);
        const auto id = static_cast<uint8_t>(torque == 0 ? 0xA2 : 0xAD);

        /// @param torque_limit int16_t类型，，数值范围2048~2048，
        /// 对应MF电机实际转矩电流范围-16.5A~16.5A，
        /// 对应MG电机实际转矩电流范围-33.0A~33.0A，
        /// 母线电流和电机的实际扭矩因不同电机而异。
        /// @param velocity int32_t类型，对应实际转速为0.01dps/LSB；
        struct __attribute__((packed)) {
            uint8_t id;
            uint8_t null;
            int16_t torque_limit;
            int32_t velocity;
        } command{.id = id, .torque_limit = torque_bit, .velocity = velocity_bit};

        return std::bit_cast<uint64_t>(command);
    }

    /// @brief 主机发送该命令以控制电机的位置（多圈角度）。
    /// @note 电机在收到命令后回复主机。电机回复数据和读取电机状态 2 命令相同（仅命令字节
    /// DATA[0]不同，这里为 0xA3/0xA4）。
    uint64_t generate_angle_command(double angle, double velocity = 0) const {
        if (std::isnan(angle) || std::isnan(velocity))
            return 0;

        const auto angle_bit = static_cast<int32_t>(
            angle / std::numbers::pi * 180. * 100. * velocity_reduction_ratio_);
        const auto velocity_bit =
            static_cast<uint16_t>(velocity * 18000 / std::numbers::pi * velocity_reduction_ratio_);
        const auto id = static_cast<uint8_t>(velocity == 0 ? 0xA3 : 0xA4);

        /// @param angle 对应实际位置为 0.01degree/LSB，即36000代表
        /// 360°，电机转动方向由目标位置和当前位置的差值决定。
        /// @param velocity 限制电机转动的最大速度，对应实际转速 1dps/LSB，即 360代表 360dps。
        struct __attribute__((packed)) {
            uint8_t id;
            uint8_t null;
            uint16_t velocity;
            int32_t encoder;
        } command{.id = id, .velocity = velocity_bit, .encoder = angle_bit};

        return std::bit_cast<uint64_t>(command);
    }

protected:
    // Constant and Limits
    uint16_t raw_encoder_max_;
    double raw_current_max_;
    double real_current_max_;
    double torque_max_;

    double velocity_reduction_ratio_;
    double torque_constant_;
    bool multi_turn_angle_enabled_;
    int64_t encoder_zero_point_;

    // Coefficients
    double encoder_to_angle_coefficient_;
    double angle_to_encoder_coefficient_;

    double raw_to_velocity_coefficient_;
    double velocity_to_raw_coefficient_;

    double raw_current_to_torque_coefficient_;
    double torque_to_raw_current_coefficient_;

    // Status
    int64_t multi_turn_encoder_count_;
    int64_t last_encoder_count_;

    int8_t temperature_;
    double angle_;
    double torque_;
    double velocity_;

    uint16_t can_id_ = 0x140;
    std::atomic<uint64_t> can_buffer_ = 0;
};

} // namespace librmcs::device