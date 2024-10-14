#pragma once

#include <cstring>

#include <atomic>
#include <bit>

namespace rmcs::device {

class Dr16 {
public:
    explicit Dr16() = default;

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != 6 + 8 + 4)
            return;

        // Avoid using reinterpret_cast here because it does not account for pointer alignment.
        // Dr16DataPart structures are aligned, and using reinterpret_cast on potentially unaligned
        // uart_data can cause undefined behavior on architectures that enforce strict alignment
        // requirements (e.g., ARM).
        // Directly accessing unaligned memory through a casted pointer can lead to crashes,
        // inefficiencies, or incorrect data reads. Instead, std::memcpy safely copies the data from
        // unaligned memory to properly aligned structures without violating alignment or strict
        // aliasing rules.

        Dr16DataPart1 part1;
        std::memcpy(&part1, uart_data, 6);
        uart_data += 6;
        data_part1_.store(part1, std::memory_order::relaxed);

        Dr16DataPart2 part2;
        std::memcpy(&part2, uart_data, 8);
        uart_data += 8;
        data_part2_.store(part2, std::memory_order::relaxed);

        Dr16DataPart3 part3;
        std::memcpy(&part3, uart_data, 4);
        uart_data += 4;
        data_part3_.store(part3, std::memory_order::relaxed);
    }

    void update_status() {
        auto part1 = data_part1_.load(std::memory_order::relaxed);

        auto channel_to_double = [](uint16_t value) {
            return (static_cast<int32_t>(value) - 1024) / 660.0;
        };
        joystick_right_.y = -channel_to_double(part1.joystick_channel0);
        joystick_right_.x = channel_to_double(part1.joystick_channel1);
        joystick_left_.y = -channel_to_double(part1.joystick_channel2);
        joystick_left_.x = channel_to_double(part1.joystick_channel3);

        switch_right_ = part1.switch_right;
        switch_left_ = part1.switch_left;

        auto part2 = data_part2_.load(std::memory_order::relaxed);

        mouse_velocity_.x = -part2.mouse_velocity_y / 32768.0;
        mouse_velocity_.y = -part2.mouse_velocity_x / 32768.0;

        mouse_.left = part2.mouse_left;
        mouse_.right = part2.mouse_right;

        auto part3 = data_part3_.load(std::memory_order::relaxed);

        keyboard_ = part3.keyboard;
    }

    struct Vector {
        constexpr static inline Vector zero() { return {0, 0}; }
        double x, y;
    };

    enum class Switch : uint8_t { UNKNOWN = 0, UP = 1, DOWN = 2, MIDDLE = 3 };

    struct __attribute__((packed)) Mouse {
        constexpr static inline Mouse zero() {
            constexpr uint8_t zero = 0;
            return std::bit_cast<Mouse>(zero);
        }

        bool left  : 1;
        bool right : 1;
    };

    struct __attribute__((packed)) Keyboard {
        constexpr static inline Keyboard zero() {
            constexpr uint16_t zero = 0;
            return std::bit_cast<Keyboard>(zero);
        }

        bool w     : 1;
        bool s     : 1;
        bool a     : 1;
        bool d     : 1;
        bool shift : 1;
        bool ctrl  : 1;
        bool q     : 1;
        bool e     : 1;
        bool r     : 1;
        bool f     : 1;
        bool g     : 1;
        bool z     : 1;
        bool x     : 1;
        bool c     : 1;
        bool v     : 1;
        bool b     : 1;
    };

    Vector joystick_right() { return joystick_right_; }
    Vector joystick_left() { return joystick_left_; }

    Switch switch_right() { return switch_right_; }
    Switch switch_left() { return switch_left_; }

    Vector mouse_velocity() { return mouse_velocity_; }

    Mouse mouse() { return mouse_; }
    Keyboard keyboard() { return keyboard_; }

private:
    struct __attribute__((packed, aligned(8))) Dr16DataPart1 {
        uint16_t joystick_channel0 : 11;
        uint16_t joystick_channel1 : 11;
        uint16_t joystick_channel2 : 11;
        uint16_t joystick_channel3 : 11;

        Switch switch_right : 2;
        Switch switch_left  : 2;
    };
    std::atomic<Dr16DataPart1> data_part1_{
        Dr16DataPart1{1024, 1024, 1024, 1024, Switch::DOWN, Switch::DOWN}
    };
    static_assert(decltype(data_part1_)::is_always_lock_free);

    struct __attribute__((packed, aligned(8))) Dr16DataPart2 {
        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;

        bool mouse_left;
        bool mouse_right;
    };
    std::atomic<Dr16DataPart2> data_part2_{
        Dr16DataPart2{0, 0, 0, false, false}
    };
    static_assert(decltype(data_part2_)::is_always_lock_free);

    struct __attribute__((packed, aligned(4))) Dr16DataPart3 {
        Keyboard keyboard;
        uint16_t unused;
    };
    std::atomic<Dr16DataPart3> data_part3_ = {
        Dr16DataPart3{Keyboard::zero(), 0}
    };
    static_assert(decltype(data_part3_)::is_always_lock_free);

    Vector joystick_right_ = Vector::zero();
    Vector joystick_left_ = Vector::zero();

    Switch switch_right_ = Switch::UNKNOWN;
    Switch switch_left_ = Switch::UNKNOWN;

    Vector mouse_velocity_ = Vector::zero();

    Mouse mouse_ = Mouse::zero();
    Keyboard keyboard_ = Keyboard::zero();
};

} // namespace rmcs::device