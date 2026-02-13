#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/src/utility/interrupt_lock_guard.hpp"

namespace librmcs::firmware::utility {

template <typename T, typename... Args>
class Lazy {
public:
    consteval explicit Lazy(Args... args)
        : init_status_(InitStatus::UNINITIALIZED)
        , construction_arguments{std::move(args)...} {}

    constexpr ~Lazy() {}; // No need to deconstruct
    Lazy(const Lazy&) = delete;
    Lazy& operator=(const Lazy&) = delete;
    Lazy(Lazy&&) = delete;
    Lazy& operator=(Lazy&&) = delete;

    constexpr T& init() {
        const InterruptLockGuard guard;

        auto init_status = init_status_.load(std::memory_order::relaxed);
        if (init_status != InitStatus::INITIALIZED) {
            core::utility::assert_always(init_status == InitStatus::UNINITIALIZED);
            init_status_.store(InitStatus::INITIALIZING, std::memory_order::relaxed);

            auto moved_args = std::move(construction_arguments);
            std::destroy_at(std::addressof(construction_arguments));

            construct_object(std::move(moved_args));

            init_status_.store(InitStatus::INITIALIZED, std::memory_order::relaxed);
        }

        return object;
    }

    constexpr T* get() {
        core::utility::assert_debug(static_cast<bool>(*this));
        return std::addressof(object);
    }

    constexpr T* operator->() {
        core::utility::assert_debug(static_cast<bool>(*this));
        return std::addressof(object);
    }

    constexpr T& operator*() {
        core::utility::assert_debug(static_cast<bool>(*this));
        return object;
    }

    constexpr explicit operator bool() const noexcept {
        return init_status_.load(std::memory_order::relaxed) == InitStatus::INITIALIZED;
    }

private:
    using ArgTupleT = std::tuple<Args...>;

    template <typename TupleT>
    constexpr void construct_object(TupleT&& t) {
        std::apply(
            [this](auto&&... args) {
                std::construct_at(std::addressof(object), std::forward<decltype(args)>(args)...);
            },
            std::forward<TupleT>(t));
    }

    enum class InitStatus : uint8_t { UNINITIALIZED = 2, INITIALIZING = 1, INITIALIZED = 0 };
    std::atomic<InitStatus> init_status_;

    union {
        T object;
        ArgTupleT construction_arguments;
    };
};

} // namespace librmcs::firmware::utility
