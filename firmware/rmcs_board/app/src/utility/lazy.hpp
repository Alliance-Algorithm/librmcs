#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <tuple>
#include <utility>

#include "core/src/utility/assert.hpp"
#include "firmware/rmcs_board/app/src/utility/interrupt_lock.hpp"

namespace librmcs::firmware::utility {

// Lazy-initialized object with deferred construction.
// Thread-safe single-time initialization guarded by interrupt lock.
template <typename T, typename... Args>
class Lazy {
public:
    consteval explicit Lazy(Args... args)
        : init_status_(InitStatus::kUninitialized)
        , construction_arguments_{std::move(args)...} {}

    Lazy(const Lazy&) = delete;
    Lazy& operator=(const Lazy&) = delete;
    Lazy(Lazy&&) = delete;
    Lazy& operator=(Lazy&&) = delete;

    constexpr ~Lazy() {} // No need to deconstruct

    /*!
     * @brief Construct the object on first call; no-op on subsequent calls.
     * @return Reference to the constructed object
     * @note Thread-safe. Uses interrupt lock for atomic state transitions.
     */
    constexpr T& init() {
        const InterruptLockGuard guard;

        auto init_status = init_status_.load(std::memory_order::relaxed);
        if (init_status != InitStatus::kInitialized) {
            core::utility::assert_always(init_status == InitStatus::kUninitialized);
            init_status_.store(InitStatus::kInitializing, std::memory_order::relaxed);

            auto moved_args = std::move(construction_arguments_);
            std::destroy_at(std::addressof(construction_arguments_));

            construct_object(std::move(moved_args));

            init_status_.store(InitStatus::kInitialized, std::memory_order::relaxed);
        }

        return object_;
    }

    /*!
     * @brief Get pointer to the constructed object, or nullptr if not yet
     *        initialized.
     * @return Pointer to the object, or nullptr if not yet initialized
     * @note Does not assert on uninitialized state. Check return before use.
     */
    constexpr T* try_get() {
        if (!static_cast<bool>(*this))
            return nullptr;
        return std::addressof(object_);
    }

    /*!
     * @brief Get pointer to the constructed object
     * @return Pointer to the constructed object
     * @warning Must only be called after init(). Asserts if uninitialized
     *          in debug builds.
     */
    constexpr T* get() {
        core::utility::assert_debug(static_cast<bool>(*this));
        return std::addressof(object_);
    }

    /*!
     * @brief Member access through the constructed object
     * @return Pointer to the constructed object
     * @warning Must only be called after init(). Asserts if uninitialized
     *          in debug builds.
     */
    constexpr T* operator->() {
        core::utility::assert_debug(static_cast<bool>(*this));
        return std::addressof(object_);
    }

    /*!
     * @brief Dereference to access the constructed object
     * @return Reference to the constructed object
     * @warning Must only be called after init(). Asserts if uninitialized
     *          in debug builds.
     */
    constexpr T& operator*() {
        core::utility::assert_debug(static_cast<bool>(*this));
        return object_;
    }

    constexpr explicit operator bool() const noexcept {
        return init_status_.load(std::memory_order::relaxed) == InitStatus::kInitialized;
    }

private:
    using ArgTupleT = std::tuple<Args...>;

    template <typename TupleT>
    constexpr void construct_object(TupleT&& t) {
        std::apply(
            [this](auto&&... args) {
                std::construct_at(std::addressof(object_), std::forward<decltype(args)>(args)...);
            },
            std::forward<TupleT>(t));
    }

    enum class InitStatus : uint8_t { kUninitialized = 2, kInitializing = 1, kInitialized = 0 };
    std::atomic<InitStatus> init_status_;

    union {
        T object_;
        ArgTupleT construction_arguments_;
    };
};

} // namespace librmcs::firmware::utility
