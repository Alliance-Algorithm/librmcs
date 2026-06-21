#pragma once

#include <functional>
#include <type_traits>
#include <utility>

namespace librmcs::board {

/**
 * @brief Advanced transport options passed during board construction.
 *
 * `bind_advanced_options()` returns an object derived from `AdvancedOptions` whose `thread_setup`
 * callback depends on state stored in that derived object. Copying or moving the base type would
 * slice away that state while retaining a now-dangling function pointer, so `AdvancedOptions` is
 * intentionally non-copyable and non-movable.
 *
 * Construct `AdvancedOptions` directly during board construction, or explicitly copy the required
 * plain data fields into a fresh `AdvancedOptions` instance instead of copying an existing object.
 *
 * @warning Never copy, assign, or reuse any function pointer from an object returned by
 * `bind_advanced_options()` in any other `AdvancedOptions` instance; doing so is undefined
 * behavior.
 */
class AdvancedOptions {
public:
    AdvancedOptions() = default;

    AdvancedOptions(const AdvancedOptions&) = delete;
    AdvancedOptions& operator=(const AdvancedOptions&) = delete;
    AdvancedOptions(AdvancedOptions&&) = delete;
    AdvancedOptions& operator=(AdvancedOptions&&) = delete;

    ~AdvancedOptions() = default;

    bool dangerously_skip_version_checks = false;

    /**
     * @brief Callback invoked on the transport event thread before transport I/O handling begins.
     *
     * This hook is intended only for per-thread environment setup, such as thread priority,
     * CPU affinity, thread naming, or other OS-level thread configuration.
     *
     * @warning This callback runs during transport construction, before the enclosing board object
     * finishes construction. If that board is itself a member of another object, that enclosing
     * object may also still be under construction.
     * @warning The callback must not access the board object being constructed, any state whose
     * lifetime depends on construction having finished, or any transport/protocol APIs. In
     * particular, do not capture and use `this` from an object that is still being constructed.
     */
    void (*thread_setup)(const AdvancedOptions&) noexcept = nullptr;

    AdvancedOptions& set_dangerously_skip_version_checks(bool value) {
        dangerously_skip_version_checks = value;
        return *this;
    }

    AdvancedOptions& set_thread_setup(void (*value)(const AdvancedOptions&) noexcept) {
        thread_setup = value;
        return *this;
    }
};

/**
 * @brief Binds callable to `AdvancedOptions`.
 *
 * @warning The returned object must outlive any use of its function pointers.
 * @warning Do not store, copy, move, or slice the returned object as `AdvancedOptions`.
 */
template <typename FunctorT>
requires std::is_nothrow_invocable_v<const std::decay_t<FunctorT>&>
auto bind_advanced_options(FunctorT&& thread_setup_impl) {
    using Functor = std::decay_t<FunctorT>;

    class OptionsImpl : public AdvancedOptions {
    public:
        explicit OptionsImpl(Functor thread_setup_impl)
            : thread_setup_impl_(std::move(thread_setup_impl)) {
            thread_setup = [](const AdvancedOptions& self) noexcept {
                std::invoke(static_cast<const OptionsImpl&>(self).thread_setup_impl_);
            };
        }

    private:
        Functor thread_setup_impl_;
    };

    return OptionsImpl{std::forward<FunctorT>(thread_setup_impl)};
}

} // namespace librmcs::board
