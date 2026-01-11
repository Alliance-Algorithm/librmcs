#pragma once

#include <cstddef>
#include <cstdint>

#include <algorithm>
#include <atomic>
#include <limits>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

namespace librmcs::firmware::utility {

// Lock-free Single-Producer/Single-Consumer (SPSC) ring buffer
// Inspired by Linux kfifo.
template <typename T, size_t max_size>
class RingBuffer {
public:
    using IndexType = std::conditional_t<
        (max_size <= std::numeric_limits<uint8_t>::max()), uint8_t,
        std::conditional_t<
            (max_size <= std::numeric_limits<uint16_t>::max()), uint16_t,
            std::conditional_t<
                (max_size <= std::numeric_limits<uint32_t>::max()), uint32_t, uint64_t>>>;

    static_assert(max_size >= 2, "RingBuffer size must be at least 2");
    static_assert((max_size & (max_size - 1)) == 0, "RingBuffer size must be a power of two");

    static constexpr size_t kMaxSize = max_size;
    static constexpr size_t kMask = max_size - 1;

    constexpr RingBuffer() = default;

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;
    RingBuffer(RingBuffer&&) = delete;
    RingBuffer& operator=(RingBuffer&&) = delete;

    /*!
     * @brief Destructor
     * Destroys all elements remaining in the buffer.
     */
    ~RingBuffer() { clear(); }

    /*!
     * @brief Number of elements currently readable
     * @return Count of elements available to the consumer
     * @note Uses acquire on producer index and relaxed on consumer index to
     *       ensure visibility of constructed elements to the consumer.
     */
    size_t readable() const {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);

        return static_cast<size_t>(static_cast<IndexType>(in - out));
    }

    /*!
     * @brief Number of free slots for producer
     * @return Count of slots available to write
     * @note Uses relaxed on producer index and acquire on consumer index to
     *       avoid overrun while allowing the producer to run without contention.
     */
    size_t writable() const {
        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        return kMaxSize - static_cast<size_t>(static_cast<IndexType>(in - out));
    }

    /*!
     * @brief Peek the first element (consumer side)
     * @return Pointer to the first element, or nullptr if empty
     * @warning Do not call from producer thread. The pointer remains valid
     *          until the element is popped or overwritten.
     */
    T* peek_front() {
        const auto out = out_.load(std::memory_order::relaxed);

        if (out == in_.load(std::memory_order::acquire))
            return nullptr;
        else
            return std::launder(reinterpret_cast<T*>(storage_[out & kMask].data));
    }

    /*!
     * @brief Peek the last produced element (consumer side)
     * @return Pointer to the last element, or nullptr if empty
     * @warning Do not call from producer thread. The pointer remains valid
     *          until the element is popped or overwritten.
     */
    T* peek_back() {
        const auto in = in_.load(std::memory_order::acquire);

        if (in == out_.load(std::memory_order::relaxed))
            return nullptr;
        else
            return std::launder(
                reinterpret_cast<T*>(storage_[(static_cast<size_t>(in) - 1) & kMask].data));
    }

    /*!
     * @brief Batch-construct elements at the tail (producer)
     * @tparam F Functor with signature `void(std::byte* storage)` that constructs
     *         a `T` in-place via placement-new.
     * @param count Maximum number of elements to construct (defaults to as many as fit)
     * @param fail_fast If true, this function constructs nothing and returns 0
     *        unless there is enough space for @p count elements. If false, it
     *        constructs as many elements as currently fit (up to @p count).
     * @return Number of elements actually constructed
     * @note Producer-only. Publishes with release semantics.
     */
    template <typename F>
    requires requires(F& f, std::byte* storage) {
        { f(storage) } noexcept;
    }
    size_t emplace_back_n(
        F construct_functor, size_t count = std::numeric_limits<size_t>::max(),
        bool fail_fast = false) {

        const auto in = in_.load(std::memory_order::relaxed);
        const auto out = out_.load(std::memory_order::acquire);

        const auto used = static_cast<IndexType>(in - out);
        const auto writable = kMaxSize - static_cast<size_t>(used);

        if (count > writable)
            count = fail_fast ? 0 : writable;
        if (!count)
            return 0;

        const auto offset = in & kMask;
        const auto slice = std::min(count, kMaxSize - offset);

        for (size_t i = 0; i < slice; i++)
            construct_functor(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            construct_functor(storage_[i].data);

        const auto count_index = static_cast<IndexType>(count);
        in_.store(static_cast<IndexType>(in + count_index), std::memory_order::release);

        return count;
    }

    /*!
     * @brief Construct one element in-place at the tail (producer)
     * @return true if pushed, false if buffer is full
     */
    template <typename... Args>
    bool emplace_back(Args&&... args) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{std::forward<Args>(args)...})) {
                new (storage) T{std::forward<Args>(args)...};
            },
            1);
    }

    /*!
     * @brief Batch-push using a generator (producer)
     * @tparam F Functor returning a `T` to be stored
     * @param count Maximum number to generate/push
     * @param fail_fast If true, this function pushes nothing and returns 0
     *        unless there is enough space for @p count elements. If false, it
     *        pushes as many elements as currently fit (up to @p count).
     * @return Number of elements actually pushed
     */
    template <typename F>
    requires requires(F& f) {
        { f() } noexcept;
        { T{f()} } noexcept;
    } size_t push_back_n(
        F generator, size_t count = std::numeric_limits<size_t>::max(), bool fail_fast = false) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{generator()})) {
                new (storage) T{generator()};
            },
            count, fail_fast);
    }

    /*!
     * @brief Push a copy of value (producer)
     * @return true if pushed, false if buffer is full
     */
    bool push_back(const T& value) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{value})) { new (storage) T{value}; }, 1);
    }
    /*!
     * @brief Push by moving value (producer)
     * @return true if pushed, false if buffer is full
     */
    bool push_back(T&& value) {
        return emplace_back_n(
            [&](std::byte* storage) noexcept(noexcept(T{std::move(value)})) {
                new (storage) T{std::move(value)};
            },
            1);
    }

    /*!
     * @brief Batch-pop elements from the head (consumer)
     * @tparam F Functor with signature `void(T)` receiving moved-out elements
     * @param count Maximum number of elements to pop (defaults to all available)
     * @return Number of elements actually popped
     * @note Consumer-only. Consumes with release on `out_` and destroys elements.
     */
    template <typename F>
    requires requires(F& f, T& t) {
        { f(std::move(t)) } noexcept;
    } size_t pop_front_n(F callback_functor, size_t count = std::numeric_limits<size_t>::max()) {
        const auto in = in_.load(std::memory_order::acquire);
        const auto out = out_.load(std::memory_order::relaxed);

        const auto readable = static_cast<size_t>(static_cast<IndexType>(in - out));
        if (count > readable)
            count = readable;
        if (!count)
            return 0;

        const auto offset = out & kMask;
        const auto slice = std::min(count, kMaxSize - offset);

        auto process = [&callback_functor](std::byte* storage) {
            auto& element = *std::launder(reinterpret_cast<T*>(storage));
            callback_functor(std::move(element));
            std::destroy_at(&element);
        };
        for (size_t i = 0; i < slice; i++)
            process(storage_[offset + i].data);
        for (size_t i = 0; i < count - slice; i++)
            process(storage_[i].data);

        const auto count_index = static_cast<IndexType>(count);
        out_.store(static_cast<IndexType>(out + count_index), std::memory_order::release);

        return count;
    }

    /*!
     * @brief Pop one element (consumer)
     * @return true if an element was popped, false if empty
     */
    template <typename F>
    requires requires(F& f, T& t) {
        { f(std::move(t)) } noexcept;
    } bool pop_front(F&& callback_functor) {
        return pop_front_n(std::forward<F>(callback_functor), 1);
    }

    /*!
     * @brief Clear the buffer by consuming all elements
     * @return Number of elements that were erased
     */
    size_t clear() {
        return pop_front_n([](T&&) noexcept {});
    }

private:
    struct {
        alignas(T) std::byte data[sizeof(T)];
    } storage_[max_size];

    std::atomic<IndexType> in_{0}, out_{0};
    static_assert(std::atomic<IndexType>::is_always_lock_free);
};

} // namespace librmcs::firmware::utility
