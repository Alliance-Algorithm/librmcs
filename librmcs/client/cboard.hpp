#pragma once

#include <cinttypes>

#include <atomic>
#include <bit>
#include <chrono>
#include <stdexcept>

#include <libusb.h>

#include "../utility/cross_os.hpp"
#include "../utility/logging.hpp"
#include "../utility/ring_buffer.hpp"

namespace librmcs::client {

class CBoard {
public:
    explicit CBoard(uint16_t usb_pid) {
        if (!init(0xa11c, usb_pid)) {
            throw std::runtime_error{"Failed to init usb transfer for cboard, see log for detail."};
        }
    }

    virtual ~CBoard() {
        libusb_free_transfer(libusb_receive_transfer_);
        libusb_release_interface(libusb_device_handle_, target_interface);
        libusb_close(libusb_device_handle_);
        libusb_exit(libusb_context_);
    }

    void handle_events() {
        auto ret = libusb_submit_transfer(libusb_receive_transfer_);
        if (ret != 0) {
            if (ret == LIBUSB_ERROR_NO_DEVICE)
                LOG_ERROR("Failed to submit receive transfer: Device disconnected. "
                          "Terminating...");
            else
                LOG_ERROR("Failed to submit receive transfer: %d. Terminating...", ret);
            std::terminate();
            return;
        }

        handling_events_.store(true, std::memory_order::relaxed);
        receive_transfer_busy_ = true;
        while (receive_transfer_busy_) {
            libusb_handle_events(libusb_context_);
        }
    }

    void stop_handling_events() { handling_events_.store(false, std::memory_order::relaxed); }

protected:
    virtual void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) {
        (void)can_id;
        (void)can_data;
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;
    };
    virtual void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) {
        (void)can_id;
        (void)can_data;
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;
    }

    virtual void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) {
        (void)uart_data;
        (void)uart_data_length;
    };
    virtual void uart2_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) {
        (void)uart_data;
        (void)uart_data_length;
    }
    virtual void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) {
        (void)uart_data;
        (void)uart_data_length;
    }

    virtual void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) {
        (void)x;
        (void)y;
        (void)z;
    }
    virtual void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) {
        (void)x;
        (void)y;
        (void)z;
    }

    class TransmitBuffer;

private:
    bool init(uint16_t vendor_id, uint16_t product_id) noexcept {
        int ret;

        ret = libusb_init(&libusb_context_);
        if (ret != 0) {
            LOG_ERROR("Failed to init libusb: %d", ret);
            return false;
        }
        FinalAction exit_libusb{[this]() { libusb_exit(libusb_context_); }};

        libusb_device_handle_ =
            libusb_open_device_with_vid_pid(libusb_context_, vendor_id, product_id);
        if (!libusb_device_handle_) {
            LOG_ERROR("Failed to open device (vid=0x%x, pid=0x%x)", vendor_id, product_id);
            return false;
        }
        FinalAction close_device_handle{[this]() { libusb_close(libusb_device_handle_); }};

        if constexpr (utility::is_linux()) {
            ret = libusb_set_auto_detach_kernel_driver(libusb_device_handle_, true);
            if (ret != 0) {
                LOG_ERROR("Failed to set auto detach kernel driver: %d", ret);
                return false;
            }
        }

        ret = libusb_claim_interface(libusb_device_handle_, target_interface);
        if (ret != 0) {
            LOG_ERROR("Failed to claim interface: %d", ret);
            return false;
        }
        FinalAction release_interface{
            [this]() { libusb_release_interface(libusb_device_handle_, target_interface); }};

        libusb_receive_transfer_ = libusb_alloc_transfer(0);
        if (!libusb_receive_transfer_) {
            LOG_ERROR("Failed to alloc receive transfer");
            return false;
        }

        libusb_fill_bulk_transfer(
            libusb_receive_transfer_, libusb_device_handle_, in_endpoint,
            reinterpret_cast<unsigned char*>(receive_buffer_), 64,
            [](libusb_transfer* transfer) {
                static_cast<CBoard*>(transfer->user_data)->usb_receive_complete_callback(transfer);
            },
            this, 0);

        // Libusb successfully initialized.
        release_interface.disable();
        close_device_handle.disable();
        exit_libusb.disable();
        return true;
    }

    void usb_receive_complete_callback(libusb_transfer* transfer) {
        if (!handling_events_.load(std::memory_order::relaxed)) [[unlikely]] {
            receive_transfer_busy_ = false;
            return;
        }
        FinalAction resubmit_transfer{[transfer]() {
            int ret = libusb_submit_transfer(transfer);
            if (ret != 0) [[unlikely]] {
                if (ret == LIBUSB_ERROR_NO_DEVICE)
                    LOG_ERROR("Failed to re-submit receive transfer: Device disconnected. "
                              "Terminating...");
                else
                    LOG_ERROR("Failed to re-submit receive transfer: %d. Terminating...", ret);
                std::terminate();
            }
        }};

        auto iterator = receive_buffer_;
        auto sentinel = iterator + transfer->actual_length;
        if (iterator == sentinel) [[unlikely]]
            return;

        FinalAction debug_print_buffer{[this, transfer]() {
            char hex_string[64 * 3 + 1];
            for (int i = 0; i < transfer->actual_length; i++)
                sprintf(&hex_string[i * 3], "%02x ", static_cast<uint8_t>(receive_buffer_[i]));
            LOG_ERROR("Buffer (len=%d): %s", transfer->actual_length, hex_string);
        }};

        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) [[unlikely]] {
            LOG_ERROR("USB receiving error: Transfer not completed! status=%d", transfer->status);
            return;
        }

        if (*iterator != std::byte{0xAE}) [[unlikely]] {
            LOG_ERROR(
                "USB receiving error: Unexpected header: 0x%02x!", static_cast<uint8_t>(*iterator));
            return;
        }

        ++iterator;
        if (iterator == sentinel) [[unlikely]] {
            LOG_ERROR("USB receiving error: Package without body!");
            return;
        }

        while (iterator < sentinel) {
            PACKED_STRUCT(Header { StatusId field_id : 4; });
            static_assert(sizeof(Header) == 1);

            auto field_id = std::bit_cast<Header>(*iterator).field_id;

            if (field_id == StatusId::CAN1) {
                read_can_buffer(iterator, &CBoard::can1_receive_callback);
            } else if (field_id == StatusId::CAN2) {
                read_can_buffer(iterator, &CBoard::can2_receive_callback);
            } else if (field_id == StatusId::UART1) {
                read_uart_buffer(iterator, &CBoard::uart1_receive_callback);
            } else if (field_id == StatusId::UART2) {
                read_uart_buffer(iterator, &CBoard::uart2_receive_callback);
            } else if (field_id == StatusId::UART3) {
                read_uart_buffer(iterator, &CBoard::dbus_receive_callback);
            } else if (field_id == StatusId::IMU) {
                read_imu_buffer(iterator);
            } else
                break;
        }

        if (iterator != sentinel) [[unlikely]] {
            if (iterator < sentinel) {
                LOG_ERROR(
                    "USB receiving error: Unexpected field-id: [%" PRIdPTR "] %d!",
                    iterator - receive_buffer_, static_cast<uint8_t>(*iterator) & 0xF);
            } else {
                LOG_ERROR(
                    "USB receiving error: Field reading out-of-bounds! (iterator = sentinel "
                    "+ %" PRIdPTR ")",
                    iterator - sentinel);
            }
            return;
        }

        debug_print_buffer.disable();
    }

    void read_can_buffer(std::byte*& buffer, decltype(&CBoard::can1_receive_callback) callback) {
        bool is_extended_can_id;
        bool is_remote_transmission;
        uint32_t can_data_length;
        uint32_t can_id;
        uint64_t can_data;

        auto& header = *std::launder(reinterpret_cast<const CanFieldHeader*>(buffer));
        buffer += sizeof(header);

        is_extended_can_id = header.is_extended_can_id;
        is_remote_transmission = header.is_remote_transmission;
        if (is_extended_can_id) {
            auto& ext_id = *std::launder(reinterpret_cast<const CanExtendedId*>(buffer));
            buffer += sizeof(ext_id);
            can_id = ext_id.can_id;
            can_data_length = header.has_can_data ? ext_id.data_length + 1 : 0;
        } else {
            auto& std_id = *std::launder(reinterpret_cast<const CanStandardId*>(buffer));
            buffer += sizeof(std_id);
            can_id = std_id.can_id;
            can_data_length = header.has_can_data ? std_id.data_length + 1 : 0;
        }

        std::memcpy(&can_data, buffer, can_data_length);
        buffer += can_data_length;

        (this->*callback)(
            can_id, can_data, is_extended_can_id, is_remote_transmission,
            static_cast<uint8_t>(can_data_length));
    }

    void read_uart_buffer(std::byte*& buffer, decltype(&CBoard::uart1_receive_callback) callback) {
        auto& header = *std::launder(reinterpret_cast<UartFieldHeader*>(buffer));
        buffer += sizeof(UartFieldHeader);

        uint8_t size = header.data_size;
        if (!size)
            size = static_cast<uint8_t>(*buffer++);

        (this->*callback)(buffer, size);
        buffer += size;
    }

    void read_imu_buffer(std::byte*& buffer) {
        auto& field = *std::launder(reinterpret_cast<ImuField*>(buffer));
        buffer += sizeof(ImuField);

        if (field.device_id == ImuField::DeviceId::ACCELEROMETER) {
            accelerometer_receive_callback(field.x, field.y, field.z);
        } else {
            gyroscope_receive_callback(field.x, field.y, field.z);
        }
    }

    enum class StatusId : uint8_t {
        RESERVED = 0,

        GPIO = 1,

        CAN1 = 2,
        CAN2 = 3,
        CAN3 = 4,

        UART1 = 5,
        UART2 = 6,
        UART3 = 7,
        UART4 = 8,
        UART5 = 9,
        UART6 = 10,

        IMU = 11,
    };

    enum class CommandId : uint8_t {
        RESERVED_ = 0,

        GPIO = 1,

        CAN1 = 2,
        CAN2 = 3,
        CAN3 = 4,

        UART1 = 5,
        UART2 = 6,
        UART3 = 7,
        UART4 = 8,
        UART5 = 9,
        UART6 = 10,

        LED = 11,
        BUZZER = 12,
    };

    PACKED_STRUCT(CanFieldHeader {
        uint8_t field_id            : 4;
        bool is_extended_can_id     : 1;
        bool is_remote_transmission : 1;
        bool has_can_data           : 1;
    });
    static_assert(sizeof(CanFieldHeader) == 1);

    PACKED_STRUCT(CanStandardId {
        uint16_t can_id      : 11;
        uint16_t data_length : 3;
    };)
    static_assert(sizeof(CanStandardId) == 2);

    PACKED_STRUCT(CanExtendedId {
        uint32_t can_id      : 29;
        uint32_t data_length : 3;
    });
    static_assert(sizeof(CanExtendedId) == 4);

    PACKED_STRUCT(UartFieldHeader {
        uint8_t field_id  : 4;
        uint8_t data_size : 4;
    });
    static_assert(sizeof(UartFieldHeader) == 1);

    PACKED_STRUCT(ImuField {
        uint8_t field_id : 4;
        enum class DeviceId : uint8_t {
            ACCELEROMETER = 0,
            GYROSCOPE = 1,
        } device_id : 4;
        int16_t x, y, z;
    });
    static_assert(sizeof(ImuField) == 7);

    template <typename Functor>
    struct FinalAction {
        constexpr explicit FinalAction(Functor clean)
            : clean_{clean}
            , enabled_(true) {}

        constexpr FinalAction(const FinalAction&) = delete;
        constexpr FinalAction& operator=(const FinalAction&) = delete;
        constexpr FinalAction(FinalAction&&) = delete;
        constexpr FinalAction& operator=(FinalAction&&) = delete;

        ~FinalAction() {
            if (enabled_)
                clean_();
        }

        void disable() { enabled_ = false; };

    private:
        Functor clean_;
        bool enabled_;
    };

    static constexpr int target_interface = 0x01;

    static constexpr unsigned char out_endpoint = 0x01;
    static constexpr unsigned char in_endpoint = 0x81;

    libusb_context* libusb_context_;
    libusb_device_handle* libusb_device_handle_;

    libusb_transfer* libusb_receive_transfer_;
    std::byte receive_buffer_[64];

    std::atomic<bool> handling_events_ = false;
    bool receive_transfer_busy_ = false;
};

class CBoard::TransmitBuffer final {
public:
    explicit TransmitBuffer(CBoard& cboard, size_t alloc_transfer_count)
        : cboard_(cboard)
        , free_transfers_(alloc_transfer_count)
        , alloc_transfer_count_(alloc_transfer_count) {
        free_transfers_.push_back_multi(
            [this, &cboard]() {
                auto transfer = libusb_alloc_transfer(0);
                if (!transfer)
                    throw std::bad_alloc{};
                libusb_fill_bulk_transfer(
                    transfer, cboard.libusb_device_handle_, out_endpoint, new unsigned char[64], 1,
                    [](libusb_transfer* transfer) {
                        static_cast<TransmitBuffer*>(transfer->user_data)
                            ->usb_transmit_complete_callback(transfer);
                    },
                    this, 0);
                transfer->flags = libusb_transfer_flags::LIBUSB_TRANSFER_FREE_BUFFER;
                transfer->buffer[0] = 0x81;
                return transfer;
            },
            alloc_transfer_count);
    }

    ~TransmitBuffer() {
        size_t unreleased_transfer_count = alloc_transfer_count_;
        timeval timeout{0, 1000'000};
        auto start = std::chrono::steady_clock::now();
        while (true) {
            unreleased_transfer_count -= free_transfers_.pop_front_multi(
                [&](libusb_transfer* transfer) { libusb_free_transfer(transfer); });

            // Break when all transfer released
            if (!unreleased_transfer_count)
                break;

            int ret;
            // Otherwise, handle events to allow other transfers to return to the queue
            if constexpr (utility::is_linux()) {
                // Set a 1s timeout to avoid stuck here (logically impossible, but just in case)
                ret = libusb_handle_events_timeout(cboard_.libusb_context_, &timeout);
            } else {
                // Windows does not support timeout
                ret = libusb_handle_events(cboard_.libusb_context_);
            }
            if (ret != 0) {
                LOG_ERROR(
                    "Fatal error during TransmitBuffer destruction: The function "
                    "libusb_handle_events returned an exception value: %d, which means we "
                    "cannot release all memory allocated for transfers.",
                    ret);
            } else if (std::chrono::steady_clock::now() - start > std::chrono::seconds(1)) {
                LOG_ERROR(
                    "Fatal error during TransmitBuffer destruction: The usb transmit complete "
                    "callback was not called for all transfers, which means we cannot release "
                    "all memory allocated for transfers.");
            } else
                continue;

            LOG_ERROR("The destructor will exit normally, but the unrecoverable memory leak "
                      "has already occurred. This may be a problem caused by libusb.");
            LOG_ERROR("Number of leaked transfers: %zu", unreleased_transfer_count);
            break;
        }
    }

    bool add_can1_transmission(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id = false,
        bool is_remote_transmission = false, uint8_t can_data_length = 8) {
        return add_can_transmission(
            CommandId::CAN1, can_id, can_data, is_extended_can_id, is_remote_transmission,
            can_data_length);
    }

    bool add_can2_transmission(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id = false,
        bool is_remote_transmission = false, uint8_t can_data_length = 8) {
        return add_can_transmission(
            CommandId::CAN2, can_id, can_data, is_extended_can_id, is_remote_transmission,
            can_data_length);
    }

    bool add_uart1_transmission(const std::byte* uart_data, uint8_t uart_data_length) {
        return add_uart_transmission(CommandId::UART1, uart_data, uart_data_length);
    }

    bool add_uart2_transmission(const std::byte* uart_data, uint8_t uart_data_length) {
        return add_uart_transmission(CommandId::UART2, uart_data, uart_data_length);
    }

    bool add_dbus_transmission(const std::byte* uart_data, uint8_t uart_data_length) {
        return add_uart_transmission(CommandId::UART3, uart_data, uart_data_length);
    }

    bool trigger_transmission() {
        auto front = free_transfers_.front();
        if (!front || (*front)->length <= 1)
            return false;

        return trigger_transmission_nocheck();
    }

private:
    bool add_can_transmission(
        CommandId field_id, uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
        bool is_remote_transmission, uint8_t can_data_length) {

        std::byte* buffer = try_fetch_buffer(
            sizeof(CanFieldHeader)
            + (is_extended_can_id ? sizeof(CanExtendedId) : sizeof(CanStandardId))
            + can_data_length);
        if (!buffer)
            return false;

        // Write field header
        auto& header = *new (buffer) CanFieldHeader{};
        buffer += sizeof(CanFieldHeader);
        header.field_id = static_cast<uint8_t>(field_id);
        header.is_extended_can_id = is_extended_can_id;
        header.is_remote_transmission = is_remote_transmission;
        header.has_can_data = static_cast<bool>(can_data_length);

        // Write CAN id and data length
        if (is_extended_can_id) {
            auto& ext_id = *new (buffer) CanExtendedId{};
            buffer += sizeof(CanExtendedId);
            ext_id.can_id = can_id;
            ext_id.data_length = can_data_length - 1;
        } else {
            auto& std_id = *new (buffer) CanStandardId{};
            buffer += sizeof(CanStandardId);
            std_id.can_id = can_id;
            std_id.data_length = can_data_length - 1;
        }

        // Write CAN data
        std::memcpy(buffer, &can_data, can_data_length);
        buffer += can_data_length;

        return true;
    }

    bool add_uart_transmission(
        CommandId field_id, const std::byte* uart_data, uint8_t uart_data_length) {

        std::byte* buffer =
            try_fetch_buffer(sizeof(UartFieldHeader) + (uart_data_length > 15) + uart_data_length);
        if (!buffer)
            return false;

        // Write field header
        auto& header = *new (buffer) UartFieldHeader{};
        buffer += sizeof(UartFieldHeader);
        header.field_id = static_cast<uint8_t>(field_id);
        if (uart_data_length <= 15) {
            // Store 4-bit size and field-id together
            header.data_size = uart_data_length;
        } else {
            // Store size to a separate byte
            header.data_size = 0;
            *(buffer++) = static_cast<std::byte>(uart_data_length);
        }

        // Write received data
        std::memcpy(buffer, uart_data, uart_data_length);
        buffer += uart_data_length;

        return true;
    }

    std::byte* try_fetch_buffer(size_t size) {
        if (1 + size > 64) [[unlikely]]
            return nullptr;

        while (true) {
            auto front = free_transfers_.front();
            if (!front) [[unlikely]] {
                if (!transfers_all_busy_)
                    LOG_ERROR("Failed to fetch free buffer: All transfers are busy!");
                transfers_all_busy_ = true;
                return nullptr;
            } else
                transfers_all_busy_ = false;

            libusb_transfer* transfer = *front;

            if (transfer->length + size > 64)
                trigger_transmission_nocheck();
            else {
                std::byte* buffer =
                    reinterpret_cast<std::byte*>(transfer->buffer) + transfer->length;
                transfer->length += static_cast<int>(size);
                return buffer;
            }
        }
    }

    bool trigger_transmission_nocheck() {
        libusb_transfer* transfer = nullptr;

        if (!free_transfers_.pop_front([&transfer](libusb_transfer* t) { transfer = t; }))
            return false;

        // The transfer must be submitted to libusb only after the pop_front function returns.
        // Otherwise, there is a very slight chance that the callback might be invoked too quickly,
        // resulting in a false "ring queue full" condition when recycling transfer, which could
        // subsequently lead to transfer leaks.

        int ret = libusb_submit_transfer(transfer);
        if (ret != 0) [[unlikely]] {
            if (ret == LIBUSB_ERROR_NO_DEVICE)
                LOG_ERROR(
                    "Failed to submit transmit transfer: Device disconnected. Terminating...");
            else
                LOG_ERROR("Failed to submit transmit transfer: %d. Terminating...", ret);
            std::terminate();
        }

        return true;
    }

    void usb_transmit_complete_callback(libusb_transfer* transfer) {
        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) [[unlikely]] {
            LOG_ERROR(
                "USB transmitting error: Transfer not completed! status=%d", transfer->status);
        }

        if (transfer->actual_length != transfer->length) [[unlikely]]
            LOG_ERROR(
                "USB transmitting error: transmitted(%d) < expected(%d)", transfer->actual_length,
                transfer->length);

        transfer->length = 1;

        if (!free_transfers_.push_back(transfer)) [[unlikely]] {
            LOG_ERROR("Error while attempting to recycle transmit transfer into the ring queue: "
                      "The ring queue is full.");
            LOG_ERROR("This situation should theoretically be impossible. Its occurrence typically "
                      "indicates an issue with multithreaded synchronization in the code.");
            LOG_ERROR("Although this problem is not fatal, termination is triggered to ensure the "
                      "issue is promptly identified.");
            std::terminate();
        }
    }

    CBoard& cboard_;

    utility::RingBuffer<libusb_transfer*> free_transfers_;
    size_t alloc_transfer_count_;

    bool transfers_all_busy_ = false;
};

} // namespace librmcs::client