#pragma once

#include <cstddef>
#include <iterator>

#include <librmcs/data/datas.hpp>
#include <librmcs/spec/uart.hpp>

namespace librmcs::spec::rmcs_board_ultra {

namespace internal {
class Rs485Descriptors;
}

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class Rs485Descriptor : public spec::UartDescriptor {
    friend internal::Rs485Descriptors;
    constexpr Rs485Descriptor(data::DataId id, data::DataId config_id)
        : spec::UartDescriptor(id, config_id) {}

public:
    Rs485Descriptor(const Rs485Descriptor&) = delete;
    Rs485Descriptor& operator=(const Rs485Descriptor&) = delete;
    Rs485Descriptor(Rs485Descriptor&&) = delete;
    Rs485Descriptor& operator=(Rs485Descriptor&&) = delete;

    [[nodiscard]] constexpr bool operator==(const Rs485Descriptor& other) const noexcept {
        return data_id == other.data_id;
    }
};

namespace internal {
class Rs485Descriptors {
    static constexpr Rs485Descriptor kArray[]{
        Rs485Descriptor{data::DataId::kRs4850, data::DataId::kRs4850Config},
        Rs485Descriptor{data::DataId::kRs4851, data::DataId::kRs4851Config},
    };

public:
    static constexpr std::size_t size() noexcept { return std::size(kArray); }
    static constexpr const Rs485Descriptor& operator[](std::size_t index) noexcept {
        return kArray[index];
    }
    static constexpr const Rs485Descriptor* begin() noexcept { return std::begin(kArray); }
    static constexpr const Rs485Descriptor* end() noexcept { return std::end(kArray); }
    static constexpr const Rs485Descriptor* find(data::DataId id) noexcept {
        for (const auto& descriptor : kArray) {
            if (descriptor.data_id == id)
                return &descriptor;
        }
        return nullptr;
    }
    static constexpr const Rs485Descriptor& kRs4850 = kArray[0];
    static constexpr const Rs485Descriptor& kRs4851 = kArray[1];
};
} // namespace internal

inline constexpr internal::Rs485Descriptors kRs485Descriptors{};

} // namespace librmcs::spec::rmcs_board_ultra
