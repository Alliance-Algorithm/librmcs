#pragma once

#include <cstddef>
#include <iterator>

#include <librmcs/data/datas.hpp>
#include <librmcs/spec/can.hpp>

namespace librmcs::spec::rmcs_board_ultra {

namespace internal {
class CanDescriptors;
}

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class CanDescriptor : public spec::CanDescriptor {
    friend internal::CanDescriptors;
    constexpr explicit CanDescriptor(data::DataId data_id)
        : spec::CanDescriptor(data_id) {}

public:
    CanDescriptor(const CanDescriptor&) = delete;
    CanDescriptor& operator=(const CanDescriptor&) = delete;
    CanDescriptor(CanDescriptor&&) = delete;
    CanDescriptor& operator=(CanDescriptor&&) = delete;

    [[nodiscard]] constexpr bool operator==(const CanDescriptor& other) const noexcept {
        return data_id == other.data_id;
    }
};

namespace internal {
class CanDescriptors {
    static constexpr CanDescriptor kArray[]{
        CanDescriptor{data::DataId::kCan0},
        CanDescriptor{data::DataId::kCan1},
        CanDescriptor{data::DataId::kCan2},
        CanDescriptor{data::DataId::kCan3},
    };

public:
    constexpr CanDescriptors() = default;

    static constexpr std::size_t size() noexcept { return std::size(kArray); }

    static constexpr const CanDescriptor& operator[](std::size_t index) noexcept {
        return kArray[index];
    }

    static constexpr const CanDescriptor* begin() noexcept { return std::begin(kArray); }

    static constexpr const CanDescriptor* end() noexcept { return std::end(kArray); }

    static constexpr const CanDescriptor* find(data::DataId data_id) noexcept {
        for (const auto& descriptor : kArray) {
            if (descriptor.data_id == data_id)
                return &descriptor;
        }
        return nullptr;
    }

    static constexpr const CanDescriptor& kCan0 = kArray[0];
    static constexpr const CanDescriptor& kCan1 = kArray[1];
    static constexpr const CanDescriptor& kCan2 = kArray[2];
    static constexpr const CanDescriptor& kCan3 = kArray[3];
};
} // namespace internal

inline constexpr internal::CanDescriptors kCanDescriptors{};

} // namespace librmcs::spec::rmcs_board_ultra
