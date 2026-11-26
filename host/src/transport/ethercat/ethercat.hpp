#pragma once

#include <memory>

#include <librmcs/board/common.hpp>

#include "host/src/transport/transport.hpp"

namespace librmcs::host::transport::ethercat {

class EthercatFactory {
public:
    static std::unique_ptr<Transport>
        create(const board::EthercatOptions& ethercat, const board::AdvancedOptions& options);
};

} // namespace librmcs::host::transport::ethercat
