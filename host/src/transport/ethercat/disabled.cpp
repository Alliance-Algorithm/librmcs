#include <memory>
#include <stdexcept>

#include <librmcs/board/common.hpp>

#include "host/src/transport/ethercat/ethercat.hpp"
#include "host/src/transport/transport.hpp"

namespace librmcs::host::transport::ethercat {

std::unique_ptr<Transport>
    EthercatFactory::create(const board::EthercatOptions&, const board::AdvancedOptions&) {
    throw std::runtime_error{"EtherCAT requires a Linux SDK built with LIBRMCS_ENABLE_ETHERCAT=ON"};
}

} // namespace librmcs::host::transport::ethercat
