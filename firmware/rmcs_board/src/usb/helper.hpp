#pragma once

#include "core/src/protocol/serializer.hpp"

namespace librmcs::firmware::usb {

core::protocol::Serializer& get_serializer();

}
