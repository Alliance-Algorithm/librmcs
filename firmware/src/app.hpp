#pragma once

#include "core/src/utility/immovable.hpp"
#include "firmware/src/utility/lazy.hpp"

namespace librmcs::firmware {

class App : private core::utility::Immovable {
public:
    using Lazy = utility::Lazy<App>;

    App();

    [[noreturn]] void run();
};

inline constinit App::Lazy app;

} // namespace librmcs::firmware
