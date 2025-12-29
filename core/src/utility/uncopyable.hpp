#pragma once

namespace librmcs::core::utility {

class Uncopyble {
public:
    Uncopyble() = default;
    Uncopyble(const Uncopyble&) = delete;
    Uncopyble& operator=(const Uncopyble&) = delete;
    Uncopyble(Uncopyble&&) = default;
    Uncopyble& operator=(Uncopyble&&) = default;
};

} // namespace librmcs::core::utility
