#ifndef FREQUENCY_HPP
#define FREQUENCY_HPP

#include <chrono>
#include <ratio>

namespace freq {

using hz = std::chrono::duration<long, std::ratio<1>>;
using khz = std::chrono::duration<long, std::kilo>;
using mhz = std::chrono::duration<long, std::mega>;

};

#endif
