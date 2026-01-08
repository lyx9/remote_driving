#include "fsm/utils.hpp"
#include <random>
#include <sstream>
#include <iomanip>

namespace fsm {
namespace utils {

std::string StringUtils::generateUUID() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 15);
    static const char* hex = "0123456789abcdef";

    std::stringstream ss;
    for (int i = 0; i < 8; ++i) ss << hex[dis(gen)];
    ss << "-";
    for (int i = 0; i < 4; ++i) ss << hex[dis(gen)];
    ss << "-4";  // Version 4 UUID
    for (int i = 0; i < 3; ++i) ss << hex[dis(gen)];
    ss << "-";
    ss << hex[(dis(gen) & 0x3) | 0x8];  // Variant
    for (int i = 0; i < 3; ++i) ss << hex[dis(gen)];
    ss << "-";
    for (int i = 0; i < 12; ++i) ss << hex[dis(gen)];

    return ss.str();
}

}  // namespace utils
}  // namespace fsm
