#pragma once

#include <optional>
#include <string>
#include <vector>

namespace closetbot {

std::vector<std::string> get_ports();
std::optional<std::string> get_port_of_robot();

}  // namespace closetbot
