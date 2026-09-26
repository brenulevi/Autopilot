#pragma once

#include "autopilot/config.h"
#include <filesystem>

namespace sim {
// Host filesystem adapters. Throw on invalid data or I/O errors.
ap_aircraft_config_t load_config(const std::filesystem::path& path);
// Creates a new file; refuses existing paths so previous records are preserved.
// Not an EEPROM transaction implementation or a concurrent-writer API.
void save_config(const std::filesystem::path& path, const ap_aircraft_config_t& config);
}
