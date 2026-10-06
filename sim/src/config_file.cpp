#include "sim/config_file.hpp"

#include <array>
#include <fstream>
#include <stdexcept>

namespace sim {
ap_aircraft_config_t load_config(const std::filesystem::path& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("Cannot open configuration: " + path.string());
    const auto length = file.tellg();
    if (length != static_cast<std::streamoff>(AP_CONFIG_RECORD_SIZE) &&
        length != static_cast<std::streamoff>(AP_CONFIG_V1_RECORD_SIZE) &&
        length != static_cast<std::streamoff>(AP_CONFIG_V2_RECORD_SIZE))
        throw std::runtime_error("Configuration must be 128 bytes (v3), 96 bytes (v2), or 80 bytes (v1): " + path.string());
    std::array<uint8_t, AP_CONFIG_RECORD_SIZE> bytes{};
    file.seekg(0);
    file.read(reinterpret_cast<char*>(bytes.data()), length);
    ap_aircraft_config_t config{};
    if (!file || !ap_config_decode(bytes.data(), static_cast<std::size_t>(length), &config))
        throw std::runtime_error("Invalid configuration format, version, CRC, or parameter bounds: " + path.string());
    return config;
}

void save_config(const std::filesystem::path& path, const ap_aircraft_config_t& config)
{
    std::array<uint8_t, AP_CONFIG_RECORD_SIZE> bytes{};
    if (!ap_config_encode(&config, bytes.data(), bytes.size()))
        throw std::invalid_argument("Invalid configuration parameter bounds");
    if (std::filesystem::exists(path))
        throw std::runtime_error("Configuration output already exists; choose a new path: " + path.string());
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream file;
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.open(path, std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    file.close();
}
}
