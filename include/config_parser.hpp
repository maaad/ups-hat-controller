#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

/**
 * @file config_parser.hpp
 * @brief Configuration parsing for UPS HAT Controller
 */

#include <string>
#include <map>
#include <cstdint>

// Configuration structure with default values
struct UpsHatConfig {
    int i2c_bus = 1;
    uint8_t i2c_addr = 0x2d;
    double publish_rate_hz = 1.0;
    int shutdown_delay_sec = 60;
    uint16_t low_voltage_threshold_mv = 3150;
    int low_voltage_threshold_count = 30;
    bool enable_syslog = true;
};

class ConfigParser {
public:
    static UpsHatConfig parseConfigFile(const std::string& config_path);
    static UpsHatConfig parseEnvironment();
    static UpsHatConfig getDefaultConfig();

private:
    static std::string trim(const std::string& str);
    static std::map<std::string, std::string> parseKeyValueFile(const std::string& path);
};

#endif // CONFIG_PARSER_HPP

