/**
 * @file config_parser.cpp
 * @brief Implementation of configuration file and environment variable parsing
 */

#include "config_parser.hpp"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <algorithm>
#include <cctype>

std::string ConfigParser::trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return "";
    }
    size_t last = str.find_last_not_of(" \t");
    return str.substr(first, (last - first + 1));
}

std::map<std::string, std::string> ConfigParser::parseKeyValueFile(const std::string& path) {
    std::map<std::string, std::string> result;
    std::ifstream file(path);

    if (!file.is_open()) {
        return result;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }

        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) {
            continue;
        }

        std::string key = trim(line.substr(0, eq_pos));
        std::string value = trim(line.substr(eq_pos + 1));

        if (!key.empty() && !value.empty()) {
            result[key] = value;
        }
    }

    return result;
}

UpsHatConfig ConfigParser::parseConfigFile(const std::string& config_path) {
    UpsHatConfig config = getDefaultConfig();
    auto kv_map = parseKeyValueFile(config_path);

    if (kv_map.find("I2C_BUS") != kv_map.end()) {
        config.i2c_bus = std::stoi(kv_map["I2C_BUS"]);
    }
    if (kv_map.find("I2C_ADDR") != kv_map.end()) {
        config.i2c_addr = static_cast<uint8_t>(std::stoi(kv_map["I2C_ADDR"]));
    }
    if (kv_map.find("PUBLISH_RATE_HZ") != kv_map.end()) {
        config.publish_rate_hz = std::stod(kv_map["PUBLISH_RATE_HZ"]);
    }
    if (kv_map.find("SHUTDOWN_DELAY_SEC") != kv_map.end()) {
        config.shutdown_delay_sec = std::stoi(kv_map["SHUTDOWN_DELAY_SEC"]);
    }
    if (kv_map.find("LOW_VOLTAGE_THRESHOLD") != kv_map.end()) {
        config.low_voltage_threshold_mv = static_cast<uint16_t>(std::stoi(kv_map["LOW_VOLTAGE_THRESHOLD"]));
    }
    if (kv_map.find("LOW_VOLTAGE_THRESHOLD_COUNT") != kv_map.end()) {
        config.low_voltage_threshold_count = std::stoi(kv_map["LOW_VOLTAGE_THRESHOLD_COUNT"]);
    }
    if (kv_map.find("ENABLE_SYSLOG") != kv_map.end()) {
        std::string val = kv_map["ENABLE_SYSLOG"];
        std::transform(val.begin(), val.end(), val.begin(), ::tolower);
        config.enable_syslog = (val == "true" || val == "1" || val == "yes");
    }

    return config;
}

UpsHatConfig ConfigParser::parseEnvironment() {
    UpsHatConfig config = getDefaultConfig();

    const char* env_val;

    if ((env_val = std::getenv("I2C_BUS")) != nullptr) {
        config.i2c_bus = std::stoi(env_val);
    }
    if ((env_val = std::getenv("I2C_ADDR")) != nullptr) {
        config.i2c_addr = static_cast<uint8_t>(std::stoi(env_val));
    }
    if ((env_val = std::getenv("PUBLISH_RATE_HZ")) != nullptr) {
        config.publish_rate_hz = std::stod(env_val);
    }
    if ((env_val = std::getenv("SHUTDOWN_DELAY_SEC")) != nullptr) {
        config.shutdown_delay_sec = std::stoi(env_val);
    }
    if ((env_val = std::getenv("LOW_VOLTAGE_THRESHOLD")) != nullptr) {
        config.low_voltage_threshold_mv = static_cast<uint16_t>(std::stoi(env_val));
    }
    if ((env_val = std::getenv("LOW_VOLTAGE_THRESHOLD_COUNT")) != nullptr) {
        config.low_voltage_threshold_count = std::stoi(env_val);
    }
    if ((env_val = std::getenv("ENABLE_SYSLOG")) != nullptr) {
        std::string val = env_val;
        std::transform(val.begin(), val.end(), val.begin(), ::tolower);
        config.enable_syslog = (val == "true" || val == "1" || val == "yes");
    }

    return config;
}

UpsHatConfig ConfigParser::getDefaultConfig() {
    UpsHatConfig config;
    return config;
}

