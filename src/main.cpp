/**
 * @file main.cpp
 * @brief Main entry point for UPS HAT Controller application
 */

#include "ups_hat_monitor.hpp"
#include "config_parser.hpp"
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <unistd.h>

// Global monitor instance for signal handler access
static UpsHatMonitor* g_monitor = nullptr;

/**
 * @brief Signal handler for graceful shutdown
 * @param signal Signal number (SIGINT, SIGTERM)
 */
void signalHandler(int signal) {
    if (g_monitor) {
        std::cerr << "Received signal " << signal << ", shutting down..." << std::endl;
        g_monitor->stop();
    }
}

/**
 * @brief Main entry point
 *
 * Parses configuration from file, environment variables, or command line,
 * initializes the monitor, and runs the main monitoring loop.
 */
int main(int argc, char* argv[]) {
    // Parse configuration with priority: config file > environment > defaults
    UpsHatConfig config;

    // Try configuration file first
    const char* config_path = "/etc/ups-hat-controller/ups-hat-controller.conf";
    if (access(config_path, R_OK) == 0) {
        config = ConfigParser::parseConfigFile(config_path);
    } else {
        // Fall back to environment variables or defaults
        config = ConfigParser::parseEnvironment();
    }

    // Override with command line arguments if provided
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config = ConfigParser::parseConfigFile(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [--config <path>] [--help]\n";
            std::cout << "Configuration file: /etc/ups-hat-controller/ups-hat-controller.conf\n";
            std::cout << "Environment variables: I2C_BUS, I2C_ADDR, PUBLISH_RATE_HZ, etc.\n";
            return 0;
        }
    }

    // Create monitor
    UpsHatMonitor monitor(config);
    g_monitor = &monitor;

    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Initialize monitor
    if (!monitor.initialize()) {
        std::cerr << "Failed to initialize UPS HAT monitor" << std::endl;
        return 1;
    }

    // Run monitoring loop
    monitor.run();

    return 0;
}

