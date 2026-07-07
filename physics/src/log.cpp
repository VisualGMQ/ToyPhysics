#include "toy_physics/log.hpp"

#include "spdlog/sinks/android_sink.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/null_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <iostream>

LogManager LogManager::manager;

LogManager& LogManager::GetInst() {
    return manager;
}

LogManager::LogManager() {
    m_console_logger = spdlog::stdout_color_mt("console");
}