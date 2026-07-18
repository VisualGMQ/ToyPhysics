#pragma once

#include "spdlog/spdlog.h"
#include <cassert>
#include <memory>

namespace toy_physics {
class LogManager {
public:
    static LogManager& GetInst();

    auto& GetConsoleLogger() { return m_console_logger; }

private:
    std::shared_ptr<spdlog::logger> m_console_logger;

    LogManager();

    static LogManager manager;
};
}  // namespace toy_physics

#define LOGI(fmt, ...)                                                  \
    do {                                                                \
        SPDLOG_LOGGER_INFO(                                             \
            toy_physics::LogManager::GetInst().GetConsoleLogger(), fmt, \
            ##__VA_ARGS__);                                             \
    } while (0)

#define LOGE(fmt, ...)                                                  \
    do {                                                                \
        SPDLOG_LOGGER_ERROR(                                            \
            toy_physics::LogManager::GetInst().GetConsoleLogger(), fmt, \
            ##__VA_ARGS__);                                             \
    } while (0)

#define LOGW(fmt, ...)                                                  \
    do {                                                                \
        SPDLOG_LOGGER_WARN(                                             \
            toy_physics::LogManager::GetInst().GetConsoleLogger(), fmt, \
            ##__VA_ARGS__);                                             \
    } while (0)

#define LOGD(fmt, ...)                                                  \
    do {                                                                \
        SPDLOG_LOGGER_DEBUG(                                            \
            toy_physics::LogManager::GetInst().GetConsoleLogger(), fmt, \
            ##__VA_ARGS__);                                             \
    } while (0)

#define LOGC(fmt, ...)                                                  \
    do {                                                                \
        SPDLOG_LOGGER_CRITICAL(                                         \
            toy_physics::LogManager::GetInst().GetConsoleLogger(), fmt, \
            ##__VA_ARGS__);                                             \
    } while (0)

#define LOGT(fmt, ...)                                                  \
    do {                                                                \
        SPDLOG_LOGGER_TRACE(                                            \
            toy_physics::LogManager::GetInst().GetConsoleLogger(), fmt, \
            ##__VA_ARGS__);                                             \
    } while (0)