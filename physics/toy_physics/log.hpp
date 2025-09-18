#pragma once

#include "spdlog/spdlog.h"
#include <memory>

class LogManager {
public:
    static LogManager& GetInst();

    auto& GetConsoleLogger() { return m_console_logger; }

private:
    std::shared_ptr<spdlog::logger> m_console_logger;

    LogManager();

    static LogManager manager;
};

#define LOGI(fmt, ...)                                                    \
    do {                                                                  \
        SPDLOG_LOGGER_INFO(LogManager::GetInst().GetConsoleLogger(), fmt, \
                           ##__VA_ARGS__);                                \
    } while (0)

#define LOGE(fmt, ...)                                                     \
    do {                                                                   \
        SPDLOG_LOGGER_ERROR(LogManager::GetInst().GetConsoleLogger(), fmt, \
                            ##__VA_ARGS__);                                \
    } while (0)

#define LOGW(fmt, ...)                                                    \
    do {                                                                  \
        SPDLOG_LOGGER_WARN(LogManager::GetInst().GetConsoleLogger(), fmt, \
                           ##__VA_ARGS__);                                \
    } while (0)

#define LOGD(fmt, ...)                                                     \
    do {                                                                   \
        SPDLOG_LOGGER_DEBUG(LogManager::GetInst().GetConsoleLogger(), fmt, \
                            ##__VA_ARGS__);                                \
    } while (0)

#define LOGC(fmt, ...)                                                        \
    do {                                                                      \
        SPDLOG_LOGGER_CRITICAL(LogManager::GetInst().GetConsoleLogger(), fmt, \
                               ##__VA_ARGS__);                                \
    } while (0)

#define LOGT(fmt, ...)                                                     \
    do {                                                                   \
        SPDLOG_LOGGER_TRACE(LogManager::GetInst().GetConsoleLogger(), fmt, \
                            ##__VA_ARGS__);                                \
    } while (0)
