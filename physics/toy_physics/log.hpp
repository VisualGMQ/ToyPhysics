#pragma once

#include <format>
#include <iostream>

#define LOGI(...)                                                        \
    do {                                                                 \
        std::cout << "[INFO] " << std::format(__VA_ARGS__) << std::endl; \
    } while (0)

#define LOGE(...)                                                         \
    do {                                                                  \
        std::cerr << "[ERROR] " << std::format(__VA_ARGS__) << std::endl; \
    } while (0)

#define LOGW(...)                                                        \
    do {                                                                 \
        std::cout << "[WARN] " << std::format(__VA_ARGS__) << std::endl; \
    } while (0)

#ifdef TOY_PHYSICS_DEBUG
#define LOGD(...)                                                         \
    do {                                                                  \
        std::cout << "[DEBUG] " << std::format(__VA_ARGS__) << std::endl; \
    } while (0)
#else
#define LOGD(...) \
    do {          \
    } while (0)
#endif

#define LOGC(...)                                                            \
    do {                                                                     \
        std::cerr << "[CRITICAL] " << std::format(__VA_ARGS__) << std::endl; \
    } while (0)

#ifdef TOY_PHYSICS_DEBUG
#define LOGT(...)                                                         \
    do {                                                                  \
        std::cout << "[TRACE] " << std::format(__VA_ARGS__) << std::endl; \
    } while (0)
#else
#define LOGT(...) \
    do {          \
    } while (0)
#endif
