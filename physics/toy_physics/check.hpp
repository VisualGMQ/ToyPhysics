#pragma once

#include "toy_physics/config.hpp"
#include "toy_physics/log.hpp"
#include <cassert>

#define ASSERT(expr, msg) assert(((void)msg, (expr)))

#define TOY_ENSURE(expr)               \
    if (!(expr)) {                     \
        LOGW("Ensure Failed: " #expr); \
        return;                        \
    }
#define TOY_ENSURE_R(expr)             \
    if (!(expr)) {                     \
        LOGW("Ensure Failed: " #expr); \
        return {};                     \
    }
#define TOY_ENSURE_RV(expr, return_value) \
    if (!(expr)) {                        \
        LOGW("Ensure Failed: " #expr);    \
        return (return_value);            \
    }

#define TOY_ENSURE_R_FALSE(expr)       \
    if (!(expr)) {                     \
        LOGW("Ensure Failed: " #expr); \
        return false;                  \
    }

#ifdef TOY_PHYSICS_CHECK
#define TOY_CHECK(expr)                \
    if (!(expr)) {                     \
        LOGE("Check Failed: " #expr); \
    }
#else
#define TOY_CHECK(expr)
#endif