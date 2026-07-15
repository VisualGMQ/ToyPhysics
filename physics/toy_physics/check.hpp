#pragma once

#include "toy_physics/config.hpp"
#include "toy_physics/log.hpp"
#include <cassert>

#define ASSERT(expr, msg) assert(((void)msg, (expr)))

#define TOY_ENSURE(expr)              \
    if (!(expr)) {                    \
        LOGW("Check Failed: " #expr); \
        return;                       \
    }
#define TOY_ENSURE_R(expr)            \
    if (!(expr)) {                    \
        LOGW("Check Failed: " #expr); \
        return {};                    \
    }
#define TOY_ENSURE_RV(expr, return_value) \
    if (!(expr)) {                        \
        LOGW("Check Failed: " #expr);     \
        return (return_value);            \
    }

#ifdef TOY_PHYSICS_CHECK
#define TOY_CHECK(expr) TOY_ENSURE(expr)
#define TOY_CHECK_R(expr) TOY_ENSURE_R(expr)
#define TOY_CHECK_RV(expr, return_value) TOY_ENSURE_RV(expr)
#else
#define TOY_CHECK(expr)
#define TOY_CHECK_R(expr)
#define TOY_CHECK_RV(expr)
#endif