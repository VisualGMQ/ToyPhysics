#pragma once
#include <cstdint>
#include <limits>

namespace toy_physics {

#ifndef TOY_PHYSICS_CHECK
#ifdef TOY_PHYSICS_DEBUG
#define TOY_PHYSICS_CHECK 1
#else
#define TOY_PHYSICS_CHECK 0
#endif
#endif

#ifndef TOY_PHYSICS_FLOAT_TOLERANCE
static constexpr float kDefaultFloatTolerance = 1e-5;
#else
static constexpr float kDefaultFloatTolerance = TOY_PHYSICS_FLOAT_TOLERANCE;
#endif

#ifndef TOY_PHYSICS_DOUBLE_TOLERANCE
static constexpr float kDefaultDoubleTolerance = 1e-6;
#else
static constexpr float kDefaultDoubleTolerance = TOY_PHYSICS_DOUBLE_TOLERANCE;
#endif

#ifdef TOY_PHYSICS_DOUBLE
using real = double;
static constexpr real kDefaultRealTolerance = kDefaultDoubleTolerance;
#else
using real = float;
static constexpr real kDefaultRealTolerance = kDefaultFloatTolerance;
#endif

static constexpr real kREAL_MAX = std::numeric_limits<real>::max();
static constexpr real kREAL_EPSILON = std::numeric_limits<real>::epsilon();

#ifdef TOY_PHYSICS_EPA_MAX_FACE_COUNT
static constexpr uint32_t kEPA_MAX_FACE_COUNT = TOY_PHYSICS_EPA_MAX_FACE_COUNT;
#else
static constexpr uint32_t kEPA_MAX_FACE_COUNT = 128;
#endif

#ifdef TOY_PHYSICS_EPA_MAX_POINT_COUNT
static constexpr uint32_t kEPA_MAX_POINT_COUNT = TOY_PHYSICS_EPA_MAX_FACE_COUNT;
#else
static constexpr uint32_t kEPA_MAX_POINT_COUNT = 128;
#endif

#ifdef TOY_PHYSICS_EPA_MAX_TRIANGLE_COUNT
static constexpr uint32_t kEPA_MAX_TRIANGLE_COUNT = TOY_PHYSICS_EPA_MAX_FACE_COUNT;
#else
static constexpr uint32_t kEPA_MAX_TRIANGLE_COUNT = 128;
#endif

#ifdef TOY_PHYSICS_CONVEX_MAX_VERTEX_COUNT
static constexpr uint32_t CONVEX_MAX_VERTEX_COUNT =
    TOY_PHYSICS_CONVEX_MAX_VERTEX_COUNT;
#else
static constexpr uint32_t CONVEX_MAX_VERTEX_COUNT = 256;
#endif


static constexpr uint32_t BVH_OBJ_MAX_NUM = 0xF;

}  // namespace toy_physics
