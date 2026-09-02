#pragma once

namespace toy_physics {

class Shape;

enum class BVHBuildPolicy {
    TopDown,
    Incremental,
};

enum class BVHSplitPolicy {
    Median,
    SAH,
};

template <BVHBuildPolicy>
class AABBBroadPhase {};

template <BVHBuildPolicy>
class PrunerPool {};

}  // namespace toy_physics