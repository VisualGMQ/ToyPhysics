#pragma once
#include "toy_physics/math.hpp"

namespace toy_physics {

struct Pose {
    Vector3 m_position{};
    Quaternion m_rotation{Eigen::Quaternionf::Identity()};

    [[nodiscard]] Pose TransformBy(const Pose& o) const;
    [[nodiscard]] Pose RelativeBy(const Pose& child) const;

    bool operator==(const Pose&) const noexcept;
    bool operator!=(const Pose&) const noexcept;
};

}
