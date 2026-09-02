#pragma once
#include "toy_physics/common/math.hpp"

namespace toy_physics {

struct Transform {
    Vector3 m_position{};
    Quaternion m_rotation{Quaternion::Identity()};

    [[nodiscard]] Transform TransformBy(const Transform& o) const;
    [[nodiscard]] Transform RelativeBy(const Transform& child) const;

    bool operator==(const Transform&) const noexcept;
    bool operator!=(const Transform&) const noexcept;
};

}  // namespace toy_physics
