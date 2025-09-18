#pragma once
#include "Eigen/Dense"

namespace toy_physics {

struct Pose {
    Eigen::Vector3f m_position{};
    Eigen::Quaternionf m_rotation{Eigen::Quaternionf::Identity()};

    Pose TransformBy(const Pose& o) const;
    Pose RelativeBy(const Pose& child) const;

    bool operator==(const Pose&) const noexcept;
    bool operator!=(const Pose&) const noexcept;
};

}
