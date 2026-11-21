#pragma once
#include "toy_physics/common/math.hpp"

namespace toy_physics {

class Geometry;

struct GeometryMassProperties {
    real m_mass{0};
    Vector3 m_center_of_mass{Vector3::Zero()};
    Vector3 m_inertia{Vector3::Zero()};  // inertia in principal axes
    Matrix33 m_principle_axes = Matrix33::Identity(); // axes are in columns
};

[[nodiscard]] GeometryMassProperties ComputeMassProperties(
    const Geometry& geometry, real density);

}  // namespace toy_physics