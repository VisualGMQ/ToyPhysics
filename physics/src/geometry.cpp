#include "toy_physics/geometry.hpp"

namespace toy_physics {
BoxGeometry* Geometry::AsBox() {
    return GetType() == Type::Box ? static_cast<BoxGeometry*>(this) : nullptr;
}

SphereGeometry* Geometry::AsSphere() {
    return GetType() == Type::Sphere
               ? static_cast<SphereGeometry*>(this)
               : nullptr;
}

CapsuleGeometry* Geometry::AsCapsule() {
    return GetType() == Type::Capsule
               ? static_cast<CapsuleGeometry*>(this)
               : nullptr;
}

BoxGeometry::BoxGeometry(const Eigen::Vector3f& size)
    : m_half_size{size} {
}

SphereGeometry::SphereGeometry(float radius)
    : m_radius{radius} {
}

CapsuleGeometry::CapsuleGeometry(float radius, float height)
    : m_radius{radius}, m_height{height} {
}
}
