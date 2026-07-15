#include "toy_physics/geometry.hpp"

#include "toy_physics/check.hpp"
#include "toy_physics/log.hpp"

namespace toy_physics {
BoxGeometry* Geometry::AsBox() {
    return GetType() == Type::Box ? static_cast<BoxGeometry*>(this) : nullptr;
}

SphereGeometry* Geometry::AsSphere() {
    return GetType() == Type::Sphere ? static_cast<SphereGeometry*>(this)
                                     : nullptr;
}

CapsuleGeometry* Geometry::AsCapsule() {
    return GetType() == Type::Capsule ? static_cast<CapsuleGeometry*>(this)
                                      : nullptr;
}

BoxGeometry::BoxGeometry(const Eigen::Vector3f& size) : m_half_size{size} {}

Geometry::Type BoxGeometry::GetType() const {
    return Type::Box;
}

SphereGeometry::SphereGeometry(float radius) : m_radius{radius} {}

Geometry::Type SphereGeometry::GetType() const {
    return Type::Sphere;
}

CapsuleGeometry::CapsuleGeometry(float radius, float height)
    : m_radius{radius}, m_height{height} {}

Geometry::Type CapsuleGeometry::GetType() const {
    return Type::Capsule;
}

BoundingBox::BoundingBox()
    : m_min{Eigen::Vector3f::Zero()}, m_max{Eigen::Vector3f::Zero()} {}

BoundingBox::BoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max)
    : m_min{min}, m_max{max} {
    TOY_CHECK(IsValid());
}

BoundingBox BoundingBox::FromCenter(const Eigen::Vector3f& center,
                                    const Eigen::Vector3f& half_size) {
    TOY_CHECK_R(half_size.x() > 0 && half_size.y() > 0 && half_size.z() > 0);
    BoundingBox result;
    result.m_min = center - half_size;
    result.m_max = center + half_size;
    return result;
}

BoundingBox BoundingBox::FromMinMax(const Eigen::Vector3f& min,
                                    const Eigen::Vector3f& max) {
    return {min, max};
}

bool BoundingBox::IsValid() const {
    return m_min.x() < m_max.x() && m_min.y() < m_max.y() &&
           m_min.z() < m_max.z();
}

bool BoundingBox::IsIntersect(const BoundingBox& o) const {
    TOY_ENSURE_RV(IsValid(), false);

    return !(m_min.x() >= o.m_max.x() || m_min.y() >= o.m_max.y() ||
             m_min.z() >= o.m_max.z() || m_max.x() <= o.m_min.x() ||
             m_max.y() <= o.m_min.y() || m_max.z() <= o.m_min.z());
}

BoundingBox BoundingBox::Intersect(const BoundingBox& o) const {
    TOY_CHECK_R(IsValid());
    TOY_CHECK_R(o.IsValid());

    TOY_ENSURE_R(IsIntersect(o));

    return {m_min.cwiseMax(o.m_min), m_max.cwiseMin(o.m_max)};
}

}  // namespace toy_physics
