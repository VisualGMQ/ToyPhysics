#include "toy_physics/geometry/bounding_box.hpp"

#include "toy_physics/common/check.hpp"
#include "toy_physics/lowlevel/algorithm.hpp"

namespace toy_physics {

BoundingBox::BoundingBox() : m_min{Vector3::Zero()}, m_max{Vector3::Zero()} {}

BoundingBox::BoundingBox(Vector3 min, Vector3 max) : m_min{min}, m_max{max} {
    TOY_CHECK(IsValid());
}

BoundingBox BoundingBox::FromCenter(Vector3 center, Vector3 half_size) {
    TOY_CHECK(half_size.x() > 0 && half_size.y() > 0 && half_size.z() > 0);
    BoundingBox result;
    result.m_min = center - half_size;
    result.m_max = center + half_size;
    return result;
}

BoundingBox BoundingBox::FromMinMax(Vector3 min, Vector3 max) {
    return {min, max};
}

bool BoundingBox::IsValid() const {
    return m_min.x() < m_max.x() && m_min.y() < m_max.y() &&
           m_min.z() < m_max.z();
}

bool BoundingBox::IsIntersect(const BoundingBox& o) const {
    return IsAABBIntersect(m_min, m_max, o.m_min, o.m_max);
}

BoundingBox BoundingBox::Intersect(const BoundingBox& o) const {
    TOY_CHECK(IsValid());
    TOY_CHECK(o.IsValid());

    TOY_ENSURE_R(IsIntersect(o));

    return {m_min.cwiseMax(o.m_min), m_max.cwiseMin(o.m_max)};
}

BoundingBox BoundingBox::Merge(const BoundingBox& o) const {
    AABB c;
    c.m_min = m_min.cwiseMin(o.m_min);
    c.m_max = m_max.cwiseMax(o.m_max);
    return c;
}

real BoundingBox::Area() const {
    Vector3 d = m_max - m_min;
    return 2.0 * (d.x() * d.y() + d.y() * d.z() + d.z() * d.x());
}

}  // namespace toy_physics