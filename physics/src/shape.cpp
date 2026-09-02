#include "toy_physics/shape.hpp"

#include "toy_physics/lowlevel/bv.hpp"

namespace toy_physics {

Shape::Shape() {
    rebuildBV();
}

Shape::Shape(const BoxGeometry& box) : m_geometry{box} {
    rebuildBV();
}

Shape::Shape(const SphereGeometry& sphere) : m_geometry{sphere} {
    rebuildBV();
}

Shape::Shape(const CapsuleGeometry& capsule) : m_geometry{capsule} {
    rebuildBV();
}

Shape::Shape(const CylinderGeometry& cylinder) : m_geometry{cylinder} {
    rebuildBV();
}

Shape::Shape(const ConvexHullGeometry& convex_hull) : m_geometry{convex_hull} {
    rebuildBV();
}

Geometry::Type Shape::GetType() const {
    return m_geometry.GetGeometry()->GetType();
}

void Shape::SetTransform(const Transform& transform) {
    if (m_transform.m_rotation == transform.m_rotation) {
        Vector3 offset = transform.m_position - m_transform.m_position;
        m_aabb.m_min += offset;
        m_aabb.m_max += offset;
        m_transform = transform;
    } else {
        m_transform = transform;
        rebuildBV();
    }
}

void Shape::SetTransform(Vector3 position, Quaternion rotation) {
    SetTransform(Transform{position, rotation});
}

const Transform& Shape::GetTransform() const {
    return m_transform;
}

const AABB& Shape::GetBoundingBox() const {
    return m_aabb;
}

void Shape::rebuildBV() {
    m_aabb = BuildBV(*m_geometry.GetGeometry(), m_transform.m_rotation);

    m_aabb.m_min += m_transform.m_position;
    m_aabb.m_max += m_transform.m_position;
}

}  // namespace toy_physics
