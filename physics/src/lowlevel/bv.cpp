#include "toy_physics/lowlevel/bv.hpp"

#include <cmath>

namespace toy_physics {

AABB BuildBV(const BoxGeometry& box, Quaternion rotation) {
    Vector3 half =
        rotation.toRotationMatrix().cwiseAbs() * box.m_half_extent;
    return AABB::FromCenter(Vector3::Zero(), half);
}

AABB BuildBV(const SphereGeometry& sphere) {
    Vector3 half{sphere.m_radius, sphere.m_radius, sphere.m_radius};
    return AABB::FromCenter(Vector3::Zero(), half);
}

AABB BuildBV(const CapsuleGeometry& capsule, Quaternion rotation) {
    Vector3 axis = (rotation * Vector3::UnitY()).normalized();
    Vector3 half = axis.cwiseAbs() * capsule.m_half_height;
    half += Vector3::Constant(capsule.m_radius);
    return AABB::FromCenter(Vector3::Zero(), half);
}

AABB BuildBV(const CylinderGeometry& cylinder, Quaternion rotation) {
    Vector3 axis = (rotation * Vector3::UnitY()).normalized();
    Vector3 half = axis.cwiseAbs() * cylinder.m_half_height;
    for (int i = 0; i < 3; ++i) {
        half[i] += cylinder.m_radius *
                   std::sqrt(std::max(real(0), 1 - axis[i] * axis[i]));
    }
    return AABB::FromCenter(Vector3::Zero(), half);
}

AABB BuildBV(const ConvexHullGeometry& convex, Quaternion rotation) {
    const ConvexData* data = convex.GetConvex();
    if (!data || data->m_vertices.empty()) {
        return AABB{};
    }
    Quaternion inv = rotation.conjugate();
    Vector3 d[3] = {inv * Vector3::UnitX(), inv * Vector3::UnitY(),
                    inv * Vector3::UnitZ()};
    Vector3 mn = Vector3::Constant(kREAL_MAX);
    Vector3 mx = Vector3::Constant(-kREAL_MAX);
    for (const Vector3& v : data->m_vertices) {
        for (int i = 0; i < 3; ++i) {
            real proj = v.dot(d[i]);
            mn[i] = std::min(mn[i], proj);
            mx[i] = std::max(mx[i], proj);
        }
    }
    return {mn, mx};
}

AABB BuildBV(const std::array<Vector3, 3>& vertices) {
    Vector3 mn = vertices[0].cwiseMin(vertices[1]).cwiseMin(vertices[2]);
    Vector3 mx = vertices[0].cwiseMax(vertices[1]).cwiseMax(vertices[2]);
    return {mn, mx};
}

AABB BuildBV(const Geometry& geometry, Quaternion rotation) {
    switch (geometry.GetType()) {
    case Geometry::Type::Box:
        return BuildBV(*geometry.AsBox(), rotation);
    case Geometry::Type::Sphere:
        return BuildBV(*geometry.AsSphere());
    case Geometry::Type::Capsule:
        return BuildBV(*geometry.AsCapsule(), rotation);
    case Geometry::Type::Cylinder:
        return BuildBV(*geometry.AsCylinder(), rotation);
    case Geometry::Type::ConvexHull:
        return BuildBV(*geometry.AsConvexHull(), rotation);
    case Geometry::Type::Invalid:
    default:
        return AABB{};
    }
}

}  // namespace toy_physics
