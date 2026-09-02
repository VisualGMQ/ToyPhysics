#include "toy_physics/geometry/union.hpp"

#include <memory>
#include <utility>

namespace toy_physics {

GeometryUnion::GeometryUnion(Geometry::Type type) {
    switch (type) {
    case Geometry::Type::Invalid:
        std::construct_at(&m_union.m_invalid);
        break;
    case Geometry::Type::Box:
        std::construct_at(&m_union.m_box);
        break;
    case Geometry::Type::Sphere:
        std::construct_at(&m_union.m_sphere);
        break;
    case Geometry::Type::Capsule:
        std::construct_at(&m_union.m_capsule);
        break;
    case Geometry::Type::Cylinder:
        std::construct_at(&m_union.m_cylinder);
        break;
    case Geometry::Type::ConvexHull:
        std::construct_at(&m_union.m_convex_hull);
        break;
    }
}

GeometryUnion::GeometryUnion(const BoxGeometry& box) {
    std::construct_at(&m_union.m_box, box);
}

GeometryUnion::GeometryUnion(const SphereGeometry& sphere) {
    std::construct_at(&m_union.m_sphere, sphere);
}

GeometryUnion::GeometryUnion(const CapsuleGeometry& capsule) {
    std::construct_at(&m_union.m_capsule, capsule);
}

GeometryUnion::GeometryUnion(const CylinderGeometry& cylinder) {
    std::construct_at(&m_union.m_cylinder, cylinder);
}

GeometryUnion::GeometryUnion(const ConvexHullGeometry& convex_hull) {
    std::construct_at(&m_union.m_convex_hull, convex_hull);
}

bool GeometryUnion::IsValid() const {
    return reinterpret_cast<const Geometry*>(&m_union)->GetType() ==
           Geometry::Type::Invalid;
}

const BoxGeometry* GeometryUnion::AsBox() const {
    return GetGeometry()->AsBox();
}

BoxGeometry* GeometryUnion::AsBox() {
    return const_cast<BoxGeometry*>(std::as_const(*this).AsBox());
}

const SphereGeometry* GeometryUnion::AsSphere() const {
    return GetGeometry()->AsSphere();
}

SphereGeometry* GeometryUnion::AsSphere() {
    return const_cast<SphereGeometry*>(std::as_const(*this).AsSphere());
}

const CapsuleGeometry* GeometryUnion::AsCapsule() const {
    return GetGeometry()->AsCapsule();
}

CapsuleGeometry* GeometryUnion::AsCapsule() {
    return const_cast<CapsuleGeometry*>(std::as_const(*this).AsCapsule());
}

const CylinderGeometry* GeometryUnion::AsCylinder() const {
    return GetGeometry()->AsCylinder();
}

CylinderGeometry* GeometryUnion::AsCylinder() {
    return const_cast<CylinderGeometry*>(std::as_const(*this).AsCylinder());
}

const ConvexHullGeometry* GeometryUnion::AsConvexHull() const {
    return GetGeometry()->AsConvexHull();
}

ConvexHullGeometry* GeometryUnion::AsConvexHull() {
    return const_cast<ConvexHullGeometry*>(
        std::as_const(*this).AsConvexHull());
}

const Geometry* GeometryUnion::GetGeometry() const {
    return reinterpret_cast<const Geometry*>(&m_union);
}

Geometry* GeometryUnion::GetGeometry() {
    return const_cast<Geometry*>(std::as_const(*this).GetGeometry());
}

}  // namespace toy_physics
