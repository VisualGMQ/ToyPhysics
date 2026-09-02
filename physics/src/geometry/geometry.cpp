#include "toy_physics/geometry/geometry.hpp"

#include "toy_physics/geometry/box.hpp"
#include "toy_physics/geometry/capsule.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/geometry/cylinder.hpp"
#include "toy_physics/geometry/sphere.hpp"

namespace toy_physics {

Geometry::Geometry(Type type) : m_type{type} {}

Geometry::Type Geometry::GetType() const {
    return m_type;
}

BoxGeometry* Geometry::AsBox() {
    return const_cast<BoxGeometry*>(std::as_const(*this).AsBox());
}

SphereGeometry* Geometry::AsSphere() {
    return const_cast<SphereGeometry*>(std::as_const(*this).AsSphere());
}

CapsuleGeometry* Geometry::AsCapsule() {
    return const_cast<CapsuleGeometry*>(std::as_const(*this).AsCapsule());
}

CylinderGeometry* Geometry::AsCylinder() {
    return const_cast<CylinderGeometry*>(std::as_const(*this).AsCylinder());
}

ConvexHullGeometry* Geometry::AsConvexHull() {
    return const_cast<ConvexHullGeometry*>(std::as_const(*this).AsConvexHull());
}

const BoxGeometry* Geometry::AsBox() const {
    return m_type == Type::Box ? static_cast<const BoxGeometry*>(this)
                               : nullptr;
}

const CapsuleGeometry* Geometry::AsCapsule() const {
    return m_type == Type::Capsule ? static_cast<const CapsuleGeometry*>(this)
                                   : nullptr;
}

const CylinderGeometry* Geometry::AsCylinder() const {
    return m_type == Type::Cylinder ? static_cast<const CylinderGeometry*>(this)
                                    : nullptr;
}

const ConvexHullGeometry* Geometry::AsConvexHull() const {
    return m_type == Type::ConvexHull
               ? static_cast<const ConvexHullGeometry*>(this)
               : nullptr;
}

const SphereGeometry* Geometry::AsSphere() const {
    return m_type == Type::Sphere ? static_cast<const SphereGeometry*>(this)
                                  : nullptr;
}

}  // namespace toy_physics
