#pragma once
#include "toy_physics/geometry/box.hpp"
#include "toy_physics/geometry/capsule.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/geometry/cylinder.hpp"
#include "toy_physics/geometry/invalid_geom.hpp"
#include "toy_physics/geometry/sphere.hpp"
#include <type_traits>

namespace toy_physics {

class GeometryUnion {
public:
    GeometryUnion() = default;
    explicit GeometryUnion(Geometry::Type type);

    explicit GeometryUnion(const BoxGeometry& box);
    explicit GeometryUnion(const SphereGeometry& sphere);
    explicit GeometryUnion(const CapsuleGeometry& capsule);
    explicit GeometryUnion(const CylinderGeometry& cylinder);
    explicit GeometryUnion(const ConvexHullGeometry& convex_hull);

    [[nodiscard]] bool IsValid() const;

    [[nodiscard]] const BoxGeometry* AsBox() const;
    [[nodiscard]] BoxGeometry* AsBox();

    [[nodiscard]] const SphereGeometry* AsSphere() const;
    [[nodiscard]] SphereGeometry* AsSphere();

    [[nodiscard]] const CapsuleGeometry* AsCapsule() const;
    [[nodiscard]] CapsuleGeometry* AsCapsule();

    [[nodiscard]] const CylinderGeometry* AsCylinder() const;
    [[nodiscard]] CylinderGeometry* AsCylinder();

    [[nodiscard]] const ConvexHullGeometry* AsConvexHull() const;
    [[nodiscard]] ConvexHullGeometry* AsConvexHull();

    [[nodiscard]] const Geometry* GetGeometry() const;
    [[nodiscard]] Geometry* GetGeometry();

private:
    template <typename T>
    static constexpr bool IsGeomSatisfied = std::is_trivially_destructible_v<T>;

    static_assert(IsGeomSatisfied<InvalidGeometry>);
    static_assert(IsGeomSatisfied<BoxGeometry>);
    static_assert(IsGeomSatisfied<SphereGeometry>);
    static_assert(IsGeomSatisfied<CapsuleGeometry>);
    static_assert(IsGeomSatisfied<CylinderGeometry>);
    static_assert(IsGeomSatisfied<ConvexHullGeometry>);

    union {
        InvalidGeometry m_invalid{};
        BoxGeometry m_box;
        SphereGeometry m_sphere;
        CylinderGeometry m_cylinder;
        CapsuleGeometry m_capsule;
        ConvexHullGeometry m_convex_hull;
    } m_union;
};

}  // namespace toy_physics
