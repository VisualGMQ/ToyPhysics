#include "toy_physics/geometry/mass_property.hpp"

#include "toy_physics/geometry/box.hpp"
#include "toy_physics/geometry/capsule.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/geometry/cylinder.hpp"
#include "toy_physics/geometry/sphere.hpp"

namespace toy_physics {

GeometryMassProperties computeMassProperties(const BoxGeometry& box,
                                          real density) {
    const Vector3 h = box.m_half_extent;

    const real sx = h.x() * 2;
    const real sy = h.y() * 2;
    const real sz = h.z() * 2;

    const real mass = density * sx * sy * sz;

    Vector3 inertia{mass / 12 * (sy * sy + sz * sz),
                    mass / 12 * (sx * sx + sz * sz),
                    mass / 12 * (sx * sx + sy * sy)};

    return {mass, Vector3::Zero(), inertia};
}

GeometryMassProperties computeMassProperties(const SphereGeometry& sphere,
                                          real density) {
    const real r = sphere.m_radius;
    const real r2 = r * r;

    const real mass = density * (4 / real(3) * MATH_PI) * r2 * r;

    const real i = 2 / real(5) * mass * r2;

    Vector3 inertia{i, i, i};

    return {mass, Vector3::Zero(), inertia};
}

GeometryMassProperties computeMassProperties(const CylinderGeometry& cylinder,
                                          real density) {
    const real r = cylinder.m_radius;
    const real r2 = r * r;
    const real h = cylinder.m_half_height * 2;

    const real mass = density * MATH_PI * r2 * h;

    // axis is Y
    const real iy = 1 / real(2) * mass * r2;
    const real ixz = 1 / real(12) * mass * (3 * r2 + h * h);

    Vector3 inertia{
        ixz,
        iy,
        ixz,
    };

    return {mass, Vector3::Zero(), inertia};
}

GeometryMassProperties computeMassProperties(const CapsuleGeometry& capsule,
                                          real density) {
    const real r = capsule.m_radius;
    const real r2 = r * r;
    const real h = capsule.m_half_height * real(2);

    const real cylinder_mass = static_cast<real>(MATH_PI) * h * r2 * density;

    const real hemisphere_mass =
        (real(2) * static_cast<real>(MATH_PI) / real(3)) * r2 * r * density;

    const real height_sq = h * h;

    real inertia_y = r2 * cylinder_mass * real(0.5);

    real inertia_xz =
        inertia_y * real(0.5) + cylinder_mass * height_sq / real(12);

    const real temp = hemisphere_mass * real(4) * r2 / real(5);

    inertia_y += temp;
    inertia_xz += temp + hemisphere_mass * (real(0.5) * height_sq +
                                            real(3) / real(4) * h * r);

    const real mass = cylinder_mass + hemisphere_mass * real(2);

    Vector3 inertia{
        inertia_xz,
        inertia_y,
        inertia_xz,
    };

    return {mass, Vector3::Zero(), inertia};
}

GeometryMassProperties computeMassProperties(const ConvexHullGeometry& hull,
                                          real density) {
    const ConvexData* data = hull.GetConvex();
    if (!data) {
        return {};
    }

    const real mass = density * data->GetVolume();

    return {
        mass,
        data->GetCenterOfMass(),
        density * data->GetUnitInertia(),
        data->GetPrincipalAxes(),
    };
}

GeometryMassProperties ComputeMassProperties(const Geometry& geom,
                                             real density) {
    switch (geom.GetType()) {
        case Geometry::Type::Box:
            return computeMassProperties(*geom.AsBox(), density);

        case Geometry::Type::Sphere:
            return computeMassProperties(*geom.AsSphere(), density);

        case Geometry::Type::Capsule:
            return computeMassProperties(*geom.AsCapsule(), density);

        case Geometry::Type::Cylinder:
            return computeMassProperties(*geom.AsCylinder(), density);

        case Geometry::Type::ConvexHull:
            return computeMassProperties(*geom.AsConvexHull(), density);

        default:
            return {};
    }
}

}  // namespace toy_physics