#pragma once

#include "toy_physics/common/math.hpp"

namespace toy_physics {

class BoxGeometry;
class SphereGeometry;
class CylinderGeometry;
class CapsuleGeometry;
class ConvexHullGeometry;

/**
 * Geometry class.
 *
 * For memory efficient, this class don't have virtual destructor.
 *
 * @warning Never destruct object by Geometry*
 */
class Geometry {
public:
    enum class Type {
        Invalid,
        Box,
        Sphere,
        Capsule,
        Cylinder,
        ConvexHull,
    };

    explicit Geometry(Type type);

    [[nodiscard]] Type GetType() const;

    [[nodiscard]] BoxGeometry* AsBox();
    [[nodiscard]] SphereGeometry* AsSphere();
    [[nodiscard]] CapsuleGeometry* AsCapsule();
    [[nodiscard]] CylinderGeometry* AsCylinder();
    [[nodiscard]] ConvexHullGeometry* AsConvexHull();

    [[nodiscard]] const BoxGeometry* AsBox() const;
    [[nodiscard]] const SphereGeometry* AsSphere() const;
    [[nodiscard]] const CapsuleGeometry* AsCapsule() const;
    [[nodiscard]] const CylinderGeometry* AsCylinder() const;
    [[nodiscard]] const ConvexHullGeometry* AsConvexHull() const;

private:
    Type m_type;
};

template <Geometry::Type>
struct GeometryTraits {
    constexpr static bool MustBeStatic = false;
};

}  // namespace toy_physics
