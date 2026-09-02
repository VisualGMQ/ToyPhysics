#include "toy_physics/geometry/sphere.hpp"

#include "toy_physics/common/check.hpp"

namespace toy_physics {

SphereGeometry::SphereGeometry()
    : Geometry{Type::Sphere}, m_radius{} {}

SphereGeometry::SphereGeometry(real radius)
    : Geometry{Type::Sphere}, m_radius{radius} {
    TOY_CHECK(IsValid());
}

bool SphereGeometry::IsValid() const {
    return m_radius > 0;
}

}
