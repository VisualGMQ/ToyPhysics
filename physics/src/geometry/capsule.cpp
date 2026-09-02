#include "toy_physics/geometry/capsule.hpp"

#include "toy_physics/common/check.hpp"

namespace toy_physics {

CapsuleGeometry::CapsuleGeometry()
    : Geometry{Type::Capsule} {}

CapsuleGeometry::CapsuleGeometry(real radius, real half_height)
    : Geometry{Type::Capsule}, m_radius{radius}, m_half_height{half_height} {
    TOY_CHECK(IsValid());
}

bool CapsuleGeometry::IsValid() const {
    return m_radius > 0 && m_half_height > 0;
}

}
