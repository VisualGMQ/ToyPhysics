#include "toy_physics/geometry/cylinder.hpp"

#include "toy_physics/common/check.hpp"

namespace toy_physics {

CylinderGeometry::CylinderGeometry()
    : Geometry{Type::Cylinder} {}

CylinderGeometry::CylinderGeometry(real radius, real half_height)
    : Geometry{Type::Cylinder}, m_radius{radius}, m_half_height{half_height} {
    TOY_CHECK(IsValid());
}

bool CylinderGeometry::IsValid() const {
    return m_radius > 0 && m_half_height > 0;
}

}
