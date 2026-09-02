#include "toy_physics/geometry/box.hpp"

#include "toy_physics/common/check.hpp"

namespace toy_physics {

BoxGeometry::BoxGeometry()
    : Geometry{Type::Box} {}

BoxGeometry::BoxGeometry(Vector3 half_extent)
    : Geometry{Type::Box}, m_half_extent{half_extent} {
    TOY_CHECK(IsValid());
}

bool BoxGeometry::IsValid() const {
    return m_half_extent.x() > 0 && m_half_extent.y() > 0 &&
           m_half_extent.z() > 0;
}

}
