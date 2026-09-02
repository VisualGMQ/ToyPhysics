#pragma once
#include "toy_physics/geometry/geometry.hpp"

namespace toy_physics {

class BoxGeometry: public Geometry {
public:
    BoxGeometry();
    explicit BoxGeometry(Vector3 half_extent);
    [[nodiscard]] bool IsValid() const;

    Vector3 m_half_extent{Vector3::Zero()};
};

}
