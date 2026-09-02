#pragma once
#include "toy_physics/geometry/geometry.hpp"

namespace toy_physics {

class CylinderGeometry: public Geometry {
public:
    CylinderGeometry();
    explicit CylinderGeometry(real radius, real half_height);

    [[nodiscard]] bool IsValid() const;

    real m_radius{};
    real m_half_height{};
};

}
