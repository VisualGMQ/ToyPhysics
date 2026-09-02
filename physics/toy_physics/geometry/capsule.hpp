#pragma once
#include "toy_physics/geometry/geometry.hpp"

namespace toy_physics {

class CapsuleGeometry: public Geometry {
public:
    CapsuleGeometry();
    explicit CapsuleGeometry(real radius, real half_height);

    [[nodiscard]] bool IsValid() const;

    real m_radius{};
    real m_half_height{};
};

}
