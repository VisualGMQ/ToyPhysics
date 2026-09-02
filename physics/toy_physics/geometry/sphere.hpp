#pragma once
#include "toy_physics/geometry/geometry.hpp"

namespace toy_physics {

class SphereGeometry: public Geometry {
public:
    SphereGeometry();
    explicit SphereGeometry(real radius);

    [[nodiscard]] bool IsValid() const;

    real m_radius{};
};

}
