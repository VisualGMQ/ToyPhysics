#pragma once

#include "toy_physics/geometry/geometry.hpp"

namespace toy_physics {

class InvalidGeometry: public Geometry {
public:
    InvalidGeometry();
};

}