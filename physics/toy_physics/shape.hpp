#pragma once
#include "toy_physics/geometry.hpp"
#include "toy_physics/pose.hpp"

namespace toy_physics {

class Shape {
public:
    Pose m_local_pose;
    GeometryPtr m_geom;
};

}