#pragma once
#include "shape.hpp"

namespace toy_physics {

struct Body {
    Pose m_pose;
    Eigen::Vector3f m_velocity;
    float m_inv_mass = 0.0;
    
    Shape m_geometry;
};

}
