#pragma once

#include "Eigen/Dense"

struct Vertex {
    Eigen::Vector3f m_position;
    Eigen::Vector2f m_uv;
    Eigen::Vector3f m_normal;
};