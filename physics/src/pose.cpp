#include "toy_physics/pose.hpp"

namespace toy_physics {

Pose Pose::TransformBy(const Pose& o) const {
    Pose p;
    p.m_position = m_position +  m_rotation * o.m_position;
    p.m_rotation = m_rotation * o.m_rotation;
    return p;
}

Pose Pose::RelativeBy(const Pose& child) const {
    Pose p;
    p.m_position = m_position - child.m_position;
    p.m_rotation = m_rotation * child.m_rotation.inverse();
    return p;
}

bool Pose::operator==(const Pose& p) const noexcept {
    return m_position == p.m_position && m_rotation == p.m_rotation; 
}

bool Pose::operator!=(const Pose& o) const noexcept {
    return !(*this == o);
}
}
