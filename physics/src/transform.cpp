#include "toy_physics/transform.hpp"

namespace toy_physics {

Transform Transform::TransformBy(const Transform& o) const {
    Transform p;
    p.m_position = m_position + m_rotation * o.m_position;
    p.m_rotation = m_rotation * o.m_rotation;
    return p;
}

Transform Transform::RelativeBy(const Transform& child) const {
    Transform p;
    p.m_position = m_position - child.m_position;
    p.m_rotation = m_rotation * child.m_rotation.inverse();
    return p;
}

bool Transform::operator==(const Transform& p) const noexcept {
    return m_position == p.m_position && m_rotation == p.m_rotation;
}

bool Transform::operator!=(const Transform& o) const noexcept {
    return !(*this == o);
}
}  // namespace toy_physics
