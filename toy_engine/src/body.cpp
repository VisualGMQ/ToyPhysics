#include "body.hpp"

namespace toy_physics {

nickel::Vec3 Shape::GetCenterOfMass() const {
    return m_center_of_mass;
}

SphereShape::SphereShape(float radius) : m_radius{radius} {}

Shape::Type SphereShape::GetType() const {
    return Type::Sphere;
}

nickel::Vec3 Body::GetCenterOfMassWorldSpace() const {
    return m_position + m_rotation * GetCenterOfMassLocalSpace();
}

nickel::Vec3 Body::GetCenterOfMassLocalSpace() const {
    return m_shape->GetCenterOfMass();
}

nickel::Vec3 Body::World2BodySpace(const nickel::Vec3& p) const {
    return m_rotation.Inverse() * (p - GetCenterOfMassWorldSpace());
}

nickel::Vec3 Body::Body2WorldSpace(const nickel::Vec3& p) const {
    return GetCenterOfMassWorldSpace() + m_rotation * p;
}

void Body::AddImpulse(const nickel::Vec3& p) {
    m_linear_vel += p * m_inv_mass;
}

}  // namespace toy_physics
