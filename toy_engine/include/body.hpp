#pragma once
#include "nickel/common/math/math.hpp"

namespace toy_physics {

class Shape {
public:
    enum class Type {
        Sphere,
    };
    
    virtual ~Shape() = default;
    virtual Type GetType() const = 0;
    virtual nickel::Vec3 GetCenterOfMass() const;

private:
    nickel::Vec3 m_center_of_mass;
};

class SphereShape : public Shape {
public:
    float m_radius;

    explicit SphereShape(float radius);

    Type GetType() const override;
};

class Body {
public:
    nickel::Vec3 m_linear_vel;
    nickel::Vec3 m_position;
    nickel::Quat m_rotation;
    float m_inv_mass{};
    std::shared_ptr<Shape> m_shape;

    nickel::Vec3 GetCenterOfMassWorldSpace() const;
    nickel::Vec3 GetCenterOfMassLocalSpace() const;
    nickel::Vec3 World2BodySpace(const nickel::Vec3&) const;
    nickel::Vec3 Body2WorldSpace(const nickel::Vec3&) const;
    void AddImpulse(const nickel::Vec3&);
};

}