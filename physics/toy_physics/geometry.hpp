#pragma once

#include "Eigen/Dense"
#include <memory>

namespace toy_physics {
class Geometry {
public:
    enum class Type {
        Box,
        Sphere,
        Capsule,
    };

    virtual ~Geometry() = default;
    virtual Type GetType() const = 0;

    class BoxGeometry* AsBox();
    class SphereGeometry* AsSphere();
    class CapsuleGeometry* AsCapsule();
};

class BoxGeometry : public Geometry {
public:
    explicit BoxGeometry(const Eigen::Vector3f& size);
    Type GetType() const override { return Type::Box; }

    Eigen::Vector3f m_half_size;
};

class SphereGeometry : public Geometry {
public:
    explicit SphereGeometry(float radius);
    Type GetType() const override { return Type::Sphere; }

    float m_radius;
};

class CapsuleGeometry : public Geometry {
public:
    explicit CapsuleGeometry(float radius, float height);
    Type GetType() const override { return Type::Capsule; }

    float m_radius;
    float m_height;
};

using GeometryPtr = std::shared_ptr<Geometry>;

}
