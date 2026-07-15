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
    [[nodiscard]] virtual Type GetType() const = 0;

    [[nodiscard]] class BoxGeometry* AsBox();
    [[nodiscard]] class SphereGeometry* AsSphere();
    [[nodiscard]] class CapsuleGeometry* AsCapsule();
};

class BoxGeometry : public Geometry {
public:
    explicit BoxGeometry(const Eigen::Vector3f& size);

    [[nodiscard]] Type GetType() const override;

    Eigen::Vector3f m_half_size;
};

class SphereGeometry : public Geometry {
public:
    explicit SphereGeometry(float radius);

    [[nodiscard]] Type GetType() const override;

    float m_radius;
};

class CapsuleGeometry : public Geometry {
public:
    explicit CapsuleGeometry(float radius, float height);

    [[nodiscard]] Type GetType() const override;

    float m_radius;
    float m_height;
};

using GeometryPtr = std::shared_ptr<Geometry>;

struct BoundingBox {
    Eigen::Vector3f m_min;
    Eigen::Vector3f m_max;

    BoundingBox();
    BoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max);

    static BoundingBox FromCenter(const Eigen::Vector3f& center,
                                  const Eigen::Vector3f& half_size);
    static BoundingBox FromMinMax(const Eigen::Vector3f& min,
                                  const Eigen::Vector3f& max);

    [[nodiscard]] bool IsValid() const;

    [[nodiscard]] bool IsIntersect(const BoundingBox&) const;
    [[nodiscard]] BoundingBox Intersect(const BoundingBox&) const;
};

}  // namespace toy_physics
