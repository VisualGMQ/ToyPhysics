#pragma once

#include "toy_physics/common/storage.hpp"
#include "toy_physics/common/tight_pool.hpp"
#include "toy_physics/common/transform.hpp"
#include "toy_physics/geometry/geometry.hpp"

#include <span>
#include <vector>

namespace toy_physics {

class Shape;

class DynamicRigidBody;
class StaticRigidBody;
class Foundation;

class RigidActor : public NonCopyable {
public:
    friend class World;

    enum class Type {
        Dynamic,
        Static,
    };

    explicit RigidActor(Type type, Foundation&);
    virtual ~RigidActor();

    [[nodiscard]] DynamicRigidBody* AsDynamic();
    [[nodiscard]] StaticRigidBody* AsStatic();

    [[nodiscard]] const DynamicRigidBody* AsDynamic() const;
    [[nodiscard]] const StaticRigidBody* AsStatic() const;

    [[nodiscard]] Type GetType() const;

    virtual void Teleport(const Transform& transform);
    [[nodiscard]] const Transform& GetTransform() const;

    [[nodiscard]] std::span<Shape*> GetShapes();
    [[nodiscard]] std::span<Shape* const> GetShapes() const;

    virtual void AttachShape(Shape& shape);
    virtual void RemoveShape(size_t index);
    void RemoveShape(Shape& shape);

protected:
    Transform m_transform;
    std::vector<Shape*> m_shapes;
    Type m_type;
    TightPoolID m_tight_id{InvalidTightPoolID}; // for World use

    Foundation& m_foundation;
};

struct RigidBodyMassProperties {
    real m_inv_mass{0};

    Vector3 m_inv_inertia{Vector3::Zero()};
    Quaternion m_inertia_rotation{
        Quaternion::Identity()};  // principal inertia axes -> body local
    Vector3 m_center_of_mass{Vector3::Zero()};
};

class DynamicRigidBody : public StorageElem<DynamicRigidBody>,
                         public RigidActor {
public:
    friend class Shape;
    friend class RigidSolver;

    explicit DynamicRigidBody(Foundation&);

    [[nodiscard]] Vector3 GetVelocity() const;
    [[nodiscard]] Vector3 GetAngularVelocity() const;

    void SetVelocity(const Vector3& velocity);
    void SetAngularVelocity(const Vector3& angular_velocity);

    void AttachShape(Shape& shape) override;
    void RemoveShape(size_t index) override;

    [[nodiscard]] const RigidBodyMassProperties& GetMassProperties() const;

    /**
     * convenient function to do $I^{-1} v$,
     * where I is inertia in world space.
     */
    [[nodiscard]] Vector3 MultiplyInvInertiaWorld(Vector3 v) const;

    [[nodiscard]] Vector3 GetCenterOfMassPosition() const;
    [[nodiscard]] Vector3 GetVelocityAt(const Vector3& world_point) const;
    void ApplyImpulse(const Vector3& impulse, const Vector3& r);

    void ApplyForce(const Vector3& force);
    void ApplyForceAtPoint(const Vector3& force, const Vector3& world_point);
    void ApplyTorque(const Vector3& torque);
    [[nodiscard]] Vector3 GetForce() const;
    [[nodiscard]] Vector3 GetTorque() const;
    void ClearForces();

private:
    RigidBodyMassProperties m_mp;

    Vector3 m_velocity{Vector3::Zero()};
    Vector3 m_angular_velocity{Vector3::Zero()};

    Vector3 m_force{Vector3::Zero()};
    Vector3 m_torque{Vector3::Zero()};

    void ensureMassProperties();
};

RigidBodyMassProperties ComputeRigidBodyMassProperties(
    std::span<const Shape* const> shapes);

class StaticRigidBody : public StorageElem<StaticRigidBody>, public RigidActor {
public:
    explicit StaticRigidBody(Foundation&);
};

class DynamicRigidBodyFactory : public Storage<DynamicRigidBody> {};

class StaticRigidBodyFactory : public Storage<StaticRigidBody> {};

}  // namespace toy_physics
