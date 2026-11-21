#pragma once

#include "toy_physics/common/tight_pool.hpp"
#include "toy_physics/common/transform.hpp"
#include "toy_physics/common/storage.hpp"
#include "toy_physics/geometry/bounding_box.hpp"
#include "toy_physics/geometry/union.hpp"

namespace toy_physics {

class RigidActor;

class Shape : public StorageElem<Shape>, public NonCopyable {
public:
    friend class BroadPhase;
    friend class RigidActor;

    explicit Shape(const BoxGeometry& box);
    explicit Shape(const SphereGeometry& sphere);
    explicit Shape(const CapsuleGeometry& capsule);
    explicit Shape(const CylinderGeometry& cylinder);
    explicit Shape(const ConvexHullGeometry& convex_hull);

    [[nodiscard]] Geometry::Type GetType() const;

    void SetLocalTransform(const Transform& transform);
    void SetLocalTransform(Vector3 position, Quaternion rotation);

    [[nodiscard]] const Transform& GetLocalTransform() const;
    [[nodiscard]] const Transform& GetTransform() const;
    [[nodiscard]] const BoundingBox& GetBoundingBox() const;

    void SetFriction(real);
    [[nodiscard]] real GetFriction() const;

    [[nodiscard]] const Geometry* GetGeometry() const;

    RigidActor* GetOwner();
    [[nodiscard]] const RigidActor* GetOwner() const;

    [[nodiscard]] real GetDensity() const;
    void SetDensity(real);

private:
    void rebuildBV();
    void attachToActor(RigidActor* actor);
    void updateByOwnerTransform();

    GeometryUnion m_geometry;
    Transform m_local_transform;
    Transform m_transform;
    BoundingBox m_aabb;
    real m_friction{0.2};
    real m_density{1};

    Vector3 m_inv_inertia;

    TightPoolID m_sq_index{
        InvalidTightPoolID};  // for scene query broad phase use
    RigidActor* m_owner{};
};

class ShapeFactory : public Storage<Shape> {};

}  // namespace toy_physics
