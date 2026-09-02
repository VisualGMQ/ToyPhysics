#pragma once

#include "common/tight_pool.hpp"
#include "toy_physics/common/storage.hpp"
#include "toy_physics/geometry/bounding_box.hpp"
#include "toy_physics/geometry/union.hpp"
#include "toy_physics/transform.hpp"

namespace toy_physics {

class Shape : public StorageElem<Shape> {
public:
    friend class BroadPhase;

    Shape();
    explicit Shape(const BoxGeometry& box);
    explicit Shape(const SphereGeometry& sphere);
    explicit Shape(const CapsuleGeometry& capsule);
    explicit Shape(const CylinderGeometry& cylinder);
    explicit Shape(const ConvexHullGeometry& convex_hull);

    [[nodiscard]] Geometry::Type GetType() const;

    void SetTransform(const Transform& transform);
    void SetTransform(Vector3 position, Quaternion rotation);

    [[nodiscard]] const Transform& GetTransform() const;
    [[nodiscard]] const BoundingBox& GetBoundingBox() const;

private:
    void rebuildBV();

    GeometryUnion m_geometry;
    Transform m_transform;
    BoundingBox m_aabb;
    TightPoolID m_sq_index{
        InvalidTightPoolID};  // for scene query broad phase use
};

class ShapeFactory : public Storage<Shape> {};

}  // namespace toy_physics
