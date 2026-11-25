#include "toy_physics/rigid_sim/shape.hpp"

#include "toy_physics/rigid_sim/rigidbody.hpp"
#include "toy_physics/lowlevel/bv.hpp"

namespace toy_physics {

Shape::Shape(const BoxGeometry& box) : m_geometry{box} {
    rebuildBV();
}

Shape::Shape(const SphereGeometry& sphere) : m_geometry{sphere} {
    rebuildBV();
}

Shape::Shape(const CapsuleGeometry& capsule) : m_geometry{capsule} {
    rebuildBV();
}

Shape::Shape(const CylinderGeometry& cylinder) : m_geometry{cylinder} {
    rebuildBV();
}

Shape::Shape(const ConvexHullGeometry& convex_hull) : m_geometry{convex_hull} {
    rebuildBV();
}

Geometry::Type Shape::GetType() const {
    return m_geometry.GetGeometry()->GetType();
}

void Shape::SetLocalTransform(const Transform& transform) {
    m_local_transform = transform;
    if (m_owner) {
        updateByOwnerTransform();
        m_owner->RefreshBoundingBox();
    } else {
        m_transform = transform;
        rebuildBV();
    }
}

void Shape::SetLocalTransform(Vector3 position, Quaternion rotation) {
    SetLocalTransform(Transform{position, rotation});
}

const Transform& Shape::GetLocalTransform() const {
    return m_local_transform;
}

const Transform& Shape::GetTransform() const {
    return m_transform;
}

const AABB& Shape::GetBoundingBox() const {
    return m_aabb;
}

const AABB& Shape::GetFatBoundingBox() const {
    return m_fat_aabb;
}

void Shape::SetFatAABBMargin(real margin) {
    m_fat_aabb_margin = margin;

    m_fat_aabb = AABB{};
    rebuildBV();
}

real Shape::GetFatAABBMargin() const {
    return m_fat_aabb_margin;
}

bool Shape::consumeSceneQueryDirty() {
    const bool dirty = m_sq_bp_dirty;
    m_sq_bp_dirty = false;
    return dirty;
}

bool Shape::consumeSimulationDirty() {
    const bool dirty = m_sim_bp_dirty;
    m_sim_bp_dirty = false;
    return dirty;
}

void Shape::clearBroadPhaseDirty() {
    m_sq_bp_dirty = false;
    m_sim_bp_dirty = false;
}

void Shape::SetFriction(real friction) {
    m_friction = friction;
}

real Shape::GetFriction() const {
    return m_friction;
}

const Geometry* Shape::GetGeometry() const {
    return m_geometry.GetGeometry();
}

Geometry* Shape::GetGeometry() {
    return m_geometry.GetGeometry();
}

RigidActor* Shape::GetOwner() {
    return m_owner;
}

const RigidActor* Shape::GetOwner() const {
    return m_owner;
}

real Shape::GetDensity() const {
    return m_density;
}

void Shape::SetDensity(real density) {
    m_density = density;

    if (m_owner != nullptr && m_owner->GetType() == RigidActor::Type::Dynamic) {
        m_owner->AsDynamic()->ensureMassProperties();
    }
}

void Shape::rebuildBV() {
    m_aabb = BuildBV(*m_geometry.GetGeometry(), m_transform.m_rotation);

    m_aabb.m_min += m_transform.m_position;
    m_aabb.m_max += m_transform.m_position;

    if (m_aabb.IsInside(m_fat_aabb)) {
        return;
    }

    // regenerate fat AABB
    const Vector3 margin = Vector3::Constant(m_fat_aabb_margin);
    m_fat_aabb = AABB{m_aabb.m_min - margin, m_aabb.m_max + margin};

    m_sq_bp_dirty = true;
    m_sim_bp_dirty = true;
}

void Shape::attachToActor(RigidActor* actor) {
    m_owner = actor;

    updateByOwnerTransform();
}

void Shape::updateByOwnerTransform() {
    TOY_ENSURE(m_owner);

    m_transform = m_owner->GetTransform().TransformBy(m_local_transform);
    rebuildBV();
}

}  // namespace toy_physics
