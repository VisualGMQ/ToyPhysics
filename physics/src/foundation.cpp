#include "toy_physics/foundation.hpp"
#include "toy_physics/geometry/capsule.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/geometry/cylinder.hpp"
#include "toy_physics/geometry/sphere.hpp"
#include "toy_physics/rigid_sim/rigidbody.hpp"
#include "toy_physics/rigid_sim/shape.hpp"
#include "toy_physics/world.hpp"

namespace toy_physics {

Foundation::Foundation() {
    m_world_factory = std::make_unique<WorldFactory>();
    m_convex_factory = std::make_unique<ConvexFactory>();
    m_shape_factory = std::make_unique<ShapeFactory>();
    m_dynamic_body_factory = std::make_unique<DynamicRigidBodyFactory>();
    m_static_body_factory = std::make_unique<StaticRigidBodyFactory>();
}

Foundation::~Foundation() {
    m_world_factory.reset();
    m_dynamic_body_factory.reset();
    m_static_body_factory.reset();
    m_shape_factory.reset();
    m_convex_factory.reset();
}

World* Foundation::CreateWorld(const World::Settings& settings) {
    return m_world_factory->Create(settings, *this);
}

void Foundation::DestroyWorld(World& world) {
    m_world_factory->Destroy(&world);
}

DynamicRigidBody* Foundation::CreateDynamic() {
    return m_dynamic_body_factory->Create(*this);
}

StaticRigidBody* Foundation::CreateStatic() {
    return m_static_body_factory->Create(*this);
}

Shape* Foundation::CreateShape(const BoxGeometry& box) const {
    return m_shape_factory->Create(box);
}

Shape* Foundation::CreateShape(const SphereGeometry& sphere) const {
    return m_shape_factory->Create(sphere);
}

Shape* Foundation::CreateShape(const CapsuleGeometry& capsule) const {
    return m_shape_factory->Create(capsule);
}

Shape* Foundation::CreateShape(const CylinderGeometry& cylinder) const {
    return m_shape_factory->Create(cylinder);
}

Shape* Foundation::CreateShape(
    const ConvexHullGeometry& convex_hull) const {
    return m_shape_factory->Create(convex_hull);
}

void Foundation::DestroyShape(Shape& shape) {
    m_shape_factory->Destroy(&shape);
}

void Foundation::DestroyRigidActor(RigidActor& actor) {
    auto type = actor.GetType();
    switch (type) {
        case RigidActor::Type::Dynamic:
            m_dynamic_body_factory->Destroy(actor.AsDynamic());
            break;
        case RigidActor::Type::Static:
            m_static_body_factory->Destroy(actor.AsStatic());
            break;
    }
}


}  // namespace toy_physics