#pragma once
#include "toy_physics/common/noncopyable.hpp"
#include "toy_physics/world.hpp"

#include <memory>

namespace toy_physics {

class WorldFactory;
class StaticRigidBodyFactory;
class DynamicRigidBodyFactory;

class Shape;
class BroadPhase;
class ConvexFactory;
class ShapeFactory;
class DynamicRigidBody;
class StaticRigidBody;

class SphereGeometry;
class CapsuleGeometry;
class CylinderGeometry;
class ConvexHullGeometry;

class Foundation : NonCopyable {
public:
    Foundation();
    ~Foundation();

    World* CreateWorld(const World::Settings&);
    void DestroyWorld(World&);

    [[nodiscard]] StaticRigidBody* CreateStatic();
    [[nodiscard]] DynamicRigidBody* CreateDynamic();
    [[nodiscard]] Shape* CreateShape(const BoxGeometry& box) const;
    [[nodiscard]] Shape* CreateShape(const SphereGeometry& sphere) const;
    [[nodiscard]] Shape* CreateShape(const CapsuleGeometry& capsule) const;
    [[nodiscard]] Shape* CreateShape(const CylinderGeometry& cylinder) const;
    [[nodiscard]] Shape* CreateShape(
        const ConvexHullGeometry& convex_hull) const;

    void DestroyRigidActor(RigidActor&);
    void DestroyShape(Shape&);

private:
    std::unique_ptr<ConvexFactory> m_convex_factory;
    std::unique_ptr<WorldFactory> m_world_factory;
    std::unique_ptr<ShapeFactory> m_shape_factory;
    std::unique_ptr<DynamicRigidBodyFactory> m_dynamic_body_factory;
    std::unique_ptr<StaticRigidBodyFactory> m_static_body_factory;
};

}