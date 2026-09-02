#include "toy_physics/context.hpp"
#include "toy_physics/bp/bvh_topdown.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/shape.hpp"

namespace toy_physics {

Context::Context() {
    m_convex_factory = std::make_unique<ConvexFactory>();
    m_shape_factory = std::make_unique<ShapeFactory>();
    m_sq_broad_phase =
        std::make_unique<AABBBroadPhase<BVHBuildPolicy::TopDown>>();
}

Context::~Context() {
    m_sq_broad_phase.reset();
    m_shape_factory.reset();
    m_convex_factory.reset();
}

Shape* Context::CreateShape(const BoxGeometry& box) const {
    return m_shape_factory->Create(box);
}

void Context::AddShape(Shape& shape) const {
    m_sq_broad_phase->AddObjects(std::span<Shape>{&shape, 1});
}

const BroadPhase& Context::GetSceneQueryBroadPhase() const {
    return *m_sq_broad_phase;
}

void Context::ForceRebuildSceneQueryBroadPhase() {
    m_sq_broad_phase->ApplyModify();
}

}  // namespace toy_physics
