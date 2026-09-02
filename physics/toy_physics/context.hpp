#pragma once

#include "toy_physics/common/noncopyable.hpp"
#include "toy_physics/geometry/box.hpp"
#include <memory>

namespace toy_physics {

class Shape;
class BroadPhase;
class ConvexFactory;
class ShapeFactory;

class Context : NonCopyable {
public:
    Context();
    ~Context();

    [[nodiscard]] Shape* CreateShape(const BoxGeometry&) const;

    void AddShape(Shape& shape) const;

    const BroadPhase& GetSceneQueryBroadPhase() const;
    void ForceRebuildSceneQueryBroadPhase();

private:
    std::unique_ptr<ConvexFactory> m_convex_factory;
    std::unique_ptr<ShapeFactory> m_shape_factory;
    std::unique_ptr<BroadPhase> m_sq_broad_phase;
};

}  // namespace toy_physics
