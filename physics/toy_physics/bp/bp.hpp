#pragma once
#include "toy_physics/common/common.hpp"

#include <span>

namespace toy_physics {

class Shape;

class BroadPhase {
public:
    virtual ~BroadPhase() = default;

    static void AttachIndexTo(Shape& shape, TightPoolID index);
    static TightPoolID FetchIndexFrom(const Shape& shape);

    virtual void AddObjects(std::span<Shape>) = 0;
    virtual void RemoveObjects(std::span<Shape>) = 0;
    virtual void ApplyModify() = 0;
    [[nodiscard]] virtual bool ShouldRebuild() const = 0;
};

}  // namespace toy_physics
