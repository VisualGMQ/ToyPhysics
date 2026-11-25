#include "toy_physics/bp/bp.hpp"

#include "toy_physics/rigid_sim/shape.hpp"

namespace toy_physics {

void BroadPhase::AttachIndexTo(Shape& shape, TightPoolID index,
                               IndexSlot slot) {
    if (slot == IndexSlot::Simulation) {
        shape.m_sim_index = index;
    } else {
        shape.m_sq_index = index;
    }
}

TightPoolID BroadPhase::FetchIndexFrom(const Shape& shape, IndexSlot slot) {
    return slot == IndexSlot::Simulation ? shape.m_sim_index : shape.m_sq_index;
}

void BroadPhase::AddObjects(std::span<Shape*> shapes) {
    for (Shape* shape : shapes) {
        if (shape != nullptr) {
            AddObjects(std::span<Shape>{shape, 1});
        }
    }
}

void BroadPhase::RemoveObjects(std::span<Shape*> shapes) {
    for (Shape* shape : shapes) {
        if (shape != nullptr) {
            RemoveObjects(std::span<Shape>{shape, 1});
        }
    }
}

}  // namespace toy_physics
