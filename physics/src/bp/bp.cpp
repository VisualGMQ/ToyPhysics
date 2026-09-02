#include "toy_physics/bp/bp.hpp"
#include "toy_physics/shape.hpp"

namespace toy_physics {

void BroadPhase::AttachIndexTo(Shape& shape, TightPoolID id) {
    shape.m_sq_index = id;
}

TightPoolID BroadPhase::FetchIndexFrom(const Shape& shape) {
    return shape.m_sq_index;
}

}  // namespace toy_physics
