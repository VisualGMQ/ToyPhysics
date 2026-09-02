#pragma once
#include "toy_physics/common/math.hpp"

namespace toy_physics {

struct BoundingBox {
    Vector3 m_min;
    Vector3 m_max;

    BoundingBox();
    BoundingBox(Vector3 min, Vector3 max);

    static BoundingBox FromCenter(Vector3 center, Vector3 half_size);
    static BoundingBox FromMinMax(Vector3 min, Vector3 max);

    [[nodiscard]] bool IsValid() const;

    [[nodiscard]] bool IsIntersect(const BoundingBox&) const;
    [[nodiscard]] BoundingBox Intersect(const BoundingBox&) const;

    [[nodiscard]] BoundingBox Merge(const BoundingBox&) const;
    [[nodiscard]] real Area() const;
};

using AABB = BoundingBox;

}
