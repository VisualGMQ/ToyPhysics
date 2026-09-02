#include "toy_physics/geometry/convex_hull.hpp"

#include "toy_physics/common/check.hpp"

namespace toy_physics {

ConvexHullGeometry::ConvexHullGeometry() : Geometry(Type::ConvexHull) {}

ConvexHullGeometry::ConvexHullGeometry(const ConvexData& convex)
    : Geometry(Type::ConvexHull), m_convex{&convex} {
    TOY_CHECK(IsValid());
}

bool ConvexHullGeometry::IsValid() const {
    return m_convex != nullptr;
}

const ConvexData* ConvexHullGeometry::GetConvex() const {
    return m_convex;
}

}  // namespace toy_physics