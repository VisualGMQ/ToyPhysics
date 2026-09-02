#pragma once

#include "toy_physics/geometry/bounding_box.hpp"
#include "toy_physics/geometry/box.hpp"
#include "toy_physics/geometry/capsule.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/geometry/cylinder.hpp"
#include "toy_physics/geometry/sphere.hpp"
#include <array>

namespace toy_physics {

AABB BuildBV(const BoxGeometry& box, Quaternion rotation);
AABB BuildBV(const SphereGeometry& sphere);
AABB BuildBV(const CapsuleGeometry& capsule, Quaternion rotation);
AABB BuildBV(const CylinderGeometry& cylinder, Quaternion rotation);
AABB BuildBV(const ConvexHullGeometry& convex, Quaternion rotation);
AABB BuildBV(const std::array<Vector3, 3>& vertices);

AABB BuildBV(const Geometry& geometry, Quaternion rotation);

}  // namespace toy_physics
