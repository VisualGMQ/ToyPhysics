#pragma once

#include <utility>
#include "toy_physics/math.hpp"

namespace toy_physics {

// intersects
bool IsAABBIntersect(Vector3 min1, Vector3 max1, Vector3 min2, Vector3 max2);

template <int N>
bool IsKDOPIntersect(Vector<N> mins1, Vector<N> maxs1, Vector<N> mins2,
                     Vector<N> maxs2) {
    return (mins1.array() < maxs2.array()).all() &&
           (mins2.array() < maxs1.array()).all();
}

// nearest points
Vector3 GetPlaneNearestPoint(Vector3 p, Vector3 plan_pt, Vector3 normal);
Vector3 GetSegmentNearestPoint(Vector3 p, Vector3 q1, Vector3 q2);
Vector3 GetSegmentNearestPoint(Vector3 p, Vector3 q, Vector3 dir, real len);
Vector3 GetAABBNearestPoint(Vector3 p, Vector3 min, Vector3 max);
Vector3 GetOBBNearestPoint(Vector3 p, Vector3 center,
                           const std::array<Vector3, 3>& axises,
                           Vector3 half_extent);
Vector3 GetTriangleNearestPoint(Vector3 p, Vector3 q1, Vector3 q2, Vector3 q3);
Vector3 GetTetrahedronNearestPoint(Vector3 p, Vector3 a, Vector3 b, Vector3 c,
                                   Vector3 d);
Vector3 GetSphereNearestPoint(Vector3 p, Vector3 center, real radius);
std::pair<Vector3, Vector3> GetSegSegNearestPoints(Vector3 p1, Vector3 p2,
                                                   Vector3 q1, Vector3 q2);
std::pair<Vector3, Vector3> GetLineLineNearestPoints(Vector3 p, Vector3 d1,
                                                      Vector3 q, Vector3 d2);
Vector3 GetCapsuleNearestPoint(Vector3 p, Vector3 center, Vector3 dir,
                               real half_height, real radius);

// distance to

/**
 * @return distance to plane. > 0 means p on then plane font side, <
 * 0 means opposite
 */
real GetPtToPlaneDist(Vector3 p, Vector3 plan_pt, Vector3 normal);
real GetPtToSegmentDist(Vector3 p, Vector3 q, Vector3 dir, real len);
real GetPtToSegmentSquaredDist(Vector3 p, Vector3 q, Vector3 dir, real len);
real GetTriangleSquaredDist(Vector3 p, Vector3 q1, Vector3 q2, Vector3 q3);
real GetTriangleDist(Vector3 p, Vector3 q1, Vector3 q2, Vector3 q3);
real GetTetrahedronSquaredDist(Vector3 p, Vector3 a, Vector3 b, Vector3 c,
                               Vector3 d);
real GetTetrahedronDist(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d);
real GetSphereDist(Vector3 p, Vector3 center, real radius);

/**
 * @param out_result  0 when p in AABB.
 */
real GetPtToAABBSquaredDist(Vector3 p, Vector3 min, Vector3 max);
real GetPtToOBBSquaredDist(Vector3 p, Vector3 center,
                           const std::array<Vector3, 3>& axises,
                           Vector3 half_extent);

}  // namespace toy_physics
