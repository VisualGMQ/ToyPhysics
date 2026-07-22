#pragma once

#include "toy_physics/math.hpp"
#include "toy_physics/gjk.hpp"
#include <span>
#include <utility>

namespace toy_physics {

// support point

/**
 * @return < 0 if failed, otherwise the index of pts
 */
int GetSupportPoint(std::span<const Vector3> pts, Vector3 dir);

Vector3 GetSphereSupportPoint(Vector3 c, real r, Vector3 dir);
Vector3 GetOBBSupportPoint(Vector3 center, std::span<const Vector3, 3> axises,
                           Vector3 half_extent, Vector3 dir);
Vector3 GetTetrahedronSupportPoint(Vector3 p1, Vector3 p2, Vector3 p3,
                                   Vector3 p4, Vector3 dir);
Vector3 GetCapsuleSupportPoint(Vector3 center, Vector3 axis, real half_height,
                               real radius, Vector3 dir);
Vector3 GetConvexSupportPoint(std::span<const Vector3> convex, Vector3 dir);
std::pair<real, real> GetBidirectionalProjection(std::span<const Vector3> pts,
                                                 Vector3 dir);

// intersects
bool IsAABBIntersect(Vector3 min1, Vector3 max1, Vector3 min2, Vector3 max2);

template <int N>
bool IsKDOPIntersect(Vector<real, N> mins1, Vector<real, N> maxs1,
                     Vector<real, N> mins2, Vector<real, N> maxs2) {
    return (mins1.array() < maxs2.array()).all() &&
           (mins2.array() < maxs1.array()).all();
}

bool IsSphereIntersect(Vector3 c1, real r1, Vector3 c2, real r2);
bool IsSphereAABBIntersect(Vector3 c, real r, Vector3 min, Vector3 max);
bool IsSphereOBBIntersect(Vector3 c, real r, Vector3 center,
                          std::span<const Vector3, 3> axises,
                          Vector3 half_extent);
bool IsOBBsIntersect(Vector3 center1, std::span<const Vector3, 3> axises1,
                     Vector3 half_extent1, Vector3 center2,
                     std::span<const Vector3, 3> axises2, Vector3 half_extent2);
bool IsTetrahedronsIntersect(std::span<const Vector3, 4> pts1,
                             std::span<const Vector3, 4> pts2);
bool IsTetrahedronConvexIntersect(std::span<const Vector3, 4> pts,
                                  std::span<const Vector3> convex);
bool IsConvexesIntersect(std::span<const Vector3> pts1,
                         std::span<const Vector3> pts2);
bool IsCapsuleOBBIntersect(Vector3 center, Vector3 axis, real half_height,
                           real radius, Vector3 obb_center,
                           std::span<const Vector3, 3> obb_axises,
                           Vector3 obb_half_extent);
bool IsCapsuleTetrahedronIntersect(Vector3 center, Vector3 axis,
                                   real half_height, real radius, Vector3 p1,
                                   Vector3 p2, Vector3 p3, Vector3 p4);
bool IsCapsuleConvexIntersect(Vector3 center, Vector3 axis, real half_height,
                              real radius, std::span<const Vector3> convex);
bool IsSphereConvexIntersect(Vector3 c, real r,
                             std::span<const Vector3> convex);
bool IsOBBTetrahedronIntersect(Vector3 center,
                               std::span<const Vector3, 3> axises,
                               Vector3 half_extent, Vector3 p1, Vector3 p2,
                               Vector3 p3, Vector3 p4);
bool IsOBBConvexIntersect(Vector3 center, std::span<const Vector3, 3> axises,
                          Vector3 half_extent, std::span<const Vector3> convex);

// MTD
bool GetSpheresMTD(Vector3 c1, real r1, Vector3 c2, real r2, Vector3* mtd);
bool GetOBBsMTD(Vector3 center1, std::span<const Vector3, 3> axises1,
                Vector3 half_extent1, Vector3 center2,
                std::span<const Vector3, 3> axises2, Vector3 half_extent2,
                Vector3* mtd);
bool GetSphereOBBMTD(Vector3 c, real r, Vector3 center,
                     std::span<const Vector3, 3> axises, Vector3 half_extent,
                     Vector3* mtd);
bool GetSphereTetrahedronMTD(Vector3 c, real r, std::span<const Vector3, 4> pts,
                             Vector3* mtd);
bool GetSphereCapsuleMTD(Vector3 c, real r, Vector3 center, Vector3 axis,
                         real half_height, real radius, Vector3* mtd);
bool GetTetrahedronsMTD(std::span<const Vector3, 4> pts1,
                        std::span<const Vector3, 4> pts2, MTD& mtd);
bool GetOBBTetrahedronMTD(Vector3 center, std::span<const Vector3, 3> axises,
                          Vector3 half_extent, Vector3 p1, Vector3 p2,
                          Vector3 p3, Vector3 p4, Vector3* mtd);
bool GetConvexesMTD(std::span<const Vector3> pts1,
                    std::span<const Vector3> pts2, Vector3* mtd);
bool GetTetrahedronConvexMTD(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4,
                             std::span<const Vector3> convex, Vector3* mtd);
bool GetCapsuleTetrahedronMTD(Vector3 center, Vector3 axis, real half_height,
                              real radius, Vector3 p1, Vector3 p2, Vector3 p3,
                              Vector3 p4, Vector3* mtd);
bool GetCapsuleConvexMTD(Vector3 center, Vector3 axis, real half_height,
                         real radius, std::span<const Vector3> convex,
                         Vector3* mtd);
bool GetOBBConvexMTD(Vector3 center, std::span<const Vector3, 3> axises,
                     Vector3 half_extent, std::span<const Vector3> convex,
                     Vector3* mtd);

// nearest points
Vector3 GetPlaneNearestPoint(Vector3 p, Vector3 plan_pt, Vector3 normal);
Vector3 GetSegmentNearestPoint(Vector3 p, Vector3 q1, Vector3 q2);
Vector3 GetSegmentNearestPoint(Vector3 p, Vector3 q, Vector3 dir, real len);
Vector3 GetAABBNearestPoint(Vector3 p, Vector3 min, Vector3 max);
Vector3 GetOBBNearestPoint(Vector3 p, Vector3 center,
                           std::span<const Vector3, 3> axises,
                           Vector3 half_extent);
Vector3 GetTriangleNearestPoint(Vector3 p, Vector3 q1, Vector3 q2, Vector3 q3,
                                real tolerance = kDefaultRealTolerance);
Vector3 GetTetrahedronNearestPoint(Vector3 p, std::span<const Vector3, 4> pts,
                                   real tolerance = kDefaultRealTolerance);
Vector3 GetSphereNearestPoint(Vector3 p, Vector3 center, real radius);
std::pair<Vector3, Vector3> GetSegSegNearestPoints(Vector3 p1, Vector3 p2,
                                                   Vector3 q1, Vector3 q2);
std::pair<Vector3, Vector3> GetLineLineNearestPoints(Vector3 p, Vector3 d1,
                                                     Vector3 q, Vector3 d2);
Vector3 GetCapsuleNearestPoint(Vector3 p, Vector3 center, Vector3 dir,
                               real half_height, real radius);
Vector3 GetConvexNearestPoint(Vector3 p, std::span<const Vector3> convex);
std::pair<Vector3, Vector3> GetTetrahedronsNearestPoints(
    std::span<const Vector3, 4> tet1, std::span<const Vector3, 4> tet2);
std::pair<Vector3, Vector3> GetConvexesNearestPoints(
    std::span<const Vector3> convex1, std::span<const Vector3> convex2);

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
real GetTetrahedronSquaredDist(Vector3 p, std::span<const Vector3, 4> pts);
real GetTetrahedronDist(Vector3 p, std::span<const Vector3, 4> pts);
real GetSphereDist(Vector3 p, Vector3 center, real radius);

/**
 * @return 0 when p in AABB.
 */
real GetPtToAABBSquaredDist(Vector3 p, Vector3 min, Vector3 max);
real GetPtToOBBSquaredDist(Vector3 p, Vector3 center,
                           std::span<const Vector3, 3> axises,
                           Vector3 half_extent);
real GetConvexesDist(std::span<const Vector3> pts1,
                     std::span<const Vector3> pts2);
real GetTetrahedronConvexDist(std::span<const Vector3> tet,
                              std::span<const Vector3> convex);
real GetTetrahedronsDist(std::span<const Vector3, 4> tet1,
                         std::span<const Vector3, 4> tet2);
real GetCapsuleOBBDist(Vector3 center, Vector3 axis, real half_height,
                       real radius, Vector3 obb_center,
                       std::span<const Vector3, 3> obb_axises,
                       Vector3 obb_half_extent);
real GetSphereConvexDist(Vector3 c, real r, std::span<const Vector3> convex);
real GetOBBTetrahedronDist(Vector3 center, std::span<const Vector3, 3> axises,
                           Vector3 half_extent, Vector3 p1, Vector3 p2,
                           Vector3 p3, Vector3 p4);
real GetOBBConvexDist(Vector3 center, std::span<const Vector3, 3> axises,
                      Vector3 half_extent, std::span<const Vector3> convex);

class PolygonSupportFunction : public SupportFunction {
public:
    explicit PolygonSupportFunction(std::span<const Vector3> pts);
    [[nodiscard]] Vector3 operator()(Vector3 dir) const override;

private:
    std::span<const Vector3> m_pts;
};

class SphereSupportFunction : public SupportFunction {
public:
    explicit SphereSupportFunction(Vector3 center, real radius);
    [[nodiscard]] Vector3 operator()(Vector3 dir) const override;

private:
    Vector3 m_center;
    real m_radius;
};

class CapsuleSupportFunction : public SupportFunction {
public:
    explicit CapsuleSupportFunction(Vector3 center, Vector3 axis,
                                    real half_height, real radius);
    [[nodiscard]] Vector3 operator()(Vector3 dir) const override;

private:
    Vector3 m_center;
    Vector3 m_axis;
    real m_half_height;
    real m_radius;
};

class PointSupportFunction : public SupportFunction {
public:
    explicit PointSupportFunction(Vector3 center);
    [[nodiscard]] Vector3 operator()(Vector3 dir) const override;

private:
    Vector3 m_center;
};

class OBBSupportFunction : public SupportFunction {
public:
    explicit OBBSupportFunction(Vector3 center,
                                std::span<const Vector3, 3> axises,
                                Vector3 half_extent);
    [[nodiscard]] Vector3 operator()(Vector3 dir) const override;

private:
    Vector3 m_center;
    std::array<Vector3, 3> m_axises;
    Vector3 m_half_extent;
};

}  // namespace toy_physics
