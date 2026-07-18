#include "toy_physics/algorithm.hpp"

#include "toy_physics/check.hpp"

namespace toy_physics {

bool IsAABBIntersect(Vector3 min1, Vector3 max1, Vector3 min2, Vector3 max2) {
    TOY_ENSURE_R_FALSE((min1.array() <= max1.array()).all() &&
                       (min2.array() <= max2.array()).all());

    return (min1.array() < max2.array()).all() &&
           (min2.array() < max1.array()).all();
}

Vector3 GetPlaneNearestPoint(Vector3 p, Vector3 plan_pt, Vector3 normal) {
    TOY_CHECK(normal.isUnitary());

    real dist = normal.dot(p - plan_pt);
    return p - normal * dist;
}

Vector3 GetSegmentNearestPoint(Vector3 p, Vector3 q, Vector3 dir, real len) {
    TOY_CHECK(dir.isUnitary());
    TOY_CHECK(len >= 0);

    real proj_len = (p - q).dot(dir);
    proj_len = std::clamp<real>(proj_len, 0.0, len);
    return q + dir * proj_len;
}

Vector3 GetAABBNearestPoint(Vector3 p, Vector3 min, Vector3 max) {
    TOY_CHECK((min.array() <= max.array()).all());
    return p.cwiseMax(min).cwiseMin(max);
}

Vector3 GetOBBNearestPoint(Vector3 p, Vector3 center,
                           const std::array<Vector3, 3>& axises,
                           Vector3 half_extent) {
    TOY_CHECK((half_extent.array() > 0).all());

    Vector3 q = p - center;
    Vector3 result = center;
    for (size_t i = 0; i < axises.size(); ++i) {
        Vector3 axis = axises[i];
        real proj = q.dot(axis);
        real half_len = half_extent[i];
        result += std::clamp<real>(proj, -half_len, half_len) * axis;
    }
    return result;
}

Vector3 GetTriangleNearestPoint(Vector3 p, Vector3 q1, Vector3 q2, Vector3 q3) {
    Vector3 e1 = q2 - q1;
    Vector3 e2 = q3 - q1;
    Vector3 n = e1.cross(e2);
    Vector3 pt_on_plane = p;
    real n_len_sq = n.squaredNorm();
    if (n_len_sq > std::numeric_limits<real>::epsilon()) {
        pt_on_plane = GetPlaneNearestPoint(p, q1, n / std::sqrt(n_len_sq));
    }
    BarycentricCoord barycentric{q1, q2, q3, pt_on_plane,
                                 BarycentricPolicy::StopWhenNegative};
    if (barycentric.IsValid()) {
        return pt_on_plane;
    }

    if (barycentric.m_alpha < 0) {
        return GetSegmentNearestPoint(p, q2, q3);
    }
    if (barycentric.m_beta < 0) {
        return GetSegmentNearestPoint(p, q1, q3);
    }
    return GetSegmentNearestPoint(p, q1, q2);
}

Vector3 GetTetrahedronNearestPoint(Vector3 p, Vector3 a, Vector3 b, Vector3 c,
                                   Vector3 d) {
    TetrahedronBarycentric bc(a, b, c, d, p,
                              BarycentricPolicy::StopWhenNegative);
    if (bc.IsValid()) {
        return p;
    }

    if (bc.m_alpha < 0) return GetTriangleNearestPoint(p, b, c, d);
    if (bc.m_beta < 0) return GetTriangleNearestPoint(p, a, c, d);
    if (bc.m_gamma < 0) return GetTriangleNearestPoint(p, a, b, d);
    return GetTriangleNearestPoint(p, a, b, c);
}

Vector3 GetSphereNearestPoint(Vector3 p, Vector3 center, real radius) {
    TOY_CHECK(radius >= 0);

    Vector3 dir = p - center;
    real squared_dist = dir.squaredNorm();
    if (squared_dist <= radius * radius) {
        return p;
    }

    real dist = std::sqrt(squared_dist);
    dir = dir / dist;
    return center + dir * radius;
}

Vector3 GetSegmentNearestPoint(Vector3 p, Vector3 q1, Vector3 q2) {
    Vector3 dir = q2 - q1;
    real len = dir.norm();
    if (std::abs(len) <= std::numeric_limits<real>::epsilon()) {
        return p;
    }
    Vector3 out_result = GetSegmentNearestPoint(p, q1, dir / len, len);
    return out_result;
}

std::pair<Vector3, Vector3> GetSegSegNearestPoints(Vector3 p1, Vector3 p2,
                                                   Vector3 q1, Vector3 q2) {
    Vector3 d1 = p2 - p1;
    Vector3 d2 = q2 - q1;
    Vector3 r = p1 - q1;
    real a = d1.squaredNorm();
    real c = d2.squaredNorm();
    real b = d1.dot(d2);
    real d = d1.dot(r);
    real e = d2.dot(r);

    real best_s = 0;
    real best_t = 0;
    real best_dist_sq = (p1 - q1).squaredNorm();

    auto update_best = [&](real s, real t) {
        Vector3 c1 = p1 + s * d1;
        Vector3 c2 = q1 + t * d2;
        real dist_sq = (c1 - c2).squaredNorm();
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_s = s;
            best_t = t;
        }
    };

    real denom = a * c - b * b;
    if (std::abs(denom) > std::numeric_limits<real>::epsilon()) {
        real s = std::clamp<real>((b * e - c * d) / denom, 0, 1);
        real t = std::clamp<real>((a * e - b * d) / denom, 0, 1);
        update_best(s, t);
    }

    if (c > std::numeric_limits<real>::epsilon()) {
        update_best(0, std::clamp<real>(e / c, 0, 1));
        update_best(1, std::clamp<real>((e + b) / c, 0, 1));
    }

    if (a > std::numeric_limits<real>::epsilon()) {
        update_best(std::clamp<real>((-d) / a, 0, 1), 0);
        update_best(std::clamp<real>((b - d) / a, 0, 1), 1);
    }

    return {p1 + best_s * d1, q1 + best_t * d2};
}

std::pair<Vector3, Vector3> GetLineLineNearestPoints(Vector3 p, Vector3 d1,
                                                      Vector3 q, Vector3 d2) {
    Vector3 r = p - q;
    real a = d1.squaredNorm();
    real c = d2.squaredNorm();
    real b = d1.dot(d2);
    real d = d1.dot(r);
    real e = d2.dot(r);

    real det = a * c - b * b;
    if (std::abs(det) > std::numeric_limits<real>::epsilon()) {
        real s = (b * e - c * d) / det;
        real t = (a * e - b * d) / det;
        return {p + s * d1, q + t * d2};
    }
    // Parallel: project r onto line 2
    real t = (c > std::numeric_limits<real>::epsilon()) ? (e / c) : real{0};
    return {p, q + t * d2};
}

Vector3 GetCapsuleNearestPoint(Vector3 p, Vector3 center, Vector3 dir,
                               real half_height, real radius) {
    TOY_CHECK(dir.isUnitary());
    TOY_CHECK(radius >= 0);
    Vector3 half_axis = dir * half_height;
    Vector3 nearest_pt =
        GetSegmentNearestPoint(p, center - half_axis, center + half_axis);
    return nearest_pt + (p - nearest_pt).normalized() * radius;
}

real GetPtToPlaneDist(Vector3 p, Vector3 plan_pt, Vector3 normal) {
    TOY_CHECK(normal.isUnitary());

    return normal.dot(p - plan_pt);
}

real GetPtToSegmentDist(Vector3 p, Vector3 q, Vector3 dir, real len) {
    return std::sqrt(GetPtToSegmentSquaredDist(p, q, dir, len));
}

real GetPtToSegmentSquaredDist(Vector3 p, Vector3 q, Vector3 dir, real len) {
    Vector3 nearest_pt = GetSegmentNearestPoint(p, q, dir, len);
    return (p - nearest_pt).squaredNorm();
}

real GetTriangleSquaredDist(Vector3 p, Vector3 q1, Vector3 q2, Vector3 q3) {
    return (GetTriangleNearestPoint(p, q1, q2, q3) - p).squaredNorm();
}

real GetTriangleDist(Vector3 p, Vector3 q1, Vector3 q2, Vector3 q3) {
    return std::sqrt(GetTriangleSquaredDist(p, q1, q2, q3));
}

real GetTetrahedronSquaredDist(Vector3 p, Vector3 a, Vector3 b, Vector3 c,
                               Vector3 d) {
    return (GetTetrahedronNearestPoint(p, a, b, c, d) - p).squaredNorm();
}

real GetTetrahedronDist(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
    return std::sqrt(GetTetrahedronSquaredDist(p, a, b, c, d));
}

real GetSphereDist(Vector3 p, Vector3 center, real radius) {
    TOY_CHECK(radius >= 0);

    Vector3 dir = center - p;
    real squared_dist = dir.squaredNorm();
    if (squared_dist <= radius * radius) {
        return 0;
    }

    real dist = std::sqrt(squared_dist) - radius;
    // NOTE: when squared_dist and radius soo small, dist may be negative
    return std::max<real>(dist, 0.0);
}

real GetPtToAABBSquaredDist(Vector3 p, Vector3 min, Vector3 max) {
    Vector3 nearest_pt = GetAABBNearestPoint(p, min, max);
    return (nearest_pt - p).squaredNorm();
}

// quicker than (p - GetOBBNearestPoint()).squaredNorm()
real GetPtToOBBSquaredDist(Vector3 p, Vector3 center,
                           const std::array<Vector3, 3> axises,
                           Vector3 half_extent) {
    TOY_CHECK((half_extent.array() > 0).all());

    Vector3 q = p - center;
    real dist = 0;
    for (size_t i = 0; i < axises.size(); ++i) {
        real proj = q.dot(axises[i]);
        real excess = std::abs(proj) - half_extent[i];
        if (excess > 0) {
            dist += excess * excess;
        }
    }
    return dist;
}

}  // namespace toy_physics
