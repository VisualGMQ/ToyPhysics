#include "toy_physics/algorithm.hpp"

#include "toy_physics/check.hpp"
#include "toy_physics/epa.hpp"
#include "toy_physics/gjk.hpp"

namespace toy_physics {

int GetSupportPoint(std::span<const Vector3> pts, Vector3 dir) {
    int index = -1;
    real max = -std::numeric_limits<real>::max();
    for (size_t i = 0; i < pts.size(); ++i) {
        real dot = pts[i].dot(dir);
        if (dot > max) {
            max = dot;
            index = i;
        }
    }
    return index;
}

static Vector3 GetCentroid(std::span<const Vector3> pts) {
    Vector3 c = Vector3::Zero();
    for (const auto& p : pts) {
        c += p;
    }
    return c / static_cast<real>(pts.size());
}

static Vector3 GetInitDir(Vector3 c1, Vector3 c2) {
    Vector3 dir = c2 - c1;
    if (dir.squaredNorm() <= kREAL_EPSILON) {
        dir = Vector3::UnitX();
    }
    return dir;
}

Vector3 GetSphereSupportPoint(Vector3 c, real r, Vector3 dir) {
    TOY_CHECK(dir.isUnitary());

    return c + dir * r;
}

Vector3 GetOBBSupportPoint(Vector3 center, std::span<const Vector3, 3> axises,
                           Vector3 half_extent, Vector3 dir) {
    TOY_CHECK(!axises.empty());

    Vector3 result = center;
    for (size_t i = 0; i < axises.size(); ++i) {
        real proj = dir.dot(axises[i]);
        result += (proj >= 0 ? half_extent[i] : -half_extent[i]) * axises[i];
    }
    return result;
}

Vector3 GetTetrahedronSupportPoint(Vector3 p1, Vector3 p2, Vector3 p3,
                                   Vector3 p4, Vector3 dir) {
    real d1 = p1.dot(dir);
    real d2 = p2.dot(dir);
    real d3 = p3.dot(dir);
    real d4 = p4.dot(dir);
    if (d1 >= d2 && d1 >= d3 && d1 >= d4) return p1;
    if (d2 >= d3 && d2 >= d4) return p2;
    if (d3 >= d4) return p3;
    return p4;
}

Vector3 GetCapsuleSupportPoint(Vector3 center, Vector3 axis, real half_height,
                               real radius, Vector3 dir) {
    Vector3 p1 = center + axis * half_height;
    Vector3 p2 = center - axis * half_height;
    Vector3 seg = dir.dot(p1) >= dir.dot(p2) ? p1 : p2;
    return seg + dir * radius;
}

Vector3 GetConvexSupportPoint(std::span<const Vector3> convex, Vector3 dir) {
    TOY_CHECK(!convex.empty());

    size_t best = 0;
    real best_dot = -std::numeric_limits<real>::max();
    for (size_t i = 0; i < convex.size(); ++i) {
        real d = convex[i].dot(dir);
        if (d > best_dot) {
            best_dot = d;
            best = i;
        }
    }
    return convex[best];
}

std::pair<real, real> GetBidirectionalProjection(std::span<const Vector3> pts,
                                                 Vector3 dir) {
    TOY_ENSURE_RV(!pts.empty(), std::make_pair<real>(0, 0));

    real min_proj = pts[0].dot(dir);
    real max_proj = min_proj;
    for (size_t i = 1; i < pts.size(); ++i) {
        real proj = pts[i].dot(dir);
        if (proj < min_proj) min_proj = proj;
        if (proj > max_proj) max_proj = proj;
    }
    return {min_proj, max_proj};
}

bool IsAABBIntersect(Vector3 min1, Vector3 max1, Vector3 min2, Vector3 max2) {
    TOY_ENSURE_R_FALSE((min1.array() <= max1.array()).all() &&
                       (min2.array() <= max2.array()).all());

    return (min1.array() < max2.array()).all() &&
           (min2.array() < max1.array()).all();
}

bool IsSphereIntersect(Vector3 c1, real r1, Vector3 c2, real r2) {
    TOY_CHECK(r1 >= 0 && r2 >= 0);

    real r = r1 + r2;
    return (c1 - c2).squaredNorm() < r * r;
}

bool IsSphereAABBIntersect(Vector3 c, real r, Vector3 min, Vector3 max) {
    TOY_CHECK(r >= 0);
    TOY_CHECK((min.array() <= max.array()).all());

    Vector3 nearest = GetAABBNearestPoint(c, min, max);
    return (nearest - c).squaredNorm() < r * r;
}

bool IsOBBsIntersect(Vector3 center1, std::span<const Vector3, 3> axises1,
                     Vector3 half_extent1, Vector3 center2,
                     std::span<const Vector3, 3> axises2,
                     Vector3 half_extent2) {
    std::array<Vector3, 15> axes;
    size_t count = 0;

    for (int i = 0; i < 3; ++i) axes[count++] = axises1[i];
    for (int i = 0; i < 3; ++i) axes[count++] = axises2[i];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vector3 c = axises1[i].cross(axises2[j]);
            real s = c.squaredNorm();
            if (s > std::numeric_limits<real>::epsilon()) {
                axes[count++] = c / std::sqrt(s);
            }
        }
    }

    for (size_t i = 0; i < count; ++i) {
        Vector3 axis = axes[i];

        real r1 = 0;
        for (int j = 0; j < 3; ++j)
            r1 += half_extent1[j] * std::abs(axis.dot(axises1[j]));
        real c1 = center1.dot(axis);

        real r2 = 0;
        for (int j = 0; j < 3; ++j)
            r2 += half_extent2[j] * std::abs(axis.dot(axises2[j]));
        real c2 = center2.dot(axis);

        if (std::abs(c1 - c2) - (r1 + r2) >= 0) return false;
    }
    return true;
}

bool IsTetrahedronsIntersect(std::span<const Vector3, 4> pts1,
                             std::span<const Vector3, 4> pts2) {
    PolygonSupportFunction support1{pts1}, support2{pts2};
    Vector3 dir = GetInitDir(GetCentroid(pts1), GetCentroid(pts2));
    return IsIntersect(support1, support2, dir, kDefaultRealTolerance);
}

bool GetOBBsMTD(Vector3 center1, std::span<const Vector3, 3> axises1,
                Vector3 half_extent1, Vector3 center2,
                std::span<const Vector3, 3> axises2, Vector3 half_extent2,
                Vector3* mtd) {
    std::array<Vector3, 15> proj_axises;
    size_t count = 0;

    for (int i = 0; i < 3; ++i) proj_axises[count++] = axises1[i];
    for (int i = 0; i < 3; ++i) proj_axises[count++] = axises2[i];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vector3 c = axises1[i].cross(axises2[j]);
            real s = c.squaredNorm();
            if (s > std::numeric_limits<real>::epsilon()) {
                proj_axises[count++] = c / std::sqrt(s);
            }
        }
    }

    real min_penetration = std::numeric_limits<real>::max();
    Vector3 best_dir = Vector3::Zero();

    for (size_t i = 0; i < count; ++i) {
        Vector3 axis = proj_axises[i];

        real r1 = 0;
        for (int j = 0; j < 3; ++j)
            r1 += half_extent1[j] * std::abs(axis.dot(axises1[j]));
        real c1 = center1.dot(axis);

        real r2 = 0;
        for (int j = 0; j < 3; ++j)
            r2 += half_extent2[j] * std::abs(axis.dot(axises2[j]));
        real c2 = center2.dot(axis);

        real separation = std::abs(c1 - c2) - (r1 + r2);
        if (separation >= 0) return false;

        real penetration = -separation;
        Vector3 dir = (c2 >= c1) ? axis : -axis;

        if (penetration < min_penetration) {
            min_penetration = penetration;
            best_dir = dir;
        }
    }

    if (mtd) *mtd = best_dir * min_penetration;
    return true;
}

bool GetSphereOBBMTD(Vector3 c, real r, Vector3 center,
                     std::span<const Vector3, 3> axises, Vector3 half_extent,
                     Vector3* mtd) {
    Vector3 nearest = GetOBBNearestPoint(c, center, axises, half_extent);
    Vector3 d = nearest - c;
    real dist = d.squaredNorm();

    if (dist <= std::numeric_limits<real>::epsilon()) {
        Vector3 q = c - center;
        real min_escape = std::numeric_limits<real>::max();
        Vector3 best_dir = Vector3::Zero();
        for (int i = 0; i < 3; ++i) {
            real proj = q.dot(axises[i]);
            real esc_pos = half_extent[i] - proj;
            real esc_neg = half_extent[i] + proj;
            if (esc_pos < min_escape) {
                min_escape = esc_pos;
                best_dir = axises[i];
            }
            if (esc_neg < min_escape) {
                min_escape = esc_neg;
                best_dir = -axises[i];
            }
        }
        if (mtd) *mtd = best_dir * min_escape;
        return true;
    }

    dist = std::sqrt(dist);
    if (dist >= r) return false;

    if (mtd) *mtd = d / dist * r;
    return true;
}

bool GetSphereTetrahedronMTD(Vector3 c, real r, std::span<const Vector3, 4> pts,
                             Vector3* mtd) {
    Vector3 nearest = GetTetrahedronNearestPoint(c, pts);
    Vector3 d = nearest - c;
    real dist = d.squaredNorm();

    if (dist <= std::numeric_limits<real>::epsilon()) {
        std::array norms = {(pts[2] - pts[1]).cross(pts[3] - pts[1]),
                            (pts[2] - pts[0]).cross(pts[3] - pts[0]),
                            (pts[1] - pts[0]).cross(pts[3] - pts[0]),
                            (pts[1] - pts[0]).cross(pts[2] - pts[0])};
        std::array verts = {pts[1], pts[0], pts[0], pts[0]};

        real min_escape = std::numeric_limits<real>::max();
        Vector3 best_dir = Vector3::Zero();
        for (int i = 0; i < 4; ++i) {
            Vector3 n = norms[i];
            real s = n.squaredNorm();
            if (s < std::numeric_limits<real>::epsilon()) continue;
            n /= std::sqrt(s);
            real signed_dist = n.dot(c - verts[i]);
            real escape = std::abs(signed_dist);
            if (escape < min_escape) {
                min_escape = escape;
                best_dir = -n * signed_dist;
            }
        }
        if (mtd) *mtd = best_dir.normalized() * (min_escape + r);
        return true;
    }

    dist = std::sqrt(dist);
    if (dist >= r) return false;

    if (mtd) *mtd = d / dist * r;
    return true;
}

bool GetSphereCapsuleMTD(Vector3 c, real r, Vector3 center, Vector3 axis,
                         real half_height, real radius, Vector3* mtd) {
    Vector3 seg = GetSegmentNearestPoint(c, center - axis * half_height,
                                         center + axis * half_height);
    Vector3 d = seg - c;
    real dist = d.squaredNorm();

    real total_r = r + radius;

    if (dist <= std::numeric_limits<real>::epsilon()) {
        // Sphere center on the capsule axis: escape radially
        Vector3 esc = axis.cross(Vector3::UnitX());
        if (esc.squaredNorm() < std::numeric_limits<real>::epsilon()) {
            esc = axis.cross(Vector3::UnitY());
        }
        if (mtd) {
            *mtd = esc.normalized() * total_r;
        }
        return true;
    }

    dist = std::sqrt(dist);
    if (dist >= total_r) {
        return false;
    }

    if (mtd) {
        *mtd = d / dist * total_r;
    }
    return true;
}

bool GetSpheresMTD(Vector3 c1, real r1, Vector3 c2, real r2, Vector3* mtd) {
    Vector3 d = c2 - c1;
    real squared_dist = d.squaredNorm();
    real r = r1 + r2;
    if (squared_dist >= r * r) {
        return false;
    }

    real dist = std::sqrt(squared_dist);

    if (mtd) {
        // two circle center overlapped, choose a direction casually
        if (std::abs(dist) < std::numeric_limits<real>::epsilon()) {
            *mtd = Vector3::UnitX() * r;
        } else {
            *mtd = d / dist * r;
        }
    }

    return true;
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
                           std::span<const Vector3, 3> axises,
                           Vector3 half_extent) {
    TOY_CHECK((half_extent.array() > 0).all());
    TOY_CHECK(!axises.empty());

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

Vector3 GetTriangleNearestPoint(Vector3 p, Vector3 q1, Vector3 q2, Vector3 q3,
                                real tolerance) {
    Vector3 d12 = q2 - q1;
    Vector3 d13 = q3 - q1;
    Vector3 n = d12.cross(d13);
    Vector3 pt_on_plane = p;
    real n_len_sq = n.squaredNorm();
    if (n_len_sq > tolerance) {
        pt_on_plane = GetPlaneNearestPoint(p, q1, n / std::sqrt(n_len_sq));
    }
    BarycentricCoord barycentric{q1, q2, q3, pt_on_plane,
                                 BarycentricPolicy::StopWhenNegative};
    if (barycentric.IsInnerPoint()) {
        return barycentric.m_alpha * q1 + barycentric.m_beta * q2 +
               barycentric.m_gamma * q3;
    }

    if (!barycentric.IsValid()) {
        // Degenerate: brute-force vertices and edges
        Vector3 best = q1;
        real best2 = (p - q1).squaredNorm();

        for (auto* v : {&q2, &q3}) {
            real d2 = (p - *v).squaredNorm();
            if (d2 < best2) {
                best2 = d2;
                best = *v;
            }
        }

        Vector3 edges[3][2] = {
            {q1, q2},
            {q1, q3},
            {q2, q3}
        };
        for (auto& e : edges) {
            Vector3 dir = e[1] - e[0];
            if (dir.squaredNorm() > std::numeric_limits<real>::epsilon()) {
                Vector3 np = GetSegmentNearestPoint(p, e[0], e[1]);
                real d2 = (p - np).squaredNorm();
                if (d2 < best2) {
                    best2 = d2;
                    best = np;
                }
            }
        }
        return best;
    }

    if (barycentric.m_alpha < 0) {
        return GetSegmentNearestPoint(p, q2, q3);
    }
    if (barycentric.m_beta < 0) {
        return GetSegmentNearestPoint(p, q1, q3);
    }
    return GetSegmentNearestPoint(p, q1, q2);
}

Vector3 GetTetrahedronNearestPoint(Vector3 p, std::span<const Vector3, 4> pts,
                                   real tolerance) {
    TetrahedronBarycentric bc(pts, p, BarycentricPolicy::StopWhenNegative);
    if (bc.IsInnerPoint()) {
        return p;
    }

    if (!bc.IsValid()) {
        // Degenerate: brute-force vertices, edges and faces
        Vector3 best = pts[0];
        real best2 = (p - pts[0]).squaredNorm();

        // vertices
        for (int i = 1; i < 4; ++i) {
            real d2 = (p - pts[i]).squaredNorm();
            if (d2 < best2) {
                best2 = d2;
                best = pts[i];
            }
        }

        // edges
        int epairs[6][2] = {
            {0, 1},
            {0, 2},
            {0, 3},
            {1, 2},
            {1, 3},
            {2, 3}
        };
        for (auto& pr : epairs) {
            Vector3 dir = pts[pr[1]] - pts[pr[0]];
            if (dir.squaredNorm() > tolerance) {
                Vector3 np = GetSegmentNearestPoint(p, pts[pr[0]], pts[pr[1]]);
                real d2 = (p - np).squaredNorm();
                if (d2 < best2) {
                    best2 = d2;
                    best = np;
                }
            }
        }

        // faces
        int fpairs[4][3] = {
            {1, 2, 3},
            {0, 2, 3},
            {0, 1, 3},
            {0, 1, 2}
        };
        for (auto& fp : fpairs) {
            Vector3 np =
                GetTriangleNearestPoint(p, pts[fp[0]], pts[fp[1]], pts[fp[2]]);
            real d2 = (p - np).squaredNorm();
            if (d2 < best2) {
                best2 = d2;
                best = np;
            }
        }
        return best;
    }

    if (bc.m_coord[0] < 0)
        return GetTriangleNearestPoint(p, pts[1], pts[2], pts[3]);
    if (bc.m_coord[1] < 0)
        return GetTriangleNearestPoint(p, pts[0], pts[2], pts[3]);
    if (bc.m_coord[2] < 0)
        return GetTriangleNearestPoint(p, pts[0], pts[1], pts[3]);
    return GetTriangleNearestPoint(p, pts[0], pts[1], pts[2]);
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

Vector3 GetConvexNearestPoint(Vector3 p, std::span<const Vector3> convex) {
    PointSupportFunction support1(p);
    PolygonSupportFunction support2(convex);

    Vector3 out_p1, out_p2;
    (void)CalcClosestPoints(support1, support2, out_p1, out_p2, kREAL_MAX,
                            kDefaultRealTolerance);
    return out_p2;
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

real GetTetrahedronSquaredDist(Vector3 p, std::span<const Vector3, 4> pts) {
    return (GetTetrahedronNearestPoint(p, pts) - p).squaredNorm();
}

real GetTetrahedronDist(Vector3 p, std::span<const Vector3, 4> pts) {
    return std::sqrt(GetTetrahedronSquaredDist(p, pts));
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
                           std::span<const Vector3, 3> axises,
                           Vector3 half_extent) {
    TOY_CHECK((half_extent.array() > 0).all());
    TOY_CHECK(!axises.empty());

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

bool IsTetrahedronConvexIntersect(std::span<const Vector3, 4> pts,
                                  std::span<const Vector3> convex) {
    PolygonSupportFunction s1(pts), s2(convex);
    Vector3 dir = GetInitDir(GetCentroid(pts), GetCentroid(convex));
    return IsIntersect(s1, s2, dir, kDefaultRealTolerance);
}

bool IsConvexesIntersect(std::span<const Vector3> pts1,
                         std::span<const Vector3> pts2) {
    PolygonSupportFunction s1(pts1), s2(pts2);
    Vector3 dir = GetInitDir(GetCentroid(pts1), GetCentroid(pts2));
    return IsIntersect(s1, s2, dir, kDefaultRealTolerance);
}

bool IsCapsuleOBBIntersect(Vector3 center, Vector3 axis, real half_height,
                           real radius, Vector3 obb_center,
                           std::span<const Vector3, 3> obb_axises,
                           Vector3 obb_half_extent) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    OBBSupportFunction s2(obb_center, obb_axises, obb_half_extent);
    return IsIntersect(s1, s2, GetInitDir(center, obb_center),
                       kDefaultRealTolerance);
}

bool IsCapsuleTetrahedronIntersect(Vector3 center, Vector3 axis,
                                   real half_height, real radius, Vector3 p1,
                                   Vector3 p2, Vector3 p3, Vector3 p4) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2(tv);
    return IsIntersect(s1, s2, GetInitDir(center, GetCentroid(tv)),
                       kDefaultRealTolerance);
}

bool IsCapsuleConvexIntersect(Vector3 center, Vector3 axis, real half_height,
                              real radius, std::span<const Vector3> convex) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    PolygonSupportFunction s2(convex);
    return IsIntersect(s1, s2, GetInitDir(center, GetCentroid(convex)),
                       kDefaultRealTolerance);
}

bool IsSphereConvexIntersect(Vector3 c, real r,
                             std::span<const Vector3> convex) {
    SphereSupportFunction s1(c, r);
    PolygonSupportFunction s2(convex);
    return IsIntersect(s1, s2, GetInitDir(c, GetCentroid(convex)),
                       kDefaultRealTolerance);
}

bool IsOBBTetrahedronIntersect(Vector3 center,
                               std::span<const Vector3, 3> axises,
                               Vector3 half_extent, Vector3 p1, Vector3 p2,
                               Vector3 p3, Vector3 p4) {
    OBBSupportFunction s1(center, axises, half_extent);
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2(tv);
    return IsIntersect(s1, s2, GetInitDir(center, GetCentroid(tv)),
                       kDefaultRealTolerance);
}

bool IsOBBConvexIntersect(Vector3 center, std::span<const Vector3, 3> axises,
                          Vector3 half_extent,
                          std::span<const Vector3> convex) {
    OBBSupportFunction s1(center, axises, half_extent);
    PolygonSupportFunction s2(convex);
    return IsIntersect(s1, s2, GetInitDir(center, GetCentroid(convex)),
                       kDefaultRealTolerance);
}

bool GetTetrahedronsMTD(std::span<const Vector3, 4> pts1,
                        std::span<const Vector3, 4> pts2, MTD& mtd) {
    PolygonSupportFunction s1{pts1};
    PolygonSupportFunction s2{pts2};

    Vector3 penetration, p1, p2;
    if (!CalcPenetrationDepth(s1, s2, penetration, p1, p2,
                              kDefaultRealTolerance)) {
        return false;
    }

    mtd.m_len = penetration.norm();
    if (mtd.m_len <= 0) {
        return false;
    }
    mtd.m_dir = penetration / mtd.m_len;
    return true;
}

bool GetOBBTetrahedronMTD(Vector3 center, std::span<const Vector3, 3> axises,
                          Vector3 half_extent, Vector3 p1, Vector3 p2,
                          Vector3 p3, Vector3 p4, Vector3* mtd) {
    OBBSupportFunction s1{center, axises, half_extent};
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2{std::span<const Vector3>(tv)};

    Vector3 penetration, wp1, wp2;
    if (!CalcPenetrationDepth(s1, s2, penetration, wp1, wp2,
                              kDefaultRealTolerance)) {
        return false;
    }
    if (penetration.norm() <= 0) return false;
    if (mtd) *mtd = penetration;
    return true;
}

bool GetConvexesMTD(std::span<const Vector3> pts1,
                    std::span<const Vector3> pts2, Vector3* mtd) {
    PolygonSupportFunction s1(pts1);
    PolygonSupportFunction s2(pts2);

    Vector3 penetration, p1, p2;
    if (!CalcPenetrationDepth(s1, s2, penetration, p1, p2,
                              kDefaultRealTolerance)) {
        return false;
    }
    if (penetration.norm() <= 0) return false;
    if (mtd) *mtd = penetration;
    return true;
}

bool GetTetrahedronConvexMTD(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4,
                             std::span<const Vector3> convex, Vector3* mtd) {
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s1{std::span<const Vector3>(tv)};
    PolygonSupportFunction s2{convex};

    Vector3 penetration, wp1, wp2;
    if (!CalcPenetrationDepth(s1, s2, penetration, wp1, wp2,
                              kDefaultRealTolerance)) {
        return false;
    }
    if (penetration.norm() <= 0) return false;
    if (mtd) *mtd = penetration;
    return true;
}

bool GetCapsuleTetrahedronMTD(Vector3 center, Vector3 axis, real half_height,
                              real radius, Vector3 p1, Vector3 p2, Vector3 p3,
                              Vector3 p4, Vector3* mtd) {
    CapsuleSupportFunction s1{center, axis, half_height, radius};
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2{std::span<const Vector3>(tv)};

    Vector3 penetration, wp1, wp2;
    if (!CalcPenetrationDepth(s1, s2, penetration, wp1, wp2,
                              kDefaultRealTolerance)) {
        return false;
    }
    if (penetration.norm() <= 0) return false;
    if (mtd) *mtd = penetration;
    return true;
}

bool GetCapsuleConvexMTD(Vector3 center, Vector3 axis, real half_height,
                         real radius, std::span<const Vector3> convex,
                         Vector3* mtd) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    PolygonSupportFunction s2(convex);

    Vector3 penetration, p1, p2;
    if (!CalcPenetrationDepth(s1, s2, penetration, p1, p2,
                              kDefaultRealTolerance)) {
        return false;
    }
    if (penetration.norm() <= 0) return false;
    if (mtd) *mtd = penetration;
    return true;
}

bool GetOBBConvexMTD(Vector3 center, std::span<const Vector3, 3> axises,
                     Vector3 half_extent, std::span<const Vector3> convex,
                     Vector3* mtd) {
    OBBSupportFunction s1(center, axises, half_extent);
    PolygonSupportFunction s2(convex);

    Vector3 penetration, p1, p2;
    if (!CalcPenetrationDepth(s1, s2, penetration, p1, p2,
                              kDefaultRealTolerance)) {
        return false;
    }
    if (penetration.norm() <= 0) return false;
    if (mtd) *mtd = penetration;
    return true;
}

std::pair<Vector3, Vector3> GetTetrahedronsNearestPoints(
    std::span<const Vector3, 4> tet1, std::span<const Vector3, 4> tet2) {
    PolygonSupportFunction s1(tet1), s2(tet2);
    Vector3 dir = GetInitDir(GetCentroid(tet1), GetCentroid(tet2));

    Vector3 p1, p2;
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        if (!CalcCommonPoint(s1, s2, dir, p1, p2, kDefaultRealTolerance)) {
            return {Vector3::Zero(), Vector3::Zero()};
        }
        return {p1, p2};
    }

    (void)CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX,
                            kDefaultRealTolerance);
    return {p1, p2};
}

std::pair<Vector3, Vector3> GetConvexesNearestPoints(
    std::span<const Vector3> convex1, std::span<const Vector3> convex2) {
    PolygonSupportFunction s1(convex1), s2(convex2);
    Vector3 dir = GetInitDir(GetCentroid(convex1), GetCentroid(convex2));

    Vector3 p1, p2;
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        if (!CalcCommonPoint(s1, s2, dir, p1, p2, kDefaultRealTolerance)) {
            return {Vector3::Zero(), Vector3::Zero()};
        }
        return {p1, p2};
    }

    (void)CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX,
                            kDefaultRealTolerance);
    return {p1, p2};
}

real GetConvexesDist(std::span<const Vector3> pts1,
                     std::span<const Vector3> pts2) {
    PolygonSupportFunction s1(pts1), s2(pts2);
    Vector3 dir = GetInitDir(GetCentroid(pts1), GetCentroid(pts2));
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 = CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX,
                                   kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetTetrahedronConvexDist(std::span<const Vector3> tet,
                              std::span<const Vector3> convex) {
    PolygonSupportFunction s1(tet), s2(convex);
    Vector3 dir = GetInitDir(GetCentroid(tet), GetCentroid(convex));
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 = CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX,
                                   kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetTetrahedronsDist(std::span<const Vector3, 4> tet1,
                         std::span<const Vector3, 4> tet2) {
    PolygonSupportFunction s1(tet1), s2(tet2);
    Vector3 dir = GetInitDir(GetCentroid(tet1), GetCentroid(tet2));
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 = CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX,
                                   kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetCapsuleOBBDist(Vector3 center, Vector3 axis, real half_height,
                       real radius, Vector3 obb_center,
                       std::span<const Vector3, 3> obb_axises,
                       Vector3 obb_half_extent) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    OBBSupportFunction s2(obb_center, obb_axises, obb_half_extent);
    if (IsIntersect(s1, s2, GetInitDir(center, obb_center),
                    kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 = CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX,
                                   kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetSphereConvexDist(Vector3 c, real r, std::span<const Vector3> convex) {
    SphereSupportFunction s1(c, r);
    PolygonSupportFunction s2(convex);
    if (IsIntersect(s1, s2, GetInitDir(c, GetCentroid(convex)),
                    kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 = CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX,
                                   kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetOBBTetrahedronDist(Vector3 center, std::span<const Vector3, 3> axises,
                           Vector3 half_extent, Vector3 p1, Vector3 p2,
                           Vector3 p3, Vector3 p4) {
    OBBSupportFunction s1(center, axises, half_extent);
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2(tv);
    if (IsIntersect(s1, s2, GetInitDir(center, GetCentroid(tv)),
                    kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 wp1, wp2;
    real dist2 = CalcClosestPoints(s1, s2, wp1, wp2, kREAL_MAX,
                                   kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetOBBConvexDist(Vector3 center, std::span<const Vector3, 3> axises,
                      Vector3 half_extent, std::span<const Vector3> convex) {
    OBBSupportFunction s1(center, axises, half_extent);
    PolygonSupportFunction s2(convex);
    if (IsIntersect(s1, s2, GetInitDir(center, GetCentroid(convex)),
                    kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 = CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX,
                                   kDefaultRealTolerance);
    return std::sqrt(dist2);
}

OBBSupportFunction::OBBSupportFunction(Vector3 center,
                                       std::span<const Vector3, 3> axises,
                                       Vector3 half_extent)
    : m_center{center}, m_half_extent{half_extent} {
    for (int i = 0; i < 3; ++i) m_axises[i] = axises[i];
}

Vector3 OBBSupportFunction::operator()(Vector3 dir) const {
    return GetOBBSupportPoint(m_center, m_axises, m_half_extent, dir);
}

PolygonSupportFunction::PolygonSupportFunction(std::span<const Vector3> pts)
    : m_pts{pts} {}

Vector3 PolygonSupportFunction::operator()(Vector3 dir) const {
    size_t index = GetSupportPoint(m_pts, dir);
    return m_pts[index];
}

SphereSupportFunction::SphereSupportFunction(Vector3 center, real radius)
    : m_center{center}, m_radius{radius} {}

Vector3 SphereSupportFunction::operator()(Vector3 dir) const {
    return GetSphereSupportPoint(m_center, m_radius, dir);
}

CapsuleSupportFunction::CapsuleSupportFunction(Vector3 center, Vector3 axis,
                                               real half_height, real radius)
    : m_center{center},
      m_axis{axis},
      m_half_height{half_height},
      m_radius{radius} {}

Vector3 PointSupportFunction::operator()(Vector3) const {
    return m_center;
}

Vector3 CapsuleSupportFunction::operator()(Vector3 dir) const {
    return GetCapsuleSupportPoint(m_center, m_axis, m_half_height, m_radius,
                                  dir);
}

PointSupportFunction::PointSupportFunction(Vector3 center) : m_center{center} {}

}  // namespace toy_physics
