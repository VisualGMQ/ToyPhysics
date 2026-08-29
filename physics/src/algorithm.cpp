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

bool IsAABBIntersect(Vector3 min1, Vector3 max1, Vector3 min2, Vector3 max2) {
    TOY_ENSURE_R_FALSE((min1.array() <= max1.array()).all() &&
                       (min2.array() <= max2.array()).all());

    return (min1.array() < max2.array()).all() &&
           (min2.array() < max1.array()).all();
}

// Moller, "A Fast Triangle-Triangle Intersection Test" (1997)
// ported from PhysX
// geomutils/src/intersection/GuIntersectionTriangleTriangle.cpp
namespace moller_tritri {

struct Interval {
    real min = 0;
    real max = 0;
    Vector3 minPoint = Vector3::Zero();
    Vector3 maxPoint = Vector3::Zero();

    void include(real d, const Vector3& p) {
        if (d < min) {
            min = d;
            minPoint = p;
        }
        if (d > max) {
            max = d;
            maxPoint = p;
        }
    }

    static bool overlapOrTouch(const Interval& a, const Interval& b) {
        return !(a.min > b.max || b.min > a.max);
    }
};

// interval of a triangle on the intersection line: interpolate the exact
// edge-plane crossing points, then project them onto dir
static Interval computeInterval(real distA, real distB, real distC,
                                const Vector3& a, const Vector3& b,
                                const Vector3& c, const Vector3& dir) {
    Interval i;

    const bool bA = distA > 0;
    const bool bB = distB > 0;
    const bool bC = distC > 0;
    distA = std::abs(distA);
    distB = std::abs(distB);
    distC = std::abs(distC);

    if (bA != bB) {
        const Vector3 p =
            (distA / (distA + distB)) * b + (distB / (distA + distB)) * a;
        i.include(dir.dot(p), p);
    }
    if (bA != bC) {
        const Vector3 p =
            (distA / (distA + distC)) * c + (distC / (distA + distC)) * a;
        i.include(dir.dot(p), p);
    }
    if (bB != bC) {
        const Vector3 p =
            (distB / (distB + distC)) * c + (distC / (distB + distC)) * b;
        i.include(dir.dot(p), p);
    }

    return i;
}

static real orient2d(const Vector3& a, const Vector3& b, const Vector3& c,
                     int x, int y) {
    return (a[y] - c[y]) * (b[x] - c[x]) - (a[x] - c[x]) * (b[y] - c[y]);
}

static bool pointInTriangle(const Vector3& a, const Vector3& b,
                            const Vector3& c, const Vector3& point, int x,
                            int y) {
    const real ab = orient2d(a, b, point, x, y);
    const real bc = orient2d(b, c, point, x, y);
    const real ca = orient2d(c, a, point, x, y);

    return (ab >= 0) == (bc >= 0) && (ab >= 0) == (ca >= 0);
}

static bool linesIntersect(const Vector3& startA, const Vector3& endA,
                           const Vector3& startB, const Vector3& endB, int x,
                           int y) {
    const real aaS = orient2d(startA, endA, startB, x, y);
    const real aaE = orient2d(startA, endA, endB, x, y);

    if ((aaS >= 0) == (aaE >= 0)) {
        return false;
    }

    const real bbS = orient2d(startB, endB, startA, x, y);
    const real bbE = orient2d(startB, endB, endA, x, y);

    return (bbS >= 0) != (bbE >= 0);
}

static void getProjectionIndices(Vector3 normal, int& x, int& y) {
    normal.x() = std::abs(normal.x());
    normal.y() = std::abs(normal.y());
    normal.z() = std::abs(normal.z());

    if (normal.x() >= normal.y() && normal.x() >= normal.z()) {
        x = 1;  // x is the dominant normal direction
        y = 2;
    } else if (normal.y() >= normal.x() && normal.y() >= normal.z()) {
        x = 2;  // y is the dominant normal direction
        y = 0;
    } else {
        x = 0;  // z is the dominant normal direction
        y = 1;
    }
}

static bool trianglesIntersectCoplanar(const Vector3& normal, const Vector3& a1,
                                       const Vector3& b1, const Vector3& c1,
                                       const Vector3& a2, const Vector3& b2,
                                       const Vector3& c2) {
    int x = 0;
    int y = 0;
    getProjectionIndices(normal, x, y);

    const real third = 1.0f / 3.0f;

    if (linesIntersect(a1, b1, a2, b2, x, y) ||
        linesIntersect(a1, b1, b2, c2, x, y) ||
        linesIntersect(a1, b1, c2, a2, x, y) ||
        linesIntersect(b1, c1, a2, b2, x, y) ||
        linesIntersect(b1, c1, b2, c2, x, y) ||
        linesIntersect(b1, c1, c2, a2, x, y) ||
        linesIntersect(c1, a1, a2, b2, x, y) ||
        linesIntersect(c1, a1, b2, c2, x, y) ||
        linesIntersect(c1, a1, c2, a2, x, y) ||
        pointInTriangle(a1, b1, c1, third * (a2 + b2 + c2), x, y) ||
        pointInTriangle(a2, b2, c2, third * (a1 + b1 + c1), x, y)) {
        return true;
    }
    return false;
}

static bool intersectTriangleTriangle(const Vector3& a1, const Vector3& b1,
                                      const Vector3& c1, const Vector3& a2,
                                      const Vector3& b2, const Vector3& c2,
                                      real tolerance, bool ignoreCoplanar) {
    // plane of triangle 1: n1 . (x - a1) = 0
    const Vector3 n1 = (b1 - a1).cross(c1 - a1);
    const real p1ToA = n1.dot(a2 - a1);
    const real p1ToB = n1.dot(b2 - a1);
    const real p1ToC = n1.dot(c2 - a1);

    if (std::abs(p1ToA) < tolerance && std::abs(p1ToB) < tolerance &&
        std::abs(p1ToC) < tolerance) {
        // coplanar triangles
        return ignoreCoplanar
                   ? false
                   : trianglesIntersectCoplanar(n1, a1, b1, c1, a2, b2, c2);
    }

    // all points of triangle 2 on the same side of triangle 1
    if ((p1ToA > 0) == (p1ToB > 0) && (p1ToA > 0) == (p1ToC > 0)) {
        return false;
    }

    // plane of triangle 2
    const Vector3 n2 = (b2 - a2).cross(c2 - a2);
    const real p2ToA = n2.dot(a1 - a2);
    const real p2ToB = n2.dot(b1 - a2);
    const real p2ToC = n2.dot(c1 - a2);

    if ((p2ToA > 0) == (p2ToB > 0) && (p2ToA > 0) == (p2ToC > 0)) {
        return false;
    }

    // direction of the intersection line
    Vector3 intersectionDirection = n1.cross(n2);
    const real l2 = intersectionDirection.squaredNorm();
    intersectionDirection /= std::sqrt(l2);

    const Interval i1 =
        computeInterval(p2ToA, p2ToB, p2ToC, a1, b1, c1, intersectionDirection);
    const Interval i2 =
        computeInterval(p1ToA, p1ToB, p1ToC, a2, b2, c2, intersectionDirection);

    return Interval::overlapOrTouch(i1, i2);
}

}  // namespace moller_tritri

bool IsTrianglesIntersect(std::span<const Vector3, 3> t1,
                          std::span<const Vector3, 3> t2, real tolerance,
                          bool ignoreCoplanar) {
    Vector3 n1 = (t1[1] - t1[0]).cross(t1[2] - t1[0]);
    Vector3 n2 = (t2[1] - t2[0]).cross(t2[2] - t2[0]);
    if (n1.squaredNorm() <= tolerance || n2.squaredNorm() <= tolerance) {
        return false;
    }

    return moller_tritri::intersectTriangleTriangle(
        t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], tolerance, ignoreCoplanar);
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

// Ericson, "Real-Time Collision Detection", ch 5.1.5:
// closest point on triangle to p by Voronoi region analysis
static Vector3 ClosestPtPointTriangle(Vector3 p, Vector3 a, Vector3 b,
                                      Vector3 c) {
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = p - a;
    real d1 = ab.dot(ap);
    real d2 = ac.dot(ap);
    // vertex region outside a
    if (d1 <= 0 && d2 <= 0) return a;

    Vector3 bp = p - b;
    real d3 = ab.dot(bp);
    real d4 = ac.dot(bp);
    // vertex region outside b
    if (d3 >= 0 && d4 <= d3) return b;

    // edge region of ab
    real vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        real v = d1 / (d1 - d3);
        return a + v * ab;
    }

    Vector3 cp = p - c;
    real d5 = ab.dot(cp);
    real d6 = ac.dot(cp);
    // vertex region outside c
    if (d6 >= 0 && d5 <= d6) return c;

    // edge region of ac
    real vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        real w = d2 / (d2 - d6);
        return a + w * ac;
    }

    // edge region of bc
    real va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    // face region
    real denom = 1 / (va + vb + vc);
    real v = vb * denom;
    real w = vc * denom;
    return a + ab * v + ac * w;
}

bool IsSphereTriangleIntersect(Vector3 c, real r, Vector3 p1, Vector3 p2,
                               Vector3 p3, real tolerance) {
    TOY_CHECK(r >= 0);

    Vector3 nearest;
    if ((p2 - p1).cross(p3 - p1).squaredNorm() <= tolerance) {
        // degenerate triangle: brute force over vertices and edges
        nearest = GetTriangleNearestPoint(c, p1, p2, p3, tolerance);
    } else {
        nearest = ClosestPtPointTriangle(c, p1, p2, p3);
    }
    return (nearest - c).squaredNorm() <= r * r;
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
    return IsIntersect(support1, support2, Vector3::Zero(),
                       kDefaultRealTolerance);
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

bool RaycastHit::IsInitialOverlap() const {
    return m_initial_overlap;
}

bool SweepHit::IsInitialOverlap() const {
    return m_initial_overlap;
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
        constexpr int epairs[6][2] = {
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
        constexpr int fpairs[4][3] = {
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

Vector3 GetCylinderNearestPoint(Vector3 p, Vector3 center, Vector3 axis,
                                real half_height, real radius) {
    TOY_CHECK(axis.isUnitary());
    TOY_CHECK(radius >= 0);

    real t = (p - center).dot(axis);
    real tcl = std::clamp<real>(t, -half_height, half_height);
    Vector3 q = center + axis * tcl;
    Vector3 radial = p - (center + axis * t);
    real rho2 = radial.squaredNorm();

    Vector3 u;
    if (rho2 <= kREAL_EPSILON) {
        u = axis.cross(Vector3::UnitX());
        if (u.squaredNorm() <= kREAL_EPSILON) {
            u = axis.cross(Vector3::UnitY());
        }
        u.normalize();
    } else {
        u = radial / std::sqrt(rho2);
    }
    real rho = std::sqrt(rho2);

    Vector3 side = q + u * radius;
    real side_dist2 = (rho - radius) * (rho - radius) + (t - tcl) * (t - tcl);

    if (rho <= radius) {
        real cap_sign = t >= 0 ? real(1) : real(-1);
        Vector3 cap = center + axis * (cap_sign * half_height) + u * rho;
        real cap_dist2 =
            (std::abs(t) - half_height) * (std::abs(t) - half_height);
        if (cap_dist2 < side_dist2) {
            return cap;
        }
    }
    return side;
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

real GetConvexSquaredDist(Vector3 p, std::span<const Vector3> convex) {
    PointSupportFunction support1(p);
    PolygonSupportFunction support2(convex);
    Vector3 out_p1, out_p2;
    return CalcClosestPoints(support1, support2, out_p1, out_p2, kREAL_MAX);
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
    return IsIntersect(s1, s2, Vector3::Zero(), kDefaultRealTolerance);
}

bool IsConvexesIntersect(std::span<const Vector3> pts1,
                         std::span<const Vector3> pts2) {
    PolygonSupportFunction s1(pts1), s2(pts2);
    return IsIntersect(s1, s2, Vector3::Zero(), kDefaultRealTolerance);
}

bool IsCapsuleOBBIntersect(Vector3 center, Vector3 axis, real half_height,
                           real radius, Vector3 obb_center,
                           std::span<const Vector3, 3> obb_axises,
                           Vector3 obb_half_extent) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    OBBSupportFunction s2(obb_center, obb_axises, obb_half_extent);
    return IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance);
}

bool IsCapsuleTetrahedronIntersect(Vector3 center, Vector3 axis,
                                   real half_height, real radius, Vector3 p1,
                                   Vector3 p2, Vector3 p3, Vector3 p4) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2(tv);
    return IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance);
}

bool IsCapsuleConvexIntersect(Vector3 center, Vector3 axis, real half_height,
                              real radius, std::span<const Vector3> convex) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    PolygonSupportFunction s2(convex);
    return IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance);
}

bool IsCylinderOBBIntersect(Vector3 center, Vector3 axis, real half_height,
                            real radius, Vector3 obb_center,
                            std::span<const Vector3, 3> obb_axises,
                            Vector3 obb_half_extent) {
    CylinderSupportFunction s1(center, axis, half_height, radius);
    OBBSupportFunction s2(obb_center, obb_axises, obb_half_extent);
    return IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance);
}

bool IsCylinderTetrahedronIntersect(Vector3 center, Vector3 axis,
                                    real half_height, real radius, Vector3 p1,
                                    Vector3 p2, Vector3 p3, Vector3 p4) {
    CylinderSupportFunction s1(center, axis, half_height, radius);
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2(tv);
    return IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance);
}

bool IsCylinderConvexIntersect(Vector3 center, Vector3 axis, real half_height,
                               real radius, std::span<const Vector3> convex) {
    CylinderSupportFunction s1(center, axis, half_height, radius);
    PolygonSupportFunction s2(convex);
    return IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance);
}

bool IsSphereConvexIntersect(Vector3 c, real r,
                             std::span<const Vector3> convex) {
    SphereSupportFunction s1(c, r);
    PolygonSupportFunction s2(convex);
    return IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance);
}

bool IsOBBTetrahedronIntersect(Vector3 center,
                               std::span<const Vector3, 3> axises,
                               Vector3 half_extent, Vector3 p1, Vector3 p2,
                               Vector3 p3, Vector3 p4) {
    OBBSupportFunction s1(center, axises, half_extent);
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2(tv);
    return IsIntersect(s1, s2, Vector3::Zero(), kDefaultRealTolerance);
}

bool IsOBBConvexIntersect(Vector3 center, std::span<const Vector3, 3> axises,
                          Vector3 half_extent,
                          std::span<const Vector3> convex) {
    OBBSupportFunction s1(center, axises, half_extent);
    PolygonSupportFunction s2(convex);
    return IsIntersect(s1, s2, Vector3::Zero(), kDefaultRealTolerance);
}

Ray::Ray(Vector3 o, Vector3 d) {
    m_origin = o;
    m_direction = d;
    m_inv_direction = Vector3(1 / d.x(), 1 / d.y(), 1 / d.z());
    m_sign[0] = (m_inv_direction.x() < 0);
    m_sign[1] = (m_inv_direction.y() < 0);
    m_sign[2] = (m_inv_direction.z() < 0);
}

Ray::Ray(const Ray& r) {
    m_origin = r.m_origin;
    m_direction = r.m_direction;
    m_inv_direction = r.m_inv_direction;
    m_sign[0] = r.m_sign[0];
    m_sign[1] = r.m_sign[1];
    m_sign[2] = r.m_sign[2];
}

bool IsLineAABBIntersect(const Ray& ray, real t0, real t1, Vector3 center,
                         Vector3 half_extent) {
    real tmin, tmax, tymin, tymax, tzmin, tzmax;
    Vector3 parameters[] = {center - half_extent, center + half_extent};

    tmin = (parameters[ray.m_sign[0]].x() - ray.m_origin.x()) *
           ray.m_inv_direction.x();
    tmax = (parameters[1 - ray.m_sign[0]].x() - ray.m_origin.x()) *
           ray.m_inv_direction.x();
    tymin = (parameters[ray.m_sign[1]].y() - ray.m_origin.y()) *
            ray.m_inv_direction.y();
    tymax = (parameters[1 - ray.m_sign[1]].y() - ray.m_origin.y()) *
            ray.m_inv_direction.y();
    if ((tmin > tymax) || (tymin > tmax)) {
        return false;
    }
    if (tymin > tmin) {
        tmin = tymin;
    }
    if (tymax < tmax) {
        tmax = tymax;
    }
    tzmin = (parameters[ray.m_sign[2]].z() - ray.m_origin.z()) *
            ray.m_inv_direction.z();
    tzmax = (parameters[1 - ray.m_sign[2]].z() - ray.m_origin.z()) *
            ray.m_inv_direction.z();
    if ((tmin > tzmax) || (tzmin > tmax)) {
        return false;
    }
    if (tzmin > tmin) {
        tmin = tzmin;
    }
    if (tzmax < tmax) {
        tmax = tzmax;
    }
    return ((tmin < t1) && (tmax > t0));
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
    if (penetration.norm() <= 0) {
        return false;
    }
    if (mtd) {
        *mtd = penetration;
    }
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
    if (penetration.norm() <= 0) {
        return false;
    }
    if (mtd) {
        *mtd = penetration;
    }
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
    if (penetration.norm() <= 0) {
        return false;
    }
    if (mtd) {
        *mtd = penetration;
    }
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

// Raycast

void setInitialOverlapHit(RaycastHit& hit, const Vector3& start) {
    hit.m_initial_overlap = true;
    hit.m_hit = start;
}

static RaycastHit makeInitialOverlapHit(const Vector3& start) {
    RaycastHit hit;
    setInitialOverlapHit(hit, start);
    return hit;
}

static Vector3 axisNormal(int axis, real dir_component) {
    Vector3 n = Vector3::Zero();
    n[axis] = dir_component > 0 ? -1 : 1;
    return n;
}

uint16_t RaycastSphere(Vector3 start, Vector3 dir, real len, Vector3 center,
                       real radius, std::span<RaycastHit> out_hits,
                       QueryFlags flags) {
    TOY_ENSURE_RV(!out_hits.empty(), 0);
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len > kREAL_EPSILON, 0);

    if ((start - center).squaredNorm() <= radius * radius) {
        out_hits[0] = makeInitialOverlapHit(start);
        return 1;
    }

    Vector3 d = dir * len;
    real a = d.dot(d);
    Vector3 q = start - center;
    real b = d.dot(q);
    real c = q.dot(q) - radius * radius;

    real delta = b * b - a * c;
    if (delta <= kREAL_EPSILON) {
        return 0;
    }

    real s = std::sqrt(delta);
    real t1 = (-b - s) / a;
    real t2 = (-b + s) / a;

    if (t1 < 0 || t1 > 1) {
        return 0;
    }

    RaycastHit enter;
    enter.m_hit = start + d * t1;
    if (flags.Has(QueryFlag::Distance)) {
        enter.m_dist = len * t1;
    }
    if (flags.Has(QueryFlag::Normal)) {
        enter.m_normal = (enter.m_hit - center).normalized();
    }
    out_hits[0] = enter;

    if (flags.Has(QueryFlag::AllSide) && out_hits.size() > 1 && t2 <= 1 &&
        std::abs(t2 - t1) > kDefaultRealTolerance) {
        RaycastHit exit;
        exit.m_hit = start + d * t2;
        if (flags.Has(QueryFlag::Distance)) {
            exit.m_dist = len * t2;
        }
        if (flags.Has(QueryFlag::Normal)) {
            exit.m_normal = (exit.m_hit - center).normalized();
        }
        out_hits[1] = exit;
        return 2;
    }
    return 1;
}

/**
 * Amy Williams et al., "An Efficient and Robust Ray-Box Intersection
 * Algorithm", Journal of graphics tools 10(1):49-54, 2005
 *
 * @return 1 on hit, 0 on miss, -1 on initial overlap
 */
int slabIntersect(const Vector3& local_start, const Vector3& local_dir,
                  const Vector3& half_extent, real& t_enter, real& t_exit,
                  int& enter_axis, int& exit_axis) {
    Ray ray{local_start, local_dir};
    Vector3 parameters[] = {-half_extent, half_extent};

    t_enter = -kREAL_MAX;
    t_exit = kREAL_MAX;
    enter_axis = 0;
    exit_axis = 0;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(ray.m_direction[i]) <= kREAL_EPSILON) {
            if (ray.m_origin[i] < parameters[0][i] ||
                ray.m_origin[i] > parameters[1][i]) {
                return 0;
            }
            continue;
        }
        real t1 = (parameters[ray.m_sign[i]][i] - ray.m_origin[i]) *
                  ray.m_inv_direction[i];
        real t2 = (parameters[1 - ray.m_sign[i]][i] - ray.m_origin[i]) *
                  ray.m_inv_direction[i];
        if (t1 > t_enter) {
            t_enter = t1;
            enter_axis = i;
        }
        if (t2 < t_exit) {
            t_exit = t2;
            exit_axis = i;
        }
        if (t_enter > t_exit) {
            return 0;
        }
    }
    if (t_exit < 0) {
        return 0;
    }
    if (t_enter <= 0) {
        return -1;
    }
    return 1;
}

uint16_t RaycastAABB(Vector3 start, Vector3 dir, real len, Vector3 center,
                     Vector3 half_extent, std::span<RaycastHit> out_hits,
                     QueryFlags flags) {
    TOY_ENSURE_RV(!out_hits.empty(), 0);
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len > kREAL_EPSILON, 0);

    Vector3 d = dir * len;
    real t_enter, t_exit;
    int enter_axis, exit_axis;
    int status = slabIntersect(start - center, d, half_extent, t_enter, t_exit,
                               enter_axis, exit_axis);
    if (status == 0) {
        return 0;
    }
    if (status == -1) {
        out_hits[0] = makeInitialOverlapHit(start);
        return 1;
    }
    if (t_enter >= 1) {
        return 0;
    }

    uint16_t hit_count = 0;
    RaycastHit enter;
    enter.m_hit = start + d * t_enter;
    if (flags.Has(QueryFlag::Distance)) {
        enter.m_dist = t_enter * len;
    }
    if (flags.Has(QueryFlag::Normal)) {
        enter.m_normal = axisNormal(enter_axis, d[enter_axis]);
    }
    out_hits[hit_count++] = enter;

    if (flags.Has(QueryFlag::AllSide) && hit_count < out_hits.size() &&
        t_exit <= 1) {
        if (t_exit - t_enter >
            kDefaultRealTolerance * std::max(real(1), t_enter)) {
            RaycastHit exit;
            exit.m_hit = start + d * t_exit;
            if (flags.Has(QueryFlag::Distance)) {
                exit.m_dist = t_exit * len;
            }
            if (flags.Has(QueryFlag::Normal)) {
                exit.m_normal = -axisNormal(exit_axis, d[exit_axis]);
            }
            out_hits[hit_count++] = exit;
        }
    }
    return hit_count;
}

bool RaycastPlane(Vector3 start, Vector3 dir, real len, Vector3 p,
                  Vector3 normal, std::span<RaycastHit, 1> out_hits,
                  QueryFlags flags) {
    TOY_ENSURE_R_FALSE(normal.isUnitary());
    TOY_ENSURE_R_FALSE(dir.isUnitary());
    TOY_ENSURE_R_FALSE(len > kREAL_EPSILON);

    Vector3 d = dir * len;
    real denom = normal.dot(d);
    if (std::abs(denom) <= kREAL_EPSILON) {
        return false;  // ray parallel to the plane
    }

    real t = normal.dot(p - start) / denom;
    if (t < 0 || t > 1) {
        return false;  // plane behind the ray origin or beyond len
    }

    RaycastHit hit;
    hit.m_hit = start + d * t;
    if (flags.Has(QueryFlag::Distance)) {
        hit.m_dist = t * len;
    }
    if (flags.Has(QueryFlag::Normal)) {
        hit.m_normal = normal;
    }
    out_hits[0] = hit;
    return true;
}

uint16_t RaycastTriangleCullBackFace(Vector3 start, Vector3 dir, real len,
                                     std::span<const Vector3, 3> vertices,
                                     RaycastHit* out_hit, QueryFlags flags) {
    TOY_ENSURE_RV(out_hit, 0);
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len > kREAL_EPSILON, 0);

    Vector3 edge1 = vertices[1] - vertices[0];
    Vector3 edge2 = vertices[2] - vertices[0];

    Vector3 d = dir * len;
    Vector3 pvec = d.cross(edge2);
    real det = edge1.dot(pvec);

    if (det < kREAL_EPSILON) {
        return 0;
    }

    Vector3 tvec = start - vertices[0];
    real u = tvec.dot(pvec);
    if (u < 0.0 || u > det) {
        return 0;
    }

    Vector3 qvec = tvec.cross(edge1);
    real v = d.dot(qvec);
    if (v < 0.0 || u + v > det) {
        return 0;
    }

    real t = edge2.dot(qvec);
    real inv_det = 1.0 / det;
    t *= inv_det;
    if (t < 0 || t > 1) {
        return 0;
    }

    RaycastHit hit;
    hit.m_hit = start + d * t;
    if (flags.Has(QueryFlag::Normal)) {
        hit.m_normal = edge1.cross(edge2).normalized();
    }
    if (flags.Has(QueryFlag::Distance)) {
        hit.m_dist = t * len;
    }
    if (flags.Has(QueryFlag::UV)) {
        u *= inv_det;
        v *= inv_det;
        hit.m_u = u;
        hit.m_v = v;
    }
    *out_hit = hit;

    return 1;
}

uint16_t RaycastTriangle(Vector3 start, Vector3 dir, real len,
                         std::span<const Vector3, 3> vertices,
                         RaycastHit* out_hit, QueryFlags flags) {
    TOY_ENSURE_RV(out_hit, 0);
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len > kREAL_EPSILON, 0);

    Vector3 d = dir * len;

    Vector3 edge1 = vertices[1] - vertices[0];
    Vector3 edge2 = vertices[2] - vertices[0];

    Vector3 pvec = d.cross(edge2);

    real det = edge1.dot(pvec);

    if (std::abs(det) < kREAL_EPSILON) {
        return 0;
    }
    real inv_det = 1.0 / det;

    Vector3 tvec = start - vertices[0];
    real u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0) {
        return 0;
    }

    Vector3 qvec = tvec.cross(edge1);
    real v = d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0) {
        return 0;
    }

    real t = edge2.dot(qvec) * inv_det;
    if (t < 0 || t > 1) {
        return 0;
    }

    RaycastHit hit;
    hit.m_hit = start + d * t;
    if (flags.Has(QueryFlag::Normal)) {
        hit.m_normal = edge1.cross(edge2).normalized();
    }
    if (flags.Has(QueryFlag::Distance)) {
        hit.m_dist = t * len;
    }
    if (flags.Has(QueryFlag::UV)) {
        hit.m_u = u;
        hit.m_v = v;
    }
    *out_hit = hit;
    return 1;
}

uint16_t RaycastOBB(Vector3 start, Vector3 dir, real len, Vector3 center,
                    std::span<const Vector3, 3> axis, Vector3 half_extent,
                    std::span<RaycastHit> out_hits, QueryFlags flags) {
    TOY_ENSURE_RV(!out_hits.empty(), 0);
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len > kREAL_EPSILON, 0);

    Matrix33 rotation;
    rotation.col(0) = axis[0];
    rotation.col(1) = axis[1];
    rotation.col(2) = axis[2];

    Vector3 local_start = rotation.transpose() * (start - center);
    Vector3 local_dir = rotation.transpose() * dir;

    uint16_t hit_count =
        RaycastAABB(local_start, local_dir, len, Vector3::Zero(), half_extent,
                    out_hits, flags);
    for (uint16_t i = 0; i < hit_count; ++i) {
        out_hits[i].m_hit = center + rotation * out_hits[i].m_hit;
        if (!out_hits[i].m_initial_overlap && flags.Has(QueryFlag::Normal)) {
            out_hits[i].m_normal = rotation * out_hits[i].m_normal;
        }
    }
    return hit_count;
}

uint16_t RaycastCapsule(Vector3 start, Vector3 dir, real len, Vector3 center,
                        Vector3 axis, real half_height, real radius,
                        std::span<RaycastHit> out_hits, QueryFlags flags) {
    TOY_ENSURE_RV(!out_hits.empty(), 0);
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len > kREAL_EPSILON, 0);

    Vector3 d = dir * len;

    Vector3 p = center - axis * half_height;
    Vector3 q = center + axis * half_height;

    if (GetPtToSegmentSquaredDist(start, p, axis, 2 * half_height) <=
        radius * radius) {
        out_hits[0] = makeInitialOverlapHit(start);
        return 1;
    }

    // Ericson, "Real-Time Collision Detection", ch 5.3.7:
    // line intersect with inf capsule
    Vector3 w = q - p;
    real ww = w.squaredNorm();
    Vector3 m = start - p;
    real md = m.dot(w);
    real nd = d.dot(w);
    real nn = d.squaredNorm();
    real mn = m.dot(d);
    real k = m.squaredNorm() - radius * radius;

    real roots[6];
    int count = 0;

    // infinite cylinder: roots valid when the axis projection is inside
    // the endcaps
    real a = ww * nn - nd * nd;
    if (std::abs(a) > kREAL_EPSILON) {
        real b = ww * mn - nd * md;
        real c = ww * k - md * md;
        real discr = b * b - a * c;
        if (discr >= 0) {
            real s = std::sqrt(discr);
            real t1 = (-b - s) / a;
            real t2 = (-b + s) / a;
            if ((md + t1 * nd) / ww >= 0 && (md + t1 * nd) / ww <= 1) {
                roots[count++] = t1;
            }
            if ((md + t2 * nd) / ww >= 0 && (md + t2 * nd) / ww <= 1) {
                roots[count++] = t2;
            }
        }
    }

    // endcap spheres: roots valid when the axis projection lies beyond
    // the corresponding endcap
    {
        real discr = mn * mn - nn * k;
        if (discr >= 0) {
            real s = std::sqrt(discr);
            real t1 = (-mn - s) / nn;
            real t2 = (-mn + s) / nn;
            if ((md + t1 * nd) / ww <= 0) {
                roots[count++] = t1;
            }
            if ((md + t2 * nd) / ww <= 0) {
                roots[count++] = t2;
            }
        }
    }
    {
        Vector3 mq = start - q;
        real mnq = mq.dot(d);
        real kq = mq.squaredNorm() - radius * radius;
        real discr = mnq * mnq - nn * kq;
        if (discr >= 0) {
            real s = std::sqrt(discr);
            real t1 = (-mnq - s) / nn;
            real t2 = (-mnq + s) / nn;
            if ((md + t1 * nd) / ww >= 1) {
                roots[count++] = t1;
            }
            if ((md + t2 * nd) / ww >= 1) {
                roots[count++] = t2;
            }
        }
    }

    real t_enter = kREAL_MAX;
    real t_exit = -kREAL_MAX;
    for (int i = 0; i < count; ++i) {
        if (roots[i] < 0 || roots[i] > 1) {
            continue;
        }
        t_enter = std::min(t_enter, roots[i]);
        t_exit = std::max(t_exit, roots[i]);
    }
    if (t_enter == kREAL_MAX) {
        return 0;
    }

    uint16_t hit_count = 0;
    RaycastHit enter;
    enter.m_hit = start + d * t_enter;
    if (flags.Has(QueryFlag::Distance)) {
        enter.m_dist = t_enter * len;
    }
    if (flags.Has(QueryFlag::Normal)) {
        enter.m_normal =
            (enter.m_hit - GetSegmentNearestPoint(enter.m_hit, p, q)) / radius;
    }
    out_hits[hit_count++] = enter;

    if (flags.Has(QueryFlag::AllSide) && hit_count < out_hits.size() &&
        t_exit - t_enter > kDefaultRealTolerance * std::max(real(1), t_enter)) {
        RaycastHit exit;
        exit.m_hit = start + d * t_exit;
        if (flags.Has(QueryFlag::Distance)) {
            exit.m_dist = t_exit * len;
        }
        if (flags.Has(QueryFlag::Normal)) {
            exit.m_normal =
                (exit.m_hit - GetSegmentNearestPoint(exit.m_hit, p, q)) /
                radius;
        }
        out_hits[hit_count++] = exit;
    }
    return hit_count;
}

uint16_t RaycastCylinder(Vector3 start, Vector3 dir, real len, Vector3 center,
                         Vector3 axis, real half_height, real radius,
                         std::span<RaycastHit> out_hits, QueryFlags flags) {
    TOY_ENSURE_RV(!out_hits.empty(), 0);
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len > kREAL_EPSILON, 0);

    Vector3 d = dir * len;

    Vector3 p0 = center - axis * half_height;
    Vector3 m = start - p0;
    real sc = (start - center).dot(axis);
    Vector3 radial = m - axis * m.dot(axis);
    if (std::abs(sc) <= half_height &&
        radial.squaredNorm() <= radius * radius) {
        out_hits[0] = makeInitialOverlapHit(start);
        return 1;
    }

    real dn = d.dot(axis);
    real mn = m.dot(axis);

    real roots[4];
    Vector3 normals[4];
    int count = 0;

    // side: roots valid when the axis coordinate is between the endcaps
    real a = d.squaredNorm() - dn * dn;
    if (std::abs(a) > kREAL_EPSILON) {
        real b = d.dot(m) - dn * mn;
        real c = m.squaredNorm() - mn * mn - radius * radius;
        real discr = b * b - a * c;
        if (discr >= 0) {
            real s = std::sqrt(discr);
            real ts[2] = {(-b - s) / a, (-b + s) / a};
            for (real t : ts) {
                if (t < 0 || t > 1) continue;
                real s_axis = mn + t * dn;
                if (s_axis < 0 || s_axis > 2 * half_height) continue;
                roots[count] = t;
                normals[count] =
                    (start + d * t - (p0 + axis * s_axis)) / radius;
                ++count;
            }
        }
    }

    // endcap disks: each plane gives at most one root, valid inside the disk
    if (std::abs(dn) > kREAL_EPSILON) {
        real signs[2] = {1, -1};
        for (real sign : signs) {
            real t = (sign * half_height - sc) / dn;
            if (t < 0 || t > 1) continue;
            Vector3 hit = start + d * t;
            Vector3 axis_pt = center + axis * (sign * half_height);
            if ((hit - axis_pt).squaredNorm() > radius * radius) continue;
            roots[count] = t;
            normals[count] = axis * sign;
            ++count;
        }
    }

    real t_enter = kREAL_MAX;
    real t_exit = -kREAL_MAX;
    Vector3 enter_normal = Vector3::Zero();
    Vector3 exit_normal = Vector3::Zero();
    for (int i = 0; i < count; ++i) {
        if (roots[i] < t_enter) {
            t_enter = roots[i];
            enter_normal = normals[i];
        }
        if (roots[i] > t_exit) {
            t_exit = roots[i];
            exit_normal = normals[i];
        }
    }
    if (t_enter == kREAL_MAX) {
        return 0;
    }

    uint16_t hit_count = 0;
    RaycastHit enter;
    enter.m_hit = start + d * t_enter;
    if (flags.Has(QueryFlag::Distance)) {
        enter.m_dist = t_enter * len;
    }
    if (flags.Has(QueryFlag::Normal)) {
        enter.m_normal = enter_normal;
    }
    out_hits[hit_count++] = enter;

    if (flags.Has(QueryFlag::AllSide) && hit_count < out_hits.size() &&
        t_exit - t_enter > kDefaultRealTolerance * std::max(real(1), t_enter)) {
        RaycastHit exit;
        exit.m_hit = start + d * t_exit;
        if (flags.Has(QueryFlag::Distance)) {
            exit.m_dist = t_exit * len;
        }
        if (flags.Has(QueryFlag::Normal)) {
            exit.m_normal = exit_normal;
        }
        out_hits[hit_count++] = exit;
    }
    return hit_count;
}

uint16_t RaycastConvex(Vector3 start, Vector3 dir, real len,
                       std::span<const Vector3> vertices,
                       std::span<const Vector3> normals,
                       std::span<RaycastHit> out_hits, QueryFlags flags) {
    TOY_ENSURE_RV(!out_hits.empty(), 0);
    TOY_ENSURE_RV(dir.isUnitary(), 0);
    TOY_ENSURE_RV(len > kREAL_EPSILON, 0);
    TOY_ENSURE_RV(!vertices.empty(), 0);
    TOY_ENSURE_RV(!normals.empty(), 0);

    Vector3 d = dir * len;

    real t_enter = -kREAL_MAX;
    real t_exit = kREAL_MAX;
    for (const Vector3& n : normals) {
        real plane_d = -kREAL_MAX;
        for (const Vector3& v : vertices) {
            plane_d = std::max(plane_d, n.dot(v));
        }

        real dn = n.dot(d);
        real numer = plane_d - n.dot(start);
        if (std::abs(dn) <= kREAL_EPSILON) {
            if (numer < 0) {
                return 0;  // parallel to the face and outside its half-space
            }
            continue;
        }
        real t = numer / dn;
        if (dn < 0) {
            if (t > t_enter) {
                t_enter = t;
                RaycastHit hit;
                hit.m_hit = start + d * t;
                if (flags.Has(QueryFlag::Distance)) {
                    hit.m_dist = t * len;
                }
                if (flags.Has(QueryFlag::Normal)) {
                    hit.m_normal = n;
                }
                out_hits[0] = hit;
            }
        } else if (t < t_exit) {
            t_exit = t;
            if (flags.Has(QueryFlag::AllSide) && out_hits.size() > 1) {
                RaycastHit hit;
                hit.m_hit = start + d * t;
                if (flags.Has(QueryFlag::Distance)) {
                    hit.m_dist = t * len;
                }
                if (flags.Has(QueryFlag::Normal)) {
                    hit.m_normal = n;
                }
                out_hits[1] = hit;
            }
        }
        if (t_enter > t_exit) {
            return 0;
        }
    }

    if (t_exit < 0) {
        return 0;
    }
    if (t_enter <= 0) {
        out_hits[0] = makeInitialOverlapHit(start);
        return 1;
    }
    if (t_enter >= 1) {
        return 0;  // convex beyond len
    }
    if (flags.Has(QueryFlag::AllSide) && out_hits.size() > 1 && t_exit <= 1 &&
        t_exit - t_enter > kDefaultRealTolerance * std::max<real>(1, t_enter)) {
        return 2;
    }
    return 1;
}

bool SweepSphereSphere(Vector3 s1, real r1, Vector3 s2, real r2, Vector3 dir,
                       real len, SweepHit* out_hit, QueryFlags flags) {
    TOY_ENSURE_RV(out_hit, false);

    RaycastHit hit;
    if (RaycastSphere(s1, dir, len, s2, r1 + r2, std::span{&hit, 1}, flags) ==
        0) {
        return false;
    }
    out_hit->m_normal = hit.m_normal;
    out_hit->m_dist = hit.m_dist;
    out_hit->m_hit = hit.m_hit;
    out_hit->m_initial_overlap = hit.m_initial_overlap;
    return true;
}

bool SweepSphereCapsule(Vector3 s1, real r1, Vector3 center, Vector3 axis,
                        real half_height, real radius, Vector3 dir, real len,
                        SweepHit* out_hit, QueryFlags flags) {
    TOY_ENSURE_RV(out_hit, false);
    TOY_ENSURE_RV(axis.isUnitary(), false);

    RaycastHit hit;
    if (RaycastCapsule(s1, dir, len, center, axis, half_height, r1 + radius,
                       std::span{&hit, 1}, flags) == 0) {
        return false;
    }
    out_hit->m_normal = hit.m_normal;
    out_hit->m_dist = hit.m_dist;
    out_hit->m_hit = hit.m_hit;
    out_hit->m_initial_overlap = hit.m_initial_overlap;
    return true;
}

std::pair<Vector3, Vector3> GetTetrahedronsNearestPoints(
    std::span<const Vector3, 4> tet1, std::span<const Vector3, 4> tet2) {
    PolygonSupportFunction s1(tet1), s2(tet2);
    auto dir = Vector3::Zero();

    Vector3 p1, p2;
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        if (!CalcCommonPoint(s1, s2, dir, p1, p2, kDefaultRealTolerance)) {
            return {Vector3::Zero(), Vector3::Zero()};
        }
        return {p1, p2};
    }

    (void)CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX, kDefaultRealTolerance);
    return {p1, p2};
}

std::pair<Vector3, Vector3> GetConvexesNearestPoints(
    std::span<const Vector3> convex1, std::span<const Vector3> convex2) {
    PolygonSupportFunction s1(convex1), s2(convex2);
    Vector3 dir = Vector3::Zero();

    Vector3 p1, p2;
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        if (!CalcCommonPoint(s1, s2, dir, p1, p2, kDefaultRealTolerance)) {
            return {Vector3::Zero(), Vector3::Zero()};
        }
        return {p1, p2};
    }

    (void)CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX, kDefaultRealTolerance);
    return {p1, p2};
}

real GetConvexesDist(std::span<const Vector3> pts1,
                     std::span<const Vector3> pts2) {
    PolygonSupportFunction s1(pts1), s2(pts2);
    Vector3 dir = Vector3::Zero();
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 =
        CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX, kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetTetrahedronConvexDist(std::span<const Vector3> tet,
                              std::span<const Vector3> convex) {
    PolygonSupportFunction s1(tet), s2(convex);
    Vector3 dir = Vector3::Zero();
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 =
        CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX, kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetTetrahedronsDist(std::span<const Vector3, 4> tet1,
                         std::span<const Vector3, 4> tet2) {
    PolygonSupportFunction s1(tet1), s2(tet2);
    Vector3 dir = Vector3::Zero();
    if (IsIntersect(s1, s2, dir, kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 =
        CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX, kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetCapsuleOBBDist(Vector3 center, Vector3 axis, real half_height,
                       real radius, Vector3 obb_center,
                       std::span<const Vector3, 3> obb_axises,
                       Vector3 obb_half_extent) {
    CapsuleSupportFunction s1(center, axis, half_height, radius);
    OBBSupportFunction s2(obb_center, obb_axises, obb_half_extent);
    if (IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 =
        CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX, kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetSphereConvexDist(Vector3 c, real r, std::span<const Vector3> convex) {
    SphereSupportFunction s1(c, r);
    PolygonSupportFunction s2(convex);
    if (IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 =
        CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX, kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetOBBTetrahedronDist(Vector3 center, std::span<const Vector3, 3> axises,
                           Vector3 half_extent, Vector3 p1, Vector3 p2,
                           Vector3 p3, Vector3 p4) {
    OBBSupportFunction s1(center, axises, half_extent);
    std::array<Vector3, 4> tv = {p1, p2, p3, p4};
    PolygonSupportFunction s2(tv);
    if (IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 wp1, wp2;
    real dist2 =
        CalcClosestPoints(s1, s2, wp1, wp2, kREAL_MAX, kDefaultRealTolerance);
    return std::sqrt(dist2);
}

real GetOBBConvexDist(Vector3 center, std::span<const Vector3, 3> axises,
                      Vector3 half_extent, std::span<const Vector3> convex) {
    OBBSupportFunction s1(center, axises, half_extent);
    PolygonSupportFunction s2(convex);
    if (IsIntersect(s1, s2, Vector3::UnitX(), kDefaultRealTolerance)) {
        return 0;
    }
    Vector3 p1, p2;
    real dist2 =
        CalcClosestPoints(s1, s2, p1, p2, kREAL_MAX, kDefaultRealTolerance);
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
    return GetSphereSupportPoint(m_center, m_radius, dir.normalized());
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
                                  dir.normalized());
}

PointSupportFunction::PointSupportFunction(Vector3 center) : m_center{center} {}

}  // namespace toy_physics
