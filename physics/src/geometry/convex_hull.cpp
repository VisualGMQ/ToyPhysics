#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/common/check.hpp"
#include "Eigen/Eigenvalues"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace toy_physics {

namespace {

struct HullPlane {
    Vector3 m_normal;
    real m_offset;
};

struct Point2 {
    real m_x;
    real m_y;
    int m_index;
};

real cross2(const Point2& o, const Point2& a, const Point2& b) {
    return (a.m_x - o.m_x) * (b.m_y - o.m_y) -
           (a.m_y - o.m_y) * (b.m_x - o.m_x);
}

void computeAABBFallback(std::span<const Vector3> vertices, real& out_volume,
                         Vector3& out_center, Vector3& out_inertia,
                         Matrix33& out_axes) {
    out_volume = 0;
    out_center = Vector3::Zero();
    out_inertia = Vector3::Zero();
    out_axes = Matrix33::Identity();

    if (vertices.empty()) {
        return;
    }

    Vector3 min = vertices[0];
    Vector3 max = vertices[0];
    for (const Vector3& v : vertices) {
        min = min.cwiseMin(v);
        max = max.cwiseMax(v);
    }

    const Vector3 size = max - min;
    out_volume = size.x() * size.y() * size.z();
    out_center = (min + max) * real(0.5);

    out_inertia = Vector3{
        out_volume / real(12) * (size.y() * size.y() + size.z() * size.z()),
        out_volume / real(12) * (size.x() * size.x() + size.z() * size.z()),
        out_volume / real(12) * (size.x() * size.x() + size.y() * size.y()),
    };
}

bool isDuplicatePlane(const std::vector<HullPlane>& planes,
                      const HullPlane& candidate, real tolerance) {
    for (const HullPlane& plane : planes) {
        if (plane.m_normal.dot(candidate.m_normal) > real(1) - real(1e-4) &&
            std::abs(plane.m_offset - candidate.m_offset) <= tolerance) {
            return true;
        }
    }
    return false;
}

std::vector<Point2> convexHull2D(std::vector<Point2> points, real tolerance) {
    std::sort(points.begin(), points.end(),
              [](const Point2& a, const Point2& b) {
                  if (a.m_x != b.m_x) {
                      return a.m_x < b.m_x;
                  }
                  return a.m_y < b.m_y;
              });

    std::vector<Point2> unique;
    unique.reserve(points.size());
    for (const Point2& p : points) {
        if (unique.empty()) {
            unique.push_back(p);
            continue;
        }
        const Point2& q = unique.back();
        const real dx = p.m_x - q.m_x;
        const real dy = p.m_y - q.m_y;
        if (dx * dx + dy * dy > tolerance * tolerance) {
            unique.push_back(p);
        }
    }

    if (unique.size() < 3) {
        return {};
    }

    std::vector<Point2> hull;
    hull.reserve(unique.size() * 2);

    for (const Point2& p : unique) {
        while (hull.size() >= 2 &&
               cross2(hull[hull.size() - 2], hull.back(), p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    const size_t lower_size = hull.size();
    for (size_t i = unique.size() - 1; i-- > 0;) {
        const Point2& p = unique[i];
        while (hull.size() > lower_size &&
               cross2(hull[hull.size() - 2], hull.back(), p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    if (!hull.empty()) {
        hull.pop_back();
    }

    return hull;
}

}  // namespace

ConvexData::ConvexData(std::span<const Vector3> vertices)
    : m_vertices{vertices.begin(), vertices.end()} {
    computeMassProperty();
}

std::span<const Vector3> ConvexData::GetVertices() const {
    return m_vertices;
}

real ConvexData::GetVolume() const {
    return m_volume;
}

const Vector3& ConvexData::GetCenterOfMass() const {
    return m_center_of_mass;
}

const Vector3& ConvexData::GetUnitInertia() const {
    return m_unit_inertia;
}

const Matrix33& ConvexData::GetPrincipalAxes() const {
    return m_principal_axes;
}

// TODO: use 3rd convex hull constructor library!
void ConvexData::computeMassProperty() {
    // enumerate faces to construct tetrahedron.
    // This is O(n^4), acceptable for small convex hulls; replace with a proper
    // quickhull implementation if the vertex count grows.
    if (m_vertices.size() < 4) {
        computeAABBFallback(m_vertices, m_volume, m_center_of_mass,
                            m_unit_inertia, m_principal_axes);
        return;
    }

    Vector3 min = m_vertices[0];
    Vector3 max = m_vertices[0];
    for (const Vector3& v : m_vertices) {
        min = min.cwiseMin(v);
        max = max.cwiseMax(v);
    }

    const real scale = std::max((max - min).norm(), real(1));
    const real tolerance = scale * real(1e-5);

    std::vector<HullPlane> planes;

    for (size_t i = 0; i < m_vertices.size(); ++i) {
        for (size_t j = i + 1; j < m_vertices.size(); ++j) {
            for (size_t k = j + 1; k < m_vertices.size(); ++k) {
                const Vector3& a = m_vertices[i];
                const Vector3& b = m_vertices[j];
                const Vector3& c = m_vertices[k];

                Vector3 normal = (b - a).cross(c - a);
                const real len2 = normal.squaredNorm();
                if (len2 <= tolerance * tolerance) {
                    continue;
                }

                normal /= std::sqrt(len2);
                const real d = normal.dot(a);

                real max_side = -std::numeric_limits<real>::max();
                real min_side = std::numeric_limits<real>::max();
                for (const Vector3& v : m_vertices) {
                    const real s = normal.dot(v) - d;
                    max_side = std::max(max_side, s);
                    min_side = std::min(min_side, s);
                }

                HullPlane plane;
                if (max_side <= tolerance) {
                    plane.m_normal = normal;
                    plane.m_offset = d;
                } else if (min_side >= -tolerance) {
                    plane.m_normal = -normal;
                    plane.m_offset = -d;
                } else {
                    continue;
                }

                if (!isDuplicatePlane(planes, plane, tolerance * real(4))) {
                    planes.push_back(plane);
                }
            }
        }
    }

    if (planes.empty()) {
        computeAABBFallback(m_vertices, m_volume, m_center_of_mass,
                            m_unit_inertia, m_principal_axes);
        return;
    }

    real total_volume = 0;
    Vector3 first_moment = Vector3::Zero();
    Matrix33 second_moment = Matrix33::Zero();

    for (const HullPlane& plane : planes) {
        Vector3 u = plane.m_normal.unitOrthogonal();
        Vector3 v = plane.m_normal.cross(u).normalized();

        std::vector<int> indices;
        for (size_t i = 0; i < m_vertices.size(); ++i) {
            if (std::abs(plane.m_normal.dot(m_vertices[i]) - plane.m_offset) <=
                tolerance * real(4)) {
                indices.push_back(static_cast<int>(i));
            }
        }

        if (indices.size() < 3) {
            continue;
        }

        std::vector<Point2> projected;
        projected.reserve(indices.size());
        for (int index : indices) {
            const Vector3& p = m_vertices[index];
            projected.push_back({p.dot(u), p.dot(v), index});
        }

        std::vector<Point2> face2d =
            convexHull2D(std::move(projected), tolerance);
        if (face2d.size() < 3) {
            continue;
        }

        std::vector<Vector3> face;
        face.reserve(face2d.size());
        for (const Point2& p : face2d) {
            face.push_back(m_vertices[p.m_index]);
        }

        // Make the polygon normal point outward.
        if ((face[1] - face[0]).cross(face[2] - face[0]).dot(plane.m_normal) <
            0) {
            std::reverse(face.begin(), face.end());
        }

        const Vector3& a = face[0];
        for (size_t i = 1; i + 1 < face.size(); ++i) {
            const Vector3& b = face[i];
            const Vector3& c = face[i + 1];

            const real volume = a.dot(b.cross(c)) / real(6);
            total_volume += volume;

            const Vector3 sum = a + b + c;
            first_moment += sum * (volume / real(4));

            Matrix33 outer =
                a * a.transpose() + b * b.transpose() + c * c.transpose();
            outer += sum * sum.transpose();

            second_moment += outer * (volume / real(20));
        }
    }

    if (std::abs(total_volume) <= tolerance * tolerance * tolerance) {
        computeAABBFallback(m_vertices, m_volume, m_center_of_mass,
                            m_unit_inertia, m_principal_axes);
        return;
    }

    // If the face winding was globally inverted, the signed integrals are
    // negative. Flip them so volume and inertia stay positive.
    if (total_volume < 0) {
        total_volume = -total_volume;
        first_moment = -first_moment;
        second_moment = -second_moment;
    }

    const Vector3 center_of_mass = first_moment / total_volume;

    // second_moment is the integral of x*x^T over the volume (density = 1).
    // Move it to the center of mass.
    Matrix33 centered_moment = second_moment - total_volume * center_of_mass *
                                                   center_of_mass.transpose();

    // Inertia about the center of mass for density = 1:
    // I = trace(C) * identity - C
    const Matrix33 inertia =
        Matrix33::Identity() * centered_moment.trace() - centered_moment;

    Eigen::SelfAdjointEigenSolver<Matrix33> solver(inertia);
    if (solver.info() != Eigen::Success) {
        computeAABBFallback(m_vertices, m_volume, m_center_of_mass,
                            m_unit_inertia, m_principal_axes);
        return;
    }

    Vector3 values = solver.eigenvalues();
    Matrix33 axes = solver.eigenvectors();

    // Sort from largest to smallest, matching the principal axis columns.
    Vector3 sorted_values;
    Matrix33 sorted_axes;

    sorted_values[0] = values[2];
    sorted_values[1] = values[1];
    sorted_values[2] = values[0];

    sorted_axes.col(0) = axes.col(2).normalized();
    sorted_axes.col(1) = axes.col(1).normalized();
    sorted_axes.col(2) = axes.col(0).normalized();

    if (sorted_axes.col(0).cross(sorted_axes.col(1)).dot(sorted_axes.col(2)) <
        0) {
        sorted_axes.col(2) = -sorted_axes.col(2);
    }

    m_volume = total_volume;
    m_center_of_mass = center_of_mass;
    m_unit_inertia = Vector3{
        std::max(sorted_values[0], real(0)),
        std::max(sorted_values[1], real(0)),
        std::max(sorted_values[2], real(0)),
    };
    m_principal_axes = sorted_axes;
}

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
