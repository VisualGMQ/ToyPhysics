#include "toy_physics/rigid_sim/manifold.hpp"

#include "toy_physics/common/check.hpp"
#include "toy_physics/geometry/box.hpp"
#include "toy_physics/geometry/capsule.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/geometry/cylinder.hpp"
#include "toy_physics/geometry/sphere.hpp"
#include "toy_physics/lowlevel/algorithm.hpp"
#include "toy_physics/lowlevel/epa.hpp"
#include "toy_physics/lowlevel/geom_support.hpp"
#include "toy_physics/rigid_sim/shape.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace toy_physics {

constexpr real kEpsilon = real(1e-5);
constexpr real kDuplicateEpsilonSq = real(1e-8);
constexpr real kFaceTolerance = real(1e-3);
constexpr real kDepthTolerance = real(1e-3);

Vector3 getBoxSupportPoint(const Geometry& geom, const Transform& trans,
                           Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::Box);

    const auto& box = static_cast<const BoxGeometry&>(geom);
    const Matrix33 rotation = trans.m_rotation.toRotationMatrix();

    Vector3 result = trans.m_position;
    for (int i = 0; i < 3; ++i) {
        const Vector3 axis = rotation * box.m_axis[i];
        const real projection = dir.dot(axis);
        result +=
            (projection >= 0 ? box.m_half_extent[i] : -box.m_half_extent[i]) *
            axis;
    }
    return result;
}

Vector3 getSphereSupportPoint(const Geometry& geom, const Transform& trans,
                              Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::Sphere);

    const auto& sphere = static_cast<const SphereGeometry&>(geom);
    Vector3 unit = dir.normalized();
    if (!unit.allFinite()) {
        unit = Vector3::UnitX();
    }
    return trans.m_position + unit * sphere.m_radius;
}

Vector3 getCapsuleSupportPoint(const Geometry& geom, const Transform& trans,
                               Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::Capsule);

    const auto& capsule = static_cast<const CapsuleGeometry&>(geom);
    const Vector3 axis = (trans.m_rotation * Vector3::UnitY()).normalized();
    Vector3 unit = dir.normalized();
    if (!unit.allFinite()) {
        unit = Vector3::UnitX();
    }

    const Vector3 top = trans.m_position + axis * capsule.m_half_height;
    const Vector3 bottom = trans.m_position - axis * capsule.m_half_height;
    const Vector3 segment_end =
        unit.dot(top) >= unit.dot(bottom) ? top : bottom;
    return segment_end + unit * capsule.m_radius;
}

Vector3 getCylinderSupportPoint(const Geometry& geom, const Transform& trans,
                                Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::Cylinder);

    const auto& cylinder = static_cast<const CylinderGeometry&>(geom);
    const Vector3 axis = (trans.m_rotation * Vector3::UnitY()).normalized();
    const CylinderSupportFunction support(
        trans.m_position, axis, cylinder.m_half_height, cylinder.m_radius);
    return support(dir);
}

Vector3 getConvexSupportPoint(const Geometry& geom, const Transform& trans,
                              Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::ConvexHull);

    const auto& hull = static_cast<const ConvexHullGeometry&>(geom);
    const ConvexData* data = hull.GetConvex();
    if (!data) {
        return trans.m_position;
    }
    const std::span<const Vector3> vertices = data->GetVertices();
    if (vertices.empty()) {
        return trans.m_position;
    }

    const Matrix33 rotation = trans.m_rotation.toRotationMatrix();
    real best = -std::numeric_limits<real>::max();
    Vector3 result = trans.m_position;
    for (const Vector3& vertex : vertices) {
        const Vector3 point = trans.m_position + rotation * vertex;
        const real projection = point.dot(dir);
        if (projection > best) {
            best = projection;
            result = point;
        }
    }
    return result;
}

using GeometrySupportFn = Vector3 (*)(const Geometry&, const Transform&,
                                      Vector3 dir);

constexpr size_t kGeometryTypeCount =
    static_cast<size_t>(Geometry::Type::kCount);

const GeometrySupportFn kGetGeomSupportPointFnMap[kGeometryTypeCount] = {
    &getBoxSupportPoint,      &getSphereSupportPoint, &getCapsuleSupportPoint,
    &getCylinderSupportPoint, &getConvexSupportPoint,
};

Vector3 getGeomSupport(const Geometry& geom, const Transform& trans,
                       Vector3 dir) {
    const size_t index = static_cast<size_t>(geom.GetType());
    TOY_ASSERT(index < kGeometryTypeCount);

    return kGetGeomSupportPointFnMap[index](geom, trans, dir);
}

struct Feature {
    std::vector<Vector3> m_points;

    [[nodiscard]] size_t Size() const { return m_points.size(); }

    [[nodiscard]] bool Empty() const { return m_points.empty(); }
};

Feature makeBoxFeature(const Geometry& geom, const Transform& trans,
                       Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::Box);

    const auto& box = static_cast<const BoxGeometry&>(geom);
    Feature feature;
    const Vector3 center = trans.m_position;
    const Matrix33 rotation = trans.m_rotation.toRotationMatrix();
    std::array<Vector3, 3> axes = {
        rotation * box.m_axis[0],
        rotation * box.m_axis[1],
        rotation * box.m_axis[2],
    };

    int axis = 0;
    real best = std::abs(dir.dot(axes[0]));
    for (int i = 1; i < 3; ++i) {
        real d = std::abs(dir.dot(axes[i]));
        if (d > best) {
            best = d;
            axis = i;
        }
    }

    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;
    const real sign = dir.dot(axes[axis]) >= 0 ? real(1) : real(-1);

    const Vector3 face_center =
        center + axes[axis] * (box.m_half_extent[axis] * sign);
    const Vector3 u_axis = axes[u] * box.m_half_extent[u];
    const Vector3 v_axis = axes[v] * box.m_half_extent[v];

    feature.m_points.push_back(face_center + u_axis + v_axis);
    feature.m_points.push_back(face_center - u_axis + v_axis);
    feature.m_points.push_back(face_center - u_axis - v_axis);
    feature.m_points.push_back(face_center + u_axis - v_axis);
    return feature;
}

Feature makeSphereFeature(const Geometry& geom, const Transform& trans,
                          Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::Sphere);

    const auto& sphere = static_cast<const SphereGeometry&>(geom);
    Feature feature;
    Vector3 unit = dir.normalized();
    if (!unit.allFinite() || unit.squaredNorm() <= kEpsilon) {
        unit = Vector3::UnitX();
    }
    feature.m_points.push_back(trans.m_position + unit * sphere.m_radius);
    return feature;
}

Feature makeCapsuleFeature(const Geometry& geom, const Transform& trans,
                           Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::Capsule);

    const auto& capsule = static_cast<const CapsuleGeometry&>(geom);
    Feature feature;
    const Vector3 center = trans.m_position;
    const Vector3 axis = (trans.m_rotation * Vector3::UnitY()).normalized();
    const Vector3 radial = dir - axis * dir.dot(axis);
    const real radial_len = radial.norm();
    const real axial = dir.dot(axis);

    if (radial_len <= kEpsilon) {
        const real sign = axial >= 0 ? real(1) : real(-1);
        feature.m_points.push_back(center +
                                   axis * (sign * capsule.m_half_height) +
                                   dir.normalized() * capsule.m_radius);
        return feature;
    }

    const Vector3 radial_unit = radial / radial_len;
    const Vector3 top =
        center + axis * capsule.m_half_height + radial_unit * capsule.m_radius;
    const Vector3 bottom =
        center - axis * capsule.m_half_height + radial_unit * capsule.m_radius;

    if (std::abs(axial) * capsule.m_half_height <= real(0.01)) {
        feature.m_points.push_back(top);
        feature.m_points.push_back(bottom);
    } else {
        feature.m_points.push_back(axial > 0 ? top : bottom);
    }
    return feature;
}

Feature makeCylinderFeature(const Geometry& geom, const Transform& trans,
                            Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::Cylinder);

    const auto& cylinder = static_cast<const CylinderGeometry&>(geom);
    Feature feature;
    const Vector3 center = trans.m_position;
    const Vector3 axis = (trans.m_rotation * Vector3::UnitY()).normalized();
    const Vector3 radial = dir - axis * dir.dot(axis);
    const real radial_sq = radial.squaredNorm();
    const real axial_sq = dir.dot(axis) * dir.dot(axis);

    if (radial_sq > axial_sq && radial_sq > kEpsilon) {
        const Vector3 radial_unit = radial.normalized();
        feature.m_points.push_back(center + axis * cylinder.m_half_height +
                                   radial_unit * cylinder.m_radius);
        feature.m_points.push_back(center - axis * cylinder.m_half_height +
                                   radial_unit * cylinder.m_radius);
        return feature;
    }

    const real sign = dir.dot(axis) >= 0 ? real(1) : real(-1);
    const Vector3 cap_center = center + axis * (sign * cylinder.m_half_height);
    Vector3 u = radial_sq > kEpsilon
                    ? radial.normalized()
                    : (trans.m_rotation * Vector3::UnitX()).normalized();
    u = (u - axis * u.dot(axis)).normalized();
    if (!u.allFinite() || u.squaredNorm() <= kEpsilon) {
        u = axis.unitOrthogonal();
    }
    const Vector3 v = axis.cross(u).normalized();

    constexpr int kCapSegments = 8;
    for (int i = 0; i < kCapSegments; ++i) {
        const real angle = real(2) * MATH_PI * static_cast<real>(i) /
                           static_cast<real>(kCapSegments);
        feature.m_points.push_back(cap_center +
                                   (std::cos(angle) * u + std::sin(angle) * v) *
                                       cylinder.m_radius);
    }
    return feature;
}

Feature makeConvexFeature(const Geometry& geom, const Transform& trans,
                          Vector3 dir) {
    TOY_ASSERT(geom.GetType() == Geometry::Type::ConvexHull);

    const auto& hull = static_cast<const ConvexHullGeometry&>(geom);
    Feature feature;
    const ConvexData* data = hull.GetConvex();
    if (!data) {
        return feature;
    }
    const std::span<const Vector3> vertices = data->GetVertices();
    if (vertices.empty()) {
        return feature;
    }

    const Matrix33 rotation = trans.m_rotation.toRotationMatrix();
    std::vector<Vector3> world;
    world.reserve(vertices.size());
    real max_projection = -std::numeric_limits<real>::max();
    for (const Vector3& v : vertices) {
        Vector3 p = trans.m_position + rotation * v;
        real projection = p.dot(dir);
        max_projection = std::max(max_projection, projection);
        world.push_back(p);
    }

    for (const Vector3& p : world) {
        if (max_projection - p.dot(dir) <= kFaceTolerance) {
            feature.m_points.push_back(p);
        }
    }

    if (feature.m_points.size() >= 3) {
        Vector3 centroid = Vector3::Zero();
        for (const Vector3& p : feature.m_points) {
            centroid += p;
        }
        centroid /= static_cast<real>(feature.m_points.size());

        Vector3 u = dir.unitOrthogonal();
        Vector3 v = dir.cross(u).normalized();
        auto angle = [&](const Vector3& p) {
            Vector3 d = p - centroid;
            return std::atan2(d.dot(v), d.dot(u));
        };
        std::sort(feature.m_points.begin(), feature.m_points.end(),
                  [&](const Vector3& a, const Vector3& b) {
                      return angle(a) < angle(b);
                  });
    }

    return feature;
}

using GeometryFeatureFn = Feature (*)(const Geometry&, const Transform&,
                                      Vector3 dir);

const GeometryFeatureFn kMakeGeomFeatureFnMap[kGeometryTypeCount] = {
    &makeBoxFeature,      &makeSphereFeature, &makeCapsuleFeature,
    &makeCylinderFeature, &makeConvexFeature,
};

Feature makeFeature(const Geometry& geom, const Transform& trans, Vector3 dir) {
    const size_t index = static_cast<size_t>(geom.GetType());
    TOY_ASSERT(index < kGeometryTypeCount);
    if (index >= kGeometryTypeCount) {
        return {};
    }
    return kMakeGeomFeatureFnMap[index](geom, trans, dir);
}

Vector3 polygonNormal(const std::vector<Vector3>& polygon) {
    if (polygon.size() < 3) {
        return Vector3::Zero();
    }

    for (size_t i = 1; i + 1 < polygon.size(); ++i) {
        Vector3 normal =
            (polygon[i] - polygon[0]).cross(polygon[i + 1] - polygon[0]);
        real len = normal.norm();
        if (len > kEpsilon) {
            return normal / len;
        }
    }
    return Vector3::Zero();
}

std::vector<Vector3> clipPolygonByPlane(const std::vector<Vector3>& polygon,
                                        Vector3 plane_point, Vector3 normal) {
    std::vector<Vector3> output;
    if (polygon.empty()) {
        return output;
    }

    Vector3 previous = polygon.back();
    real prev_distance = (previous - plane_point).dot(normal);
    bool prev_inside = prev_distance >= -kEpsilon;

    for (const Vector3& current : polygon) {
        real current_distance = (current - plane_point).dot(normal);
        bool current_inside = current_distance >= -kEpsilon;

        if (current_inside != prev_inside) {
            real denominator = prev_distance - current_distance;
            if (std::abs(denominator) > kEpsilon) {
                real t = prev_distance / denominator;
                output.push_back(previous + (current - previous) * t);
            }
        }

        if (current_inside) {
            output.push_back(current);
        }

        previous = current;
        prev_distance = current_distance;
        prev_inside = current_inside;
    }

    std::vector<Vector3> deduplicated;
    deduplicated.reserve(output.size());
    for (const Vector3& point : output) {
        if (deduplicated.empty() ||
            (point - deduplicated.back()).squaredNorm() > kDuplicateEpsilonSq) {
            deduplicated.push_back(point);
        }
    }
    if (deduplicated.size() > 1 &&
        (deduplicated.front() - deduplicated.back()).squaredNorm() <=
            kDuplicateEpsilonSq) {
        deduplicated.pop_back();
    }

    return deduplicated;
}

bool clipPolygonToFace(std::vector<Vector3>& polygon,
                       const std::vector<Vector3>& face,
                       Vector3* out_face_normal) {
    if (polygon.empty() || face.size() < 3) {
        return false;
    }

    *out_face_normal = polygonNormal(face);
    if (out_face_normal->squaredNorm() <= kEpsilon) {
        return false;
    }

    Vector3 centroid = Vector3::Zero();
    for (const Vector3& p : face) {
        centroid += p;
    }
    centroid /= static_cast<real>(face.size());

    for (size_t i = 0; i < face.size(); ++i) {
        const Vector3& a = face[i];
        const Vector3& b = face[(i + 1) % face.size()];
        Vector3 edge = b - a;
        Vector3 inward = edge.cross(*out_face_normal);
        if (inward.squaredNorm() <= kEpsilon) {
            continue;
        }
        if (inward.dot(centroid - a) < 0) {
            inward = -inward;
        }
        polygon = clipPolygonByPlane(polygon, a, inward);
        if (polygon.empty()) {
            return false;
        }
    }
    return true;
}

struct Contact {
    Vector3 m_point1;
    Vector3 m_point2;
    real m_depth;
};

bool buildParallelEdgeContacts(const Feature& feature1, const Feature& feature2,
                               Vector3 normal,
                               std::vector<Contact>& out_contacts) {
    if (feature1.Size() != 2 || feature2.Size() != 2) {
        return false;
    }

    Vector3 edge1 = feature1.m_points[1] - feature1.m_points[0];
    Vector3 edge2 = feature2.m_points[1] - feature2.m_points[0];
    if (edge1.squaredNorm() <= kEpsilon || edge2.squaredNorm() <= kEpsilon) {
        return false;
    }

    Vector3 dir1 = edge1.normalized();
    Vector3 dir2 = edge2.normalized();
    if (std::abs(dir1.dot(dir2)) < real(0.9)) {
        return false;
    }

    Vector3 direction = dir1;
    auto projection = [&](const Vector3& p) { return p.dot(direction); };

    real a1 = projection(feature1.m_points[0]);
    real b1 = projection(feature1.m_points[1]);
    Vector3 e1a = feature1.m_points[0];
    Vector3 e1b = feature1.m_points[1];
    if (a1 > b1) {
        std::swap(a1, b1);
        std::swap(e1a, e1b);
    }

    real a2 = projection(feature2.m_points[0]);
    real b2 = projection(feature2.m_points[1]);
    Vector3 e2a = feature2.m_points[0];
    Vector3 e2b = feature2.m_points[1];
    if (a2 > b2) {
        std::swap(a2, b2);
        std::swap(e2a, e2b);
    }

    const real low = std::max(a1, a2);
    const real high = std::min(b1, b2);
    if (high < low - kEpsilon) {
        return false;
    }

    auto point_at = [](const Vector3& a, const Vector3& b, real lower,
                       real upper, real t) -> Vector3 {
        if (upper - lower <= kEpsilon) {
            return a;
        }
        real s = (t - lower) / (upper - lower);
        s = std::clamp(s, real(0), real(1));
        return a + (b - a) * s;
    };

    auto add_contact = [&](real t) {
        Vector3 p1 = point_at(e1a, e1b, a1, b1, t);
        Vector3 p2 = point_at(e2a, e2b, a2, b2, t);
        real depth = (p1 - p2).dot(normal);
        if (depth >= -kDepthTolerance) {
            out_contacts.push_back({p1, p2, std::max(depth, real(0))});
        }
    };

    if (high - low <= kEpsilon) {
        add_contact((low + high) * real(0.5));
    } else {
        add_contact(low);
        add_contact(high);
    }

    return !out_contacts.empty();
}

bool buildFeatureContacts(const Feature& feature1, const Feature& feature2,
                          Vector3 normal, Vector3 witness1, Vector3 witness2,
                          real penetration_depth,
                          std::vector<Contact>& out_contacts) {
    if (feature1.Empty() || feature2.Empty()) {
        return false;
    }

    // A point feature can only produce a single contact point. The EPA witness
    // pair already is that contact, so there is no face/edge to clip.
    if (feature1.Size() == 1 || feature2.Size() == 1) {
        out_contacts.push_back({witness1, witness2, penetration_depth});
        return true;
    }

    if (feature1.Size() == 2 && feature2.Size() == 2) {
        if (buildParallelEdgeContacts(feature1, feature2, normal,
                                      out_contacts)) {
            return true;
        }
        auto [p1, p2] =
            GetSegSegNearestPoints(feature1.m_points[0], feature1.m_points[1],
                                   feature2.m_points[0], feature2.m_points[1]);
        real depth = (p1 - p2).dot(normal);
        if (depth >= -kDepthTolerance) {
            out_contacts.push_back({p1, p2, std::max(depth, real(0))});
            return true;
        }
        return false;
    }

    const Feature* reference = nullptr;
    const Feature* incident = nullptr;
    bool reference_is_first = false;

    if (feature1.Size() >= 3) {
        reference = &feature1;
        incident = &feature2;
        reference_is_first = true;
    } else if (feature2.Size() >= 3) {
        reference = &feature2;
        incident = &feature1;
        reference_is_first = false;
    } else {
        return false;
    }

    std::vector<Vector3> clipped = incident->m_points;
    Vector3 reference_normal = Vector3::Zero();
    if (!clipPolygonToFace(clipped, reference->m_points, &reference_normal)) {
        return false;
    }

    const real denominator = normal.dot(reference_normal);
    if (std::abs(denominator) <= kEpsilon) {
        return false;
    }

    for (const Vector3& point : clipped) {
        real distance = (point - reference->m_points[0]).dot(reference_normal) /
                        denominator;
        Vector3 reference_point = point - distance * normal;
        Vector3 point1 = reference_is_first ? reference_point : point;
        Vector3 point2 = reference_is_first ? point : reference_point;
        real depth = (point1 - point2).dot(normal);
        if (depth < -kDepthTolerance) {
            continue;
        }
        out_contacts.push_back({point1, point2, std::max(depth, real(0))});
    }

    return !out_contacts.empty();
}

void removeDuplicateContacts(std::vector<Contact>& contacts) {
    if (contacts.size() <= 1) {
        return;
    }

    std::vector<Contact> unique;
    unique.reserve(contacts.size());
    for (const Contact& contact : contacts) {
        bool duplicate = false;
        for (const Contact& existing : unique) {
            if ((contact.m_point1 - existing.m_point1).squaredNorm() <=
                    kDuplicateEpsilonSq &&
                (contact.m_point2 - existing.m_point2).squaredNorm() <=
                    kDuplicateEpsilonSq) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            unique.push_back(contact);
        }
    }
    contacts = std::move(unique);
}

// select "the most valuable" 4 contact pairs for manifold.
void pruneContacts(std::vector<Contact>& contacts, Vector3 normal,
                   Vector3 center_of_mass) {
    if (contacts.size() <= 4) {
        return;
    }

    std::vector<Vector3> projected;
    std::vector<real> depth_sq;
    projected.reserve(contacts.size());
    depth_sq.reserve(contacts.size());

    constexpr real min_distance_sq = real(1e-6);
    for (const Contact& contact : contacts) {
        Vector3 p = contact.m_point1 - center_of_mass;
        p -= normal * p.dot(normal);
        projected.push_back(p);

        const Vector3 separation = contact.m_point2 - contact.m_point1;
        depth_sq.push_back(std::max(min_distance_sq, separation.squaredNorm()));
    }

    auto weight = [&](size_t i) {
        return std::max(min_distance_sq, projected[i].squaredNorm()) *
               depth_sq[i];
    };

    size_t point1 = 0;
    real best = weight(0);
    for (size_t i = 1; i < contacts.size(); ++i) {
        real value = weight(i);
        if (value > best) {
            best = value;
            point1 = i;
        }
    }

    size_t point2 = 0;
    best = -std::numeric_limits<real>::max();
    for (size_t i = 0; i < contacts.size(); ++i) {
        if (i == point1) {
            continue;
        }
        real value =
            std::max(min_distance_sq,
                     (projected[i] - projected[point1]).squaredNorm()) *
            depth_sq[i];
        if (value > best) {
            best = value;
            point2 = i;
        }
    }

    size_t point3 = contacts.size();
    size_t point4 = contacts.size();
    real min_value = 0;
    real max_value = 0;
    Vector3 perpendicular =
        (projected[point2] - projected[point1]).cross(normal);
    for (size_t i = 0; i < contacts.size(); ++i) {
        if (i == point1 || i == point2) {
            continue;
        }
        real value = perpendicular.dot(projected[i] - projected[point1]);
        if (value < min_value) {
            min_value = value;
            point3 = i;
        } else if (value > max_value) {
            max_value = value;
            point4 = i;
        }
    }

    std::vector<Contact> result;
    result.reserve(4);
    result.push_back(contacts[point1]);
    if (point3 < contacts.size()) {
        result.push_back(contacts[point3]);
    }
    result.push_back(contacts[point2]);
    if (point4 < contacts.size()) {
        result.push_back(contacts[point4]);
    }
    contacts = std::move(result);
}

void fillManifold(Manifold* manifold, std::vector<Contact> contacts,
                  Vector3 normal, real penetration_depth,
                  Vector3 center_of_mass) {
    removeDuplicateContacts(contacts);
    pruneContacts(contacts, normal, center_of_mass);

    manifold->m_contact_num = static_cast<uint8_t>(
        std::min<size_t>(contacts.size(), Manifold::kMaxContacts));
    for (uint8_t i = 0; i < manifold->m_contact_num; ++i) {
        manifold->m_contacts1[i] = contacts[i].m_point1;
        manifold->m_contacts2[i] = contacts[i].m_point2;
        manifold->m_depth[static_cast<int>(i)] = contacts[i].m_depth;
    }

    manifold->m_separate_normal = normal;
    manifold->m_penetration_depth = penetration_depth;
}

bool ComputePenetration(const Geometry& geom1, const Transform& trans1,
                        const Geometry& geom2, const Transform& trans2,
                        Vector3* out_penetrate, Vector3* out_witness1,
                        Vector3* out_witness2) {
    const GeomSupportFunction support1(geom1, trans1);
    const GeomSupportFunction support2(geom2, trans2);

    Vector3 penetration;
    Vector3 witness1;
    Vector3 witness2;
    if (!CalcPenetrationDepth(support1, support2, penetration, witness1,
                              witness2, kPenetrationTolerance)) {
        return false;
    }

    if (out_penetrate) {
        *out_penetrate = penetration;
    }
    if (out_witness1) {
        *out_witness1 = witness1;
    }
    if (out_witness2) {
        *out_witness2 = witness2;
    }
    return true;
}

bool ComputeManifold(const Shape& shape1, const Shape& shape2,
                     Manifold* out_manifold) {
    if (!out_manifold) {
        return false;
    }

    *out_manifold = Manifold{};
    out_manifold->m_shape1 = &shape1;
    out_manifold->m_shape2 = &shape2;

    Vector3 penetration;
    Vector3 witness1;
    Vector3 witness2;

    const Geometry& geom1 = *shape1.GetGeometry();
    const Geometry& geom2 = *shape2.GetGeometry();
    const Transform& trans1 = shape1.GetTransform();
    const Transform& trans2 = shape2.GetTransform();

    if (!ComputePenetration(geom1, trans1, geom2, trans2, &penetration,
                            &witness1, &witness2)) {
        return false;
    }

    const real depth = penetration.norm();
    if (depth <= kEpsilon) {
        return false;
    }

    const Vector3 normal = penetration / depth;
    const Feature feature1 = makeFeature(geom1, trans1, normal);
    const Feature feature2 = makeFeature(geom2, trans2, -normal);

    std::vector<Contact> contacts;
    buildFeatureContacts(feature1, feature2, normal, witness1, witness2, depth,
                         contacts);
    if (contacts.empty()) {
        contacts.push_back({witness1, witness2, depth});
    }

    fillManifold(out_manifold, std::move(contacts), normal, depth,
                 trans1.m_position);
    return out_manifold->m_contact_num > 0;
}

}  // namespace toy_physics
