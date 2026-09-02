#include "toy_physics/lowlevel/topo_triangle.hpp"

#include "toy_physics/common/check.hpp"

namespace toy_physics {

TopoTriangle* Edge::triangle() const {
    return m_triangle;
}

int Edge::Index() const {
    return m_index;
}

size_t Edge::GetSource() const {
    return (*m_triangle)[m_index];
}

size_t Edge::GetTarget() const {
    return (*m_triangle)[circ_next(m_index)];
}

bool Edge::link(const Edge& o) const {
    bool ok = GetSource() == o.GetTarget() && GetTarget() == o.GetSource();

    if (ok) {
        triangle()->m_adjEdges[Index()] = o;
        o.triangle()->m_adjEdges[o.Index()] = *this;
    }

    return ok;
}

void Edge::half_link(const Edge& o) const {
    TOY_ASSERT(GetSource() == o.GetTarget() && GetTarget() == o.GetSource());

    triangle()->m_adjEdges[Index()] = o;
}

TopoTriangle::TopoTriangle(size_t i0, size_t i1, size_t i2)
    : m_indices{i0, i1, i2}, m_obsolete(false) {}

size_t TopoTriangle::operator[](size_t i) const {
    return m_indices[i];
}

const Edge& TopoTriangle::GetAdjEdge(size_t i) const {
    return m_adjEdges[i];
}

void TopoTriangle::SetObsolete(bool obsolete) {
    m_obsolete = obsolete;
}

bool TopoTriangle::IsObsolete() const {
    return m_obsolete;
}

bool TopoTriangle::ComputeClosest(std::span<const Vector3> verts) {
    Vector3 p0 = verts[m_indices[0]];

    Vector3 v1 = verts[m_indices[1]] - p0;
    Vector3 v2 = verts[m_indices[2]] - p0;
    real v1dv1 = v1.squaredNorm();
    real v1dv2 = v1.dot(v2);
    real v2dv2 = v2.squaredNorm();
    real p0dv1 = p0.dot(v1);
    real p0dv2 = p0.dot(v2);

    m_det = v1dv1 * v2dv2 - v1dv2 * v1dv2;  // non-negative
    m_lambda1 = p0dv2 * v1dv2 - p0dv1 * v2dv2;
    m_lambda2 = p0dv1 * v1dv2 - p0dv2 * v1dv1;

    if (m_det > 0) {
        m_closest = p0 + (m_lambda1 * v1 + m_lambda2 * v2) / m_det;
        m_dist2 = m_closest.squaredNorm();

        return true;
    }

    return false;
}

const Vector3& TopoTriangle::GetClosest() const {
    return m_closest;
}

bool TopoTriangle::IsClosestInternal() const {
    return m_lambda1 >= 0 && m_lambda2 >= 0 && m_lambda1 + m_lambda2 <= m_det;
}

bool TopoTriangle::IsVisibleFrom(std::span<const Vector3> verts,
                                 size_t index) const {
    Vector3 lever = verts[index] - m_closest;
    return m_closest.dot(lever) > 0;
}

real TopoTriangle::GetDist2() const {
    return m_dist2;
}

Vector3 TopoTriangle::GetClosestPoint(std::span<const Vector3> points) const {
    const Vector3& p0 = points[m_indices[0]];

    return p0 + (m_lambda1 * (points[m_indices[1]] - p0) +
                 m_lambda2 * (points[m_indices[2]] - p0)) /
                    m_det;
}

}  // namespace toy_physics