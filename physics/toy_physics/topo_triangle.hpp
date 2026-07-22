#pragma once
#include "toy_physics/math.hpp"

#include <span>

namespace toy_physics {

template <size_t>
class TopoTriangleStore;

class TopoTriangle;

class Edge;

/**
 * Edge linked nearby 2 TopoTriangles
 * @see TopoTriangle
 */
class Edge {
public:
    Edge() = default;

    Edge(TopoTriangle* triangle, int index)
        : m_triangle(triangle), m_index(index) {}

    template <size_t MaxCount>
    bool Suture(std::span<const Vector3> verts, size_t index,
                TopoTriangleStore<MaxCount>& triangle_store) const;

    [[nodiscard]] TopoTriangle* triangle() const;

    [[nodiscard]] int Index() const;

    [[nodiscard]] size_t GetSource() const;
    [[nodiscard]] size_t GetTarget() const;

    bool link(const Edge&) const;
    void half_link(const Edge&) const;

private:
    TopoTriangle* m_triangle{nullptr};
    int m_index{0};
};

/**
 * Store triangle by it's topological form.
 *
 * Only store vertex indices, access vertices from TopoTriangleStore
 * @see TopoTriangleStore
 */
class TopoTriangle {
public:
    friend class Edge;

    TopoTriangle() = default;

    TopoTriangle(size_t i0, size_t i1, size_t i2);

    size_t operator[](size_t i) const;

    [[nodiscard]] const Edge& GetAdjEdge(size_t i) const;

    void SetObsolete(bool obsolete);

    [[nodiscard]] bool IsObsolete() const;

    [[nodiscard]] bool ComputeClosest(std::span<const Vector3> verts);

    [[nodiscard]] const Vector3& GetClosest() const;

    [[nodiscard]] bool IsClosestInternal() const;

    [[nodiscard]] bool IsVisibleFrom(std::span<const Vector3> verts,
                                     size_t index) const;

    [[nodiscard]] real GetDist2() const;

    [[nodiscard]] Vector3 GetClosestPoint(
        std::span<const Vector3> points) const;

    template <size_t MaxCount>
    bool Suture(std::span<const Vector3> verts, size_t index,
                TopoTriangleStore<MaxCount>& triangle_store);

private:
    size_t m_indices[3];
    Edge m_adjEdges[3];

    bool m_obsolete;

    real m_det;
    real m_lambda1;
    real m_lambda2;
    Vector3 m_closest;
    real m_dist2;
};

template <size_t MaxCount>
class TopoTriangleStore {
public:
    void Clear() { m_free = 0; }

    [[nodiscard]] int GetFree() const { return m_free; }

    TopoTriangle& operator[](size_t i) { return m_triangles[i]; }

    const TopoTriangle& operator[](size_t i) const { return m_triangles[i]; }

    TopoTriangle& Last() { return m_triangles[m_free - 1]; }

    void SetFree(int backup) { m_free = backup; }

    TopoTriangle* NewTriangle(std::span<const Vector3> verts, size_t i0,
                              size_t i1, size_t i2) {
        TopoTriangle* triangle = nullptr;
        if (m_free != MaxCount) {
            triangle = &m_triangles[m_free++];
            new (triangle) TopoTriangle(i0, i1, i2);
            if (!triangle->ComputeClosest(verts)) {
                --m_free;
                triangle = nullptr;
            }
        }

        return triangle;
    }

private:
    std::array<TopoTriangle, MaxCount> m_triangles;
    int m_free{0};
};

inline int circ_next(int i) {
    return (i + 1) % 3;
}

inline int circ_prev(int i) {
    return (i + 2) % 3;
}

template <size_t MaxCount>
bool Edge::Suture(std::span<const Vector3> verts, size_t index,
                  TopoTriangleStore<MaxCount>& triangle_store) const {
    if (!m_triangle->IsObsolete()) {
        if (!m_triangle->IsVisibleFrom(verts, index)) {
            TopoTriangle* triangle = triangle_store.NewTriangle(
                verts, index, GetTarget(), GetSource());

            if (triangle) {
                Edge(triangle, 1).half_link(*this);
                return true;
            }

            return false;
        }

        m_triangle->SetObsolete(true);

        int backup = triangle_store.GetFree();

        if (!m_triangle->GetAdjEdge(circ_next(m_index))
                 .Suture(verts, index, triangle_store)) {
            m_triangle->SetObsolete(false);

            TopoTriangle* triangle = triangle_store.NewTriangle(
                verts, index, GetTarget(), GetSource());

            if (triangle) {
                Edge(triangle, 1).half_link(*this);
                return true;
            }

            return false;
        }

        if (!m_triangle->GetAdjEdge(circ_prev(m_index))
                 .Suture(verts, index, triangle_store)) {
            m_triangle->SetObsolete(false);

            triangle_store.SetFree(backup);

            TopoTriangle* triangle = triangle_store.NewTriangle(
                verts, index, GetTarget(), GetSource());

            if (triangle) {
                Edge(triangle, 1).half_link(*this);
                return true;
            }

            return false;
        }
    }

    return true;
}

template <size_t MaxCount>
bool TopoTriangle::Suture(std::span<const Vector3> verts, size_t index,
                          TopoTriangleStore<MaxCount>& triangle_store) {
    int first = triangle_store.GetFree();

    SetObsolete(true);

    bool result = m_adjEdges[0].Suture(verts, index, triangle_store) &&
                  m_adjEdges[1].Suture(verts, index, triangle_store) &&
                  m_adjEdges[2].Suture(verts, index, triangle_store);

    if (result) {
        for (int i = first, j = triangle_store.GetFree() - 1;
             i != triangle_store.GetFree(); j = i++) {
            TopoTriangle* triangle = &triangle_store[i];
            triangle->GetAdjEdge(1).half_link(Edge(triangle, 1));
            if (!Edge(triangle, 0).link(Edge(&triangle_store[j], 2))) {
                return false;
            }
        }
    }

    return result;
}

}  // namespace toy_physics
