#include "toy_physics/epa.hpp"
#include "toy_physics/gjk.hpp"
#include "toy_physics/topo_triangle.hpp"

namespace toy_physics {

static TopoTriangleStore<kEPA_MAX_FACE_COUNT> triangle_store;

static std::array<Vector3, kEPA_MAX_POINT_COUNT> pBuf;
static std::array<Vector3, kEPA_MAX_POINT_COUNT> qBuf;
static std::array<Vector3, kEPA_MAX_POINT_COUNT> yBuf;

static std::array<TopoTriangle*, kEPA_MAX_FACE_COUNT> triangle_heap{nullptr};

static constexpr real kEPARelError2 = 1e-6;

class TriangleComp {
public:
    bool operator()(const TopoTriangle* face1, const TopoTriangle* face2) {
        return face1->GetDist2() > face2->GetDist2();
    }
};

static TriangleComp triangle_comp;

void addCandidate(TopoTriangle* triangle, real upper2, int& num_triangles) {
    if (triangle->IsClosestInternal() && triangle->GetDist2() <= upper2) {
        triangle_heap[num_triangles++] = triangle;
        std::push_heap(&triangle_heap[0], &triangle_heap[num_triangles],
                       triangle_comp);
    }
}

int IsOriginInTetrahedron(const Vector3& p1, const Vector3& p2,
                          const Vector3& p3, const Vector3& p4) {
    Vector3 normal1 = (p2 - p1).cross(p3 - p1);
    if ((normal1.dot(p1) > 0) == (normal1.dot(p4) > 0)) {
        return 4;
    }

    Vector3 normal2 = (p4 - p2).cross(p3 - p2);
    if ((normal2.dot(p2) > 0) == (normal2.dot(p1) > 0)) {
        return 1;
    }

    Vector3 normal3 = (p4 - p3).cross(p1 - p3);
    if ((normal3.dot(p3) > 0) == (normal3.dot(p2) > 0)) {
        return 2;
    }

    Vector3 normal4 = (p2 - p4).cross(p1 - p4);
    if ((normal4.dot(p4) > 0) == (normal4.dot(p3) > 0)) {
        return 3;
    }

    return 0;
}

bool CalcPenetrationDepth(GJK& gjk,
                          const toy_physics::SupportFunction& support1,
                          const SupportFunction& support2,
                          Vector3& out_penetration, Vector3& out_p1,
                          Vector3& out_p2, real tolerance) {
    int num_verts = gjk.GetSimplex(pBuf, qBuf, yBuf);
    tolerance = tolerance * gjk.GetMaxDist2();

    int num_triangles = 0;

    triangle_store.Clear();

    switch (num_verts) {
        case 1:
            return false;
        case 2: {
            Vector3 dir = yBuf[1] - yBuf[0];

            dir.normalize();
            int axis = GetFurthestAxis(dir);

            static const real sin_60 = std::sqrt(real(3.0)) * real(0.5);

            Quaternion rot(0.5, dir[0] * sin_60, dir[1] * sin_60,
                           dir[2] * sin_60);
            Matrix33 rot_mat{rot};

            Vector3 aux1 = dir.cross(Vector3(axis == 0, axis == 1, axis == 2));
            Vector3 aux2 = rot_mat * aux1;
            Vector3 aux3 = rot_mat * aux2;

            pBuf[2] = support1(aux1);
            qBuf[2] = support2(-aux1);
            yBuf[2] = pBuf[2] - qBuf[2];

            pBuf[3] = support1(aux2);
            qBuf[3] = support2(-aux2);
            yBuf[3] = pBuf[3] - qBuf[3];

            pBuf[4] = support1(aux3);
            qBuf[4] = support2(-aux3);
            yBuf[4] = pBuf[4] - qBuf[4];

            if (IsOriginInTetrahedron(yBuf[0], yBuf[2], yBuf[3], yBuf[4]) ==
                0) {
                pBuf[1] = pBuf[4];
                qBuf[1] = qBuf[4];
                yBuf[1] = yBuf[4];
            } else if (IsOriginInTetrahedron(yBuf[1], yBuf[2], yBuf[3],
                                             yBuf[4]) == 0) {
                pBuf[0] = pBuf[4];
                qBuf[0] = qBuf[4];
                yBuf[0] = yBuf[4];
            } else {
                // Origin not in initial polytope
                return false;
            }

            num_verts = 4;
        }
            [[fallthrough]];
        case 4: {
            int bad_vertex =
                IsOriginInTetrahedron(yBuf[0], yBuf[1], yBuf[2], yBuf[3]);

            if (bad_vertex == 0) {
                TopoTriangle* f0 = triangle_store.NewTriangle(yBuf, 0, 1, 2);
                TopoTriangle* f1 = triangle_store.NewTriangle(yBuf, 0, 3, 1);
                TopoTriangle* f2 = triangle_store.NewTriangle(yBuf, 0, 2, 3);
                TopoTriangle* f3 = triangle_store.NewTriangle(yBuf, 1, 3, 2);

                if (!(f0 && f0->GetDist2() > 0 && f1 && f1->GetDist2() > 0 &&
                      f2 && f2->GetDist2() > 0 && f3 && f3->GetDist2() > 0)) {
                    return false;
                }

                Edge(f0, 0).link(Edge(f1, 2));
                Edge(f0, 1).link(Edge(f3, 2));
                Edge(f0, 2).link(Edge(f2, 0));
                Edge(f1, 0).link(Edge(f2, 2));
                Edge(f1, 1).link(Edge(f3, 0));
                Edge(f2, 1).link(Edge(f3, 1));

                addCandidate(f0, kREAL_MAX, num_triangles);
                addCandidate(f1, kREAL_MAX, num_triangles);
                addCandidate(f2, kREAL_MAX, num_triangles);
                addCandidate(f3, kREAL_MAX, num_triangles);
                break;
            }

            if (bad_vertex < 4) {
                pBuf[bad_vertex - 1] = pBuf[4];
                qBuf[bad_vertex - 1] = qBuf[4];
                yBuf[bad_vertex - 1] = yBuf[4];
            }

            num_verts = 3;
        }
            [[fallthrough]];
        case 3: {
            // We have a triangle inside the Minkowski sum containing
            // the origin. First blow it up.

            Vector3 v1 = yBuf[1] - yBuf[0];
            Vector3 v2 = yBuf[2] - yBuf[0];
            Vector3 vv = v1.cross(v2);

            pBuf[3] = support1(vv);
            qBuf[3] = support2(-vv);
            yBuf[3] = pBuf[3] - qBuf[3];
            pBuf[4] = support1(-vv);
            qBuf[4] = support2(vv);
            yBuf[4] = pBuf[4] - qBuf[4];

            TopoTriangle* f0 = triangle_store.NewTriangle(yBuf, 0, 1, 3);
            TopoTriangle* f1 = triangle_store.NewTriangle(yBuf, 1, 2, 3);
            TopoTriangle* f2 = triangle_store.NewTriangle(yBuf, 2, 0, 3);
            TopoTriangle* f3 = triangle_store.NewTriangle(yBuf, 0, 2, 4);
            TopoTriangle* f4 = triangle_store.NewTriangle(yBuf, 2, 1, 4);
            TopoTriangle* f5 = triangle_store.NewTriangle(yBuf, 1, 0, 4);

            if (!(f0 && f0->GetDist2() > 0 && f1 && f1->GetDist2() > 0 && f2 &&
                  f2->GetDist2() > 0 && f3 && f3->GetDist2() > 0 && f4 &&
                  f4->GetDist2() > 0 && f5 && f5->GetDist2() > 0)) {
                return false;
            }

            Edge(f0, 1).link(Edge(f1, 2));
            Edge(f1, 1).link(Edge(f2, 2));
            Edge(f2, 1).link(Edge(f0, 2));

            Edge(f0, 0).link(Edge(f5, 0));
            Edge(f1, 0).link(Edge(f4, 0));
            Edge(f2, 0).link(Edge(f3, 0));

            Edge(f3, 1).link(Edge(f4, 2));
            Edge(f4, 1).link(Edge(f5, 2));
            Edge(f5, 1).link(Edge(f3, 2));

            addCandidate(f0, kREAL_MAX, num_triangles);
            addCandidate(f1, kREAL_MAX, num_triangles);
            addCandidate(f2, kREAL_MAX, num_triangles);
            addCandidate(f3, kREAL_MAX, num_triangles);
            addCandidate(f4, kREAL_MAX, num_triangles);
            addCandidate(f5, kREAL_MAX, num_triangles);

            num_verts = 5;
        } break;
    }

    // We have a polytope inside the Minkowski sum containing
    // the origin.

    if (num_triangles == 0) {
        return false;
    }

    // at least one triangle on the heap.

    TopoTriangle* triangle = nullptr;

    real upper_bound2 = kREAL_MAX;

    do {
        triangle = triangle_heap[0];
        std::pop_heap(&triangle_heap[0], &triangle_heap[num_triangles],
                      triangle_comp);
        --num_triangles;

        if (!triangle->IsObsolete()) {
            if (num_verts == kEPA_MAX_POINT_COUNT) {
                assert(false);
                break;
            }

            pBuf[num_verts] = support1(triangle->GetClosest());
            qBuf[num_verts] = support2(-triangle->GetClosest());
            yBuf[num_verts] = pBuf[num_verts] - qBuf[num_verts];

            int index = num_verts++;
            real far_dist = yBuf[index].dot(triangle->GetClosest());

            TOY_ASSERT(far_dist > 0);
            real far_dist2 = far_dist * far_dist / triangle->GetDist2();
            upper_bound2 = std::min(upper_bound2, far_dist2);

            real error = far_dist - triangle->GetDist2();
            if (error <= std::max(kEPARelError2 * far_dist, tolerance) ||
                yBuf[index] == yBuf[(*triangle)[0]] ||
                yBuf[index] == yBuf[(*triangle)[1]] ||
                yBuf[index] == yBuf[(*triangle)[2]]) {
                break;
            }

            int i = triangle_store.GetFree();

            if (!triangle->Suture(yBuf, index, triangle_store)) {
                break;
            }

            while (i != triangle_store.GetFree()) {
                TopoTriangle* new_triangle = &triangle_store[i];
                addCandidate(new_triangle, upper_bound2, num_triangles);
                ++i;
            }
        }
    } while (num_triangles > 0 && triangle_heap[0]->GetDist2() <= upper_bound2);

    out_penetration = triangle->GetClosest();
    out_p1 = triangle->GetClosestPoint(pBuf);
    out_p2 = triangle->GetClosestPoint(qBuf);
    return true;
}

bool CalcPenetrationDepth(const SupportFunction& support1,
                          const SupportFunction& support2,
                          Vector3& out_penetration, Vector3& out_p1,
                          Vector3& out_p2, real tolerance) {
    GJK gjk;
    Vector3 v = Vector3::UnitX();
    Vector3 p1, p2;
    Vector3 w;
    real dist2 = std::numeric_limits<real>::max();
    real delta;

    do {
        p1 = support1(-v);
        p2 = support2(v);
        w = p1 - p2;
        // origin behind half-plane
        if (w.dot(v) > 0) {
            return false;
        }

        gjk.AddVertex(w, p1, p2);

        if (gjk.IsAffinelyDependent()) {
            return false;
        }

        if (!gjk.GetClosest(v)) {
            return false;
        }

        real prev_dist2 = dist2;
        dist2 = v.squaredNorm();
        if (prev_dist2 - dist2 <=
            std::numeric_limits<real>::epsilon() * prev_dist2) {
            return false;
        }
    } while (!gjk.IsFullSimplex() && dist2 > tolerance * gjk.GetMaxDist2());

    return CalcPenetrationDepth(gjk, support1, support2, out_penetration,
                                out_p1, out_p2);
}

}  // namespace toy_physics
