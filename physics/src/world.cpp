#include "toy_physics/world.hpp"
#include "toy_physics/bp/bvh_incresement.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/lowlevel/algorithm.hpp"
#include "toy_physics/lowlevel/bv.hpp"
#include "toy_physics/lowlevel/geom_support.hpp"
#include "toy_physics/rigid_sim/manifold.hpp"
#include "toy_physics/rigid_sim/shape.hpp"
#include "toy_physics/rigid_sim/solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace toy_physics {

namespace {

AABB makeGeometryAABB(const Geometry& geom, const Transform& trans) {
    AABB box = BuildBV(geom, trans.m_rotation);
    box.m_min += trans.m_position;
    box.m_max += trans.m_position;
    return box;
}

AABB makeSegmentAABB(Vector3 start, Vector3 end) {
    AABB box;
    box.m_min = start.cwiseMin(end);
    box.m_max = start.cwiseMax(end);

    const Vector3 padding = Vector3::Constant(kREAL_EPSILON * 16);
    box.m_min -= padding;
    box.m_max += padding;
    return box;
}

AABB makeSweptAABB(const AABB& start, Vector3 offset) {
    AABB box;
    box.m_min = start.m_min.cwiseMin(start.m_min + offset);
    box.m_max = start.m_max.cwiseMax(start.m_max + offset);
    return box;
}

std::array<Vector3, 3> rotatedAxes(const BoxGeometry& box,
                                   const Quaternion& rotation) {
    const Matrix33 rot = rotation.toRotationMatrix();
    return {rot * box.m_axis[0], rot * box.m_axis[1], rot * box.m_axis[2]};
}

uint16_t raycastGeometry(const Geometry& geom, const Transform& trans,
                         Vector3 origin, Vector3 dir, real len,
                         std::span<RaycastHit> out_hits, QueryFlags flags) {
    switch (geom.GetType()) {
        case Geometry::Type::Box: {
            const auto& box = static_cast<const BoxGeometry&>(geom);
            return RaycastOBB(origin, dir, len, trans.m_position,
                              rotatedAxes(box, trans.m_rotation),
                              box.m_half_extent, out_hits, flags);
        }
        case Geometry::Type::Sphere: {
            const auto& sphere = static_cast<const SphereGeometry&>(geom);
            return RaycastSphere(origin, dir, len, trans.m_position,
                                 sphere.m_radius, out_hits, flags);
        }
        case Geometry::Type::Capsule: {
            const auto& capsule = static_cast<const CapsuleGeometry&>(geom);
            const Vector3 axis =
                (trans.m_rotation * Vector3::UnitY()).normalized();
            return RaycastCapsule(origin, dir, len, trans.m_position, axis,
                                  capsule.m_half_height, capsule.m_radius,
                                  out_hits, flags);
        }
        case Geometry::Type::Cylinder: {
            const auto& cylinder = static_cast<const CylinderGeometry&>(geom);
            const Vector3 axis =
                (trans.m_rotation * Vector3::UnitY()).normalized();
            return RaycastCylinder(origin, dir, len, trans.m_position, axis,
                                   cylinder.m_half_height, cylinder.m_radius,
                                   out_hits, flags);
        }
        case Geometry::Type::ConvexHull: {
            const auto* hull = geom.AsConvexHull();
            if (hull == nullptr || hull->GetConvex() == nullptr ||
                out_hits.empty()) {
                return 0;
            }

            const PointSupportFunction point{origin};
            const GeomSupportFunction target{geom, trans};

            real t = 0;
            Vector3 offset = Vector3::Zero();
            Vector3 normal = Vector3::Zero();
            Vector3 witness1 = Vector3::Zero();
            Vector3 witness2 = Vector3::Zero();

            const int result =
                Sweep(point, target, dir, len, true, &t, &offset,
                                   &normal, &witness1, &witness2);
            if (result == 0) {
                return 0;
            }

            RaycastHit hit;
            hit.m_initial_overlap = result < 0;
            hit.m_dist = result < 0 ? real(0) : t * len;
            hit.m_hit = result < 0 ? origin : origin + dir * hit.m_dist;
            if (flags.Has(QueryFlag::Normal) && result > 0) {
                hit.m_normal = normal;
            }
            out_hits[0] = hit;
            return 1;
        }
        default:
            return 0;
    }
}

bool sweepGeometry(const Geometry& moving, const Transform& moving_trans,
                   const Geometry& target, const Transform& target_trans,
                   Vector3 dir, real len, QueryFlags flags, SweepHit* out_hit) {
    const GeomSupportFunction moving_support{moving, moving_trans};
    const GeomSupportFunction target_support{target, target_trans};

    real t = 0;
    Vector3 offset = Vector3::Zero();
    Vector3 normal = Vector3::Zero();
    Vector3 witness1 = Vector3::Zero();
    Vector3 witness2 = Vector3::Zero();

    const int result =
        toy_physics::Sweep(moving_support, target_support, dir, len, true, &t,
                           &offset, &normal, &witness1, &witness2);
    if (result == 0) {
        return false;
    }

    SweepHit hit;
    hit.m_initial_overlap = result < 0;
    hit.m_dist = result < 0 ? real(0) : t * len;

    if (result < 0) {
        hit.m_hit = witness1;
    } else {
        hit.m_normal = normal;

        const Transform at_hit{moving_trans.m_position + offset,
                               moving_trans.m_rotation};
        const GeomSupportFunction moving_at_hit{moving, at_hit};

        constexpr real kHitPointMaxDist2 = real(0.01);
        Vector3 point1 = Vector3::Zero();
        Vector3 point2 = Vector3::Zero();
        const real dist2 =
            CalcClosestPoints(moving_at_hit, target_support, point1, point2,
                              kHitPointMaxDist2, real(1e-4));
        if (dist2 <= kHitPointMaxDist2) {
            hit.m_hit = point1;
        } else {
            hit.m_hit = getGeomSupport(moving, at_hit, -normal);
        }
    }

    if (!flags.Has(QueryFlag::Distance)) {
        hit.m_dist = 0;
    }

    *out_hit = hit;
    return true;
}

}  // namespace

World::World(const Settings& settings, Foundation& foundation)
    : m_gravity{settings.m_gravity}, m_foundation{foundation} {
    m_sq_dyn_bp = std::make_unique<AABBBroadPhase<BVHBuildPolicy::Incremental>>(
        BroadPhase::IndexSlot::SceneQuery);
    m_sq_static_bp =
        std::make_unique<AABBBroadPhase<BVHBuildPolicy::Incremental>>(
            BroadPhase::IndexSlot::SceneQuery);
    m_sim_bp = std::make_unique<AABBBroadPhase<BVHBuildPolicy::Incremental>>(
        BroadPhase::IndexSlot::Simulation);

    m_rigid_solver = std::make_unique<RigidSolver>(settings.m_solver_settings);
}

World::~World() {
    for (RigidActor* actor : m_actors.GetPayloads<RigidActor*>()) {
        actor->m_world = nullptr;
    }

    m_rigid_solver.reset();
    m_sim_bp.reset();
    m_sq_static_bp.reset();
    m_sq_dyn_bp.reset();
}

void World::AddRigidActor(RigidActor& actor) {
    auto id = m_actors.Add(&actor);
    actor.m_tight_id = id;
    actor.m_world = this;

    const std::span<Shape*> shapes = actor.GetShapes();
    if (!shapes.empty()) {
        m_sim_bp->AddObjects(shapes);

        BroadPhase& sq = actor.GetType() == RigidActor::Type::Dynamic
                             ? *m_sq_dyn_bp
                             : *m_sq_static_bp;
        sq.AddObjects(shapes);

        for (Shape* shape : shapes) {
            shape->clearBroadPhaseDirty();
        }
    }

    m_sq_dirty = true;
}

void World::RemoveRigidActor(RigidActor& actor) {
    const std::span<Shape*> shapes = actor.GetShapes();
    if (!shapes.empty()) {
        m_sim_bp->RemoveObjects(shapes);

        BroadPhase& sq = actor.GetType() == RigidActor::Type::Dynamic
                             ? *m_sq_dyn_bp
                             : *m_sq_static_bp;
        sq.RemoveObjects(shapes);
    }

    m_actors.Remove(actor.m_tight_id);
    actor.m_world = nullptr;
    m_sq_dirty = true;
}

void World::addShape(Shape& shape) {
    Shape* shapes[] = {&shape};

    m_sim_bp->AddObjects(std::span<Shape*>{shapes});

    if (const RigidActor* owner = shape.GetOwner();
        owner != nullptr && owner->GetType() == RigidActor::Type::Dynamic) {
        m_sq_dyn_bp->AddObjects(std::span<Shape*>{shapes});
    } else {
        m_sq_static_bp->AddObjects(std::span<Shape*>{shapes});
    }

    shape.clearBroadPhaseDirty();
    m_sq_dirty = true;
}

void World::removeShape(Shape& shape) {
    Shape* shapes[] = {&shape};

    m_sim_bp->RemoveObjects(std::span<Shape*>{shapes});

    if (const RigidActor* owner = shape.GetOwner();
        owner != nullptr && owner->GetType() == RigidActor::Type::Dynamic) {
        m_sq_dyn_bp->RemoveObjects(std::span<Shape*>{shapes});
    } else {
        m_sq_static_bp->RemoveObjects(std::span<Shape*>{shapes});
    }

    m_sq_dirty = true;
}

const BroadPhase& World::GetSceneQueryDynamicBroadPhase() const {
    return *m_sq_dyn_bp;
}

const BroadPhase& World::GetSceneQueryStaticBroadPhase() const {
    return *m_sq_static_bp;
}

const BroadPhase& World::GetSimulationBroadPhase() const {
    return *m_sim_bp;
}

void World::refreshSceneQueryBroadPhase(bool force) {
    const auto& actors = m_actors.GetPayloads<RigidActor*>();

    m_shapes_to_update.clear();

    for (int pass = 0; pass < 2; ++pass) {
        BroadPhase& sq = pass == 0 ? *m_sq_dyn_bp : *m_sq_static_bp;

        for (RigidActor* actor : actors) {
            const bool is_dynamic =
                actor->GetType() == RigidActor::Type::Dynamic;
            if (is_dynamic != (pass == 0)) {
                continue;
            }

            for (Shape* shape : actor->GetShapes()) {
                if (force) {
                    shape->consumeSceneQueryDirty();
                    m_shapes_to_update.push_back(shape);
                } else if (shape->consumeSceneQueryDirty()) {
                    m_shapes_to_update.push_back(shape);
                }
            }
        }

        if (!m_shapes_to_update.empty()) {
            sq.AddObjects(std::span<Shape*>{m_shapes_to_update});
            m_shapes_to_update.clear();
        }
    }

    m_sq_dyn_bp->ApplyModify();
    m_sq_static_bp->ApplyModify();
}

void World::ForceRebuildSceneQueryBroadPhase() {
    refreshSceneQueryBroadPhase(true);
    m_sq_dirty = false;
}

size_t World::Overlap(const Geometry& geom, const Transform& transform,
                      std::span<Geometry*> out_geoms) {
    if (out_geoms.empty()) {
        return 0;
    }

    if (m_sq_dirty) {
        refreshSceneQueryBroadPhase();
        m_sq_dirty = false;
    }

    const AABB query_aabb = makeGeometryAABB(geom, transform);
    size_t count = 0;

    const auto filter = [&](Shape* shape) {
        if (count >= out_geoms.size()) {
            return;
        }

        if (!ComputePenetration(geom, transform, *shape->GetGeometry(),
                                shape->GetTransform(), nullptr, nullptr,
                                nullptr)) {
            return;
        }

        out_geoms[count] = shape->GetGeometry();
        ++count;
    };

    m_sq_dyn_bp->DoFilter(query_aabb, filter);
    m_sq_static_bp->DoFilter(query_aabb, filter);

    return count;
}

size_t World::Raycast(Vector3 origin, Vector3 dir, real len,
                      std::span<RaycastHit> out_hits, QueryFlags flags) {
    if (out_hits.empty() || len <= kREAL_EPSILON) {
        return 0;
    }

    const Vector3 unit = dir.normalized();
    if (!unit.allFinite()) {
        return 0;
    }

    if (m_sq_dirty) {
        refreshSceneQueryBroadPhase();
        m_sq_dirty = false;
    }

    const AABB query_aabb = makeSegmentAABB(origin, origin + unit * len);

    const QueryFlags internal_flags = flags | QueryFlag::Distance;

    std::vector<RaycastHit> hits;
    const auto filter = [&](Shape* shape) {
        std::array<RaycastHit, 8> local{};
        const uint16_t hit_count =
            raycastGeometry(*shape->GetGeometry(), shape->GetTransform(),
                            origin, unit, len, local, internal_flags);
        for (uint16_t i = 0; i < hit_count; ++i) {
            hits.push_back(local[i]);
        }
    };

    m_sq_dyn_bp->DoFilter(query_aabb, filter);
    m_sq_static_bp->DoFilter(query_aabb, filter);

    std::ranges::sort(hits, {}, &RaycastHit::m_dist);

    const size_t count = std::min(out_hits.size(), hits.size());
    for (size_t i = 0; i < count; ++i) {
        out_hits[i] = hits[i];
        if (!flags.Has(QueryFlag::Distance)) {
            out_hits[i].m_dist = 0;
        }
    }

    return count;
}

size_t World::Sweep(const Geometry& geom, const Transform& transform,
                    Vector3 dir, real len, std::span<SweepHit> out_hits,
                    QueryFlags flags) {
    if (out_hits.empty() || len <= kREAL_EPSILON) {
        return 0;
    }

    const Vector3 unit = dir.normalized();
    if (!unit.allFinite()) {
        return 0;
    }

    if (m_sq_dirty) {
        refreshSceneQueryBroadPhase();
        m_sq_dirty = false;
    }

    const AABB query_aabb =
        makeSweptAABB(makeGeometryAABB(geom, transform), unit * len);

    std::vector<SweepHit> hits;
    const auto filter = [&](Shape* shape) {
        SweepHit hit;
        if (sweepGeometry(geom, transform, *shape->GetGeometry(),
                          shape->GetTransform(), unit, len, flags, &hit)) {
            hits.push_back(hit);
        }
    };

    m_sq_dyn_bp->DoFilter(query_aabb, filter);
    m_sq_static_bp->DoFilter(query_aabb, filter);

    std::ranges::sort(hits, {}, &SweepHit::m_dist);

    const size_t count = std::min(out_hits.size(), hits.size());
    for (size_t i = 0; i < count; ++i) {
        out_hits[i] = hits[i];
    }

    return count;
}

void World::Simulate(real dt) {
    TOY_ASSERT(dt != 0);

    auto& actors = m_actors.GetPayloads<RigidActor*>();

    m_shapes_to_update.clear();
    for (RigidActor* actor : actors) {
        for (Shape* shape : actor->GetShapes()) {
            if (shape->consumeSimulationDirty()) {
                m_shapes_to_update.push_back(shape);
            }
        }
    }

    if (!m_shapes_to_update.empty()) {
        m_sim_bp->AddObjects(std::span<Shape*>{m_shapes_to_update});
    }
    m_bp_update_count = m_shapes_to_update.size();
    m_sim_bp->ApplyModify();

    for (auto actor : actors) {
        if (auto dyn = actor->AsDynamic()) {
            Vector3 vel = dyn->GetVelocity();
            dyn->SetVelocity(vel + m_gravity * dt);
        }
    }

    m_rigid_solver->Solve(actors, dt, *m_sim_bp);

    m_sq_dirty = true;
}

void World::ChangeSolverSettings(const RigidSolver::Settings& settings) {
    m_rigid_solver->ChangeSettings(settings);
}

const RigidSolver::Settings& World::GetSolverSettings() const {
    return m_rigid_solver->GetSettings();
}

}  // namespace toy_physics
