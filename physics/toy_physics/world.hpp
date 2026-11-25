#pragma once

#include "common/tight_pool.hpp"
#include "lowlevel/algorithm.hpp"
#include "rigid_sim/rigidbody.hpp"
#include "rigid_sim/solver.hpp"
#include "toy_physics/common/noncopyable.hpp"
#include "toy_physics/geometry/box.hpp"
#include <memory>
#include <span>
#include <unordered_set>
#include <vector>

namespace toy_physics {

class Shape;
class BroadPhase;
class ConvexFactory;
class ShapeFactory;
class DynamicRigidBody;
class StaticRigidBody;
class Foundation;

class World : public StorageElem<World>, NonCopyable {
public:
    friend class RigidActor;

    struct Settings {
        RigidSolver::Settings m_solver_settings;

        Vector3 m_gravity{-Vector3::UnitY() * 9.8};
    };

    explicit World(const Settings&, Foundation&);
    ~World();

    void AddRigidActor(RigidActor&);
    void RemoveRigidActor(RigidActor&);

    size_t Overlap(const Geometry&, const Transform& transform,
                   std::span<Geometry*> out_geoms);
    size_t Raycast(Vector3 origin, Vector3 dir, real len,
                   std::span<RaycastHit> out_hits,
                   QueryFlags = QueryFlag::Default);
    size_t Sweep(const Geometry&, const Transform&, Vector3 dir, real len,
                 std::span<SweepHit> out_hits, QueryFlags = QueryFlag::Default);

    [[nodiscard]] const BroadPhase& GetSceneQueryDynamicBroadPhase() const;
    [[nodiscard]] const BroadPhase& GetSceneQueryStaticBroadPhase() const;
    [[nodiscard]] const BroadPhase& GetSimulationBroadPhase() const;

    void ForceRebuildSceneQueryBroadPhase();

    void Simulate(real dt);

    void ChangeSolverSettings(const RigidSolver::Settings&);
    [[nodiscard]] const RigidSolver::Settings& GetSolverSettings() const;

private:
    void refreshSceneQueryBroadPhase(bool force = false);

    void addShape(Shape& shape);
    void removeShape(Shape& shape);

    Foundation& m_foundation;
    Vector3 m_gravity;

    TightPool<RigidActor*> m_actors;

    // for query
    std::unique_ptr<BroadPhase> m_sq_dyn_bp;
    std::unique_ptr<BroadPhase> m_sq_static_bp;

    // for simulation
    std::unique_ptr<BroadPhase> m_sim_bp;

    bool m_sq_dirty{true};

    // shapes run out fat AABB need reinsert into BroadPhase
    std::vector<Shape*> m_shapes_to_update;
    size_t m_bp_update_count{0};

    std::unique_ptr<RigidSolver> m_rigid_solver;
};

class WorldFactory : public Storage<World> {};

}  // namespace toy_physics
