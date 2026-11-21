#pragma once

#include "common/tight_pool.hpp"
#include "rigid_sim/rigidbody.hpp"
#include "rigid_sim/solver.hpp"
#include "toy_physics/common/noncopyable.hpp"
#include "toy_physics/geometry/box.hpp"
#include <memory>
#include <unordered_set>

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
    struct Settings {
        RigidSolver::Settings m_solver_settings;

        Vector3 m_gravity{-Vector3::UnitY() * 9.8};
    };

    explicit World(const Settings&, Foundation&);
    ~World();

    void AddRigidActor(RigidActor&);
    void RemoveRigidActor(RigidActor&);

    void AddShape(Shape& shape) const;

    [[nodiscard]] const BroadPhase& GetSceneQueryBroadPhase() const;
    void ForceRebuildSceneQueryBroadPhase();

    void Simulate(real dt);

    void ChangeSolverSettings(const RigidSolver::Settings&);
    [[nodiscard]] const RigidSolver::Settings& GetSolverSettings() const;

private:
    Foundation& m_foundation;
    Vector3 m_gravity;

    TightPool<RigidActor*> m_actors;

    std::unique_ptr<BroadPhase> m_sq_broad_phase;
    std::unique_ptr<RigidSolver> m_rigid_solver;
};

class WorldFactory : public Storage<World> {};

}  // namespace toy_physics
