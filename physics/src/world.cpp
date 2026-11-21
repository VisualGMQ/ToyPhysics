#include "toy_physics/world.hpp"
#include "toy_physics/bp/bvh_topdown.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/rigid_sim/shape.hpp"
#include "toy_physics/rigid_sim/solver.hpp"

namespace toy_physics {

World::World(const Settings& settings, Foundation& foundation)
    : m_gravity{settings.m_gravity}, m_foundation{foundation} {
    m_sq_broad_phase =
        std::make_unique<AABBBroadPhase<BVHBuildPolicy::TopDown>>();
    m_rigid_solver = std::make_unique<RigidSolver>(settings.m_solver_settings);
}

World::~World() {
    m_rigid_solver.reset();
    m_sq_broad_phase.reset();
}

void World::AddRigidActor(RigidActor& actor) {
    auto id = m_actors.Add(&actor);
    actor.m_tight_id = id;
}

void World::RemoveRigidActor(RigidActor& actor) {
    m_actors.Remove(actor.m_tight_id);
}

void World::AddShape(Shape& shape) const {
    m_sq_broad_phase->AddObjects(std::span<Shape>{&shape, 1});
}

const BroadPhase& World::GetSceneQueryBroadPhase() const {
    return *m_sq_broad_phase;
}

void World::ForceRebuildSceneQueryBroadPhase() {
    m_sq_broad_phase->ApplyModify();
}

void World::Simulate(real dt) {
    TOY_ASSERT(dt != 0);

    auto& actors = m_actors.GetPayloads<RigidActor*>();
    for (auto actor : actors) {
        if (auto dyn = actor->AsDynamic()) {
            Vector3 vel = dyn->GetVelocity();
            dyn->SetVelocity(vel + m_gravity * dt);
        }
    }

    m_rigid_solver->Solve(actors, dt);
}

void World::ChangeSolverSettings(const RigidSolver::Settings& settings) {
    m_rigid_solver->ChangeSettings(settings);
}

const RigidSolver::Settings& World::GetSolverSettings() const {
    return m_rigid_solver->GetSettings();
}

}  // namespace toy_physics
