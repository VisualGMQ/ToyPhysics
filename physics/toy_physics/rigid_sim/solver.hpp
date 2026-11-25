#pragma once
#include "toy_physics/rigid_sim/rigidbody.hpp"
#include "toy_physics/geometry/geometry.hpp"
#include "toy_physics/common/unordered_pair.hpp"

#include <array>
#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

namespace toy_physics {

class RigidActor;
class DynamicRigidBody;
class Manifold;
class Shape;
class BroadPhase;

struct SolverContact {
    RigidActor* m_body1{};
    RigidActor* m_body2{};

    // body1/body2 alias
    DynamicRigidBody* m_dyn1{};
    DynamicRigidBody* m_dyn2{};

    // shape pair (for warm starting cache)
    const Shape* m_shape1{};
    const Shape* m_shape2{};

    // contact relate to center of mass
    Vector3 m_r1{Vector3::Zero()};
    Vector3 m_r2{Vector3::Zero()};

    // contact in body local space (for warm starting / contact matching)
    Vector3 m_local1{Vector3::Zero()};
    Vector3 m_local2{Vector3::Zero()};

    Vector3 m_normal{Vector3::Zero()};
    Vector3 m_tangent1{Vector3::Zero()};
    Vector3 m_tangent2{Vector3::Zero()};

    real m_depth{0};  // penetration depth
    real m_friction{0};

    // 1 / JM^{-1}J^T diag elems
    real m_normal_mass{0};
    std::array<real, 2> m_tangent_masses{0};

    // LCP unknown variable: p_n, p_f
    real m_normal_impulse{0};
    std::array<real, 2> m_tangent_impulses{0};

    // -C_n/dt
    real m_bias{0};
};

class RigidSolver {
public:
    struct Settings {
        bool m_use_gyro = false;
        bool m_use_warm_start = true;

        enum FrictionPolicy {
            Max,
            Average,
            Sqrt,
        } m_friction_policy = Average;

        uint32_t m_iter_num = 10;
        real m_max_correction_velocity = 1;

        // baumgarte stability in velocity
        real m_baumgarte{0};
        real m_penetration_slop{0};
    };

    explicit RigidSolver(const Settings&);
    void Solve(std::span<RigidActor*>, real dt,
               const BroadPhase& sim_broad_phase);

    [[nodiscard]] const Settings& GetSettings() const;
    void ChangeSettings(const Settings&);

private:
    using ShapePair = UnorderedPair<const Shape*, const Shape*>;

    struct CachedPoint {
        Vector3 m_local1{Vector3::Zero()};
        Vector3 m_local2{Vector3::Zero()};
        Vector3 m_normal{Vector3::Zero()};

        real m_normal_impulse{0};
        std::array<real, 2> m_tangent_impulses{0};
    };

    struct CachedManifold {
        const Shape* m_shape1{};
        std::vector<CachedPoint> m_points;
    };

    [[nodiscard]] int findMatch(
        const CachedManifold& cached, const SolverContact& contact,
        bool flipped,
        std::array<bool, 4>& used) const;

    void warmStartContacts();
    void storeImpulses();

    Settings m_settings;
    std::vector<SolverContact> m_contacts;

    std::unordered_map<ShapePair, CachedManifold> m_cache;

    void integrateVelocities(std::span<RigidActor*>, real dt, bool use_gyro) const;
    void detectContacts(std::span<RigidActor*>, real dt,
                        const BroadPhase& sim_broad_phase);
    void convertManifoldToSolverContacts(const Manifold& manifold, real dt);
    [[nodiscard]] real computeEffectiveMass(const SolverContact&, Vector3 tangent) const;
    void solveVelocityConstraints();
    void integrateTransforms(std::span<RigidActor*>, real dt);

    [[nodiscard]] Vector3 velocityAt(DynamicRigidBody* body, const Vector3& r) const;
    [[nodiscard]] Vector3 relativeVelocity(const SolverContact& c) const;

    void solveNormal(SolverContact&);
    void solveFriction(SolverContact&);
    void applyContactImpulse(SolverContact&, const Vector3& impulse);
};

}  // namespace toy_physics