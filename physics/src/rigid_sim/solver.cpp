#include "toy_physics/rigid_sim/solver.hpp"

#include "toy_physics/bp/bp.hpp"
#include "toy_physics/rigid_sim/manifold.hpp"
#include "toy_physics/rigid_sim/rigidbody.hpp"
#include "toy_physics/rigid_sim/shape.hpp"

#include <algorithm>
#include <functional>
#include <utility>

namespace toy_physics {

RigidSolver::RigidSolver(const Settings& settings) : m_settings{settings} {}

void RigidSolver::Solve(std::span<RigidActor*> actors, real dt,
                        const BroadPhase& sim_broad_phase) {
    // Step1: integrate velocities
    integrateVelocities(actors, dt, m_settings.m_use_gyro);

    // Step2: detect contact and generate manifolds
    m_contacts.clear();
    detectContacts(actors, sim_broad_phase);

    // Step3: restore impulses from last frame and apply them
    if (m_settings.m_use_warm_start) {
        // impulse's unit is N·s, so different time will make it larger/smaller.
        // we need compute the ratio dt/prev_dt, then apply on old impulse.
        // the maximim ratio is limited to 4, to forbid a large impulse
        // which the solver can't correct it then make body move so fast
        const real impulse_ratio =
            m_prev_dt > 0 ? std::min<real>(dt / m_prev_dt, 4) : 1;
        warmStartContacts(impulse_ratio);
    }

    // Step4: solve constraints
    solveVelocityConstraints();

    // Step5: store impulses for next frame
    if (m_settings.m_use_warm_start) {
        storeImpulses();
    } else {
        m_cache.clear();
    }

    // Step6: integrate transform
    integrateTransforms(actors, dt);

    // Step7: position correction using split impulse
    if (m_settings.m_use_split_impulse) {
        solvePositionConstraints(actors, dt);
    }

    m_prev_dt = dt;
}

const RigidSolver::Settings& RigidSolver::GetSettings() const {
    return m_settings;
}

void RigidSolver::ChangeSettings(const Settings& settings) {
    m_settings = settings;
}

void RigidSolver::integrateVelocities(std::span<RigidActor*> actors, real dt,
                                      bool use_gyro) const {
    for (auto actor : actors) {
        if (actor->GetType() != RigidActor::Type::Dynamic) {
            continue;
        }

        auto body = actor->AsDynamic();
        Vector3 velocity = body->GetVelocity();
        Vector3 angular_velocity = body->GetAngularVelocity();

        velocity +=
            body->GetForce() * body->GetMassProperties().m_inv_mass * dt;
        angular_velocity +=
            body->MultiplyInvInertiaWorld(body->GetTorque() * dt);

        if (use_gyro) {
            auto& mp = body->GetMassProperties();
            Quaternion q =
                body->GetTransform().m_rotation * mp.m_inertia_rotation;

            Vector3 local_w = q.conjugate() * angular_velocity;

            Vector3 local_inertia;
            for (int i = 0; i < 3; i++) {
                real inv_inertia = mp.m_inv_inertia[i];
                local_inertia[i] = inv_inertia > 0 ? 1.0 / inv_inertia : 0;
            }

            Vector3 local_L = local_inertia.cwiseProduct(local_w);
            Vector3 new_L = local_L - local_w.cross(local_L) * dt;

            real old_len2 = local_L.squaredNorm();
            real new_len2 = new_L.squaredNorm();
            if (new_len2 > 0) {
                new_L *= std::sqrt(old_len2 / new_len2);
            }

            angular_velocity = q * mp.m_inv_inertia.cwiseProduct(new_L);
        }

        body->SetVelocity(velocity);
        body->SetAngularVelocity(angular_velocity);

        body->ClearForces();
    }
}

void RigidSolver::detectContacts(std::span<RigidActor*> actors,
                                 const BroadPhase& sim_broad_phase) {
    for (RigidActor* actor1 : actors) {
        const std::span<Shape*> shapes1 = actor1->GetShapes();
        if (shapes1.empty()) {
            continue;
        }

        sim_broad_phase.DoFilter(actor1->GetBoundingBox(), [&](Shape* shape2) {
            RigidActor* actor2 = shape2->GetOwner();
            if (actor2 == nullptr || actor2 == actor1) {
                return;
            }

            if (actor2->GetID() <= actor1->GetID()) {
                return;
            }

            if (actor1->GetType() == RigidActor::Type::Static &&
                actor2->GetType() == RigidActor::Type::Static) {
                return;
            }

            for (Shape* shape1 : shapes1) {
                if (!shape1->GetFatBoundingBox().IsIntersect(
                        shape2->GetFatBoundingBox())) {
                    continue;
                }

                Manifold manifold;
                if (!ComputeManifold(*shape1, *shape2, &manifold)) {
                    continue;
                }

                if (manifold.m_contact_num == 0) {
                    continue;
                }

                convertManifoldToSolverContacts(manifold);
            }
        });
    }
}

void RigidSolver::convertManifoldToSolverContacts(const Manifold& manifold) {
    real friction1 = manifold.m_shape1->GetFriction();
    real friction2 = manifold.m_shape2->GetFriction();
    real friction = 0;
    switch (m_settings.m_friction_policy) {
        case Settings::Max:
            friction = std::max(friction1, friction2);
            break;
        case Settings::Average:
            friction = (friction1 + friction2) * 0.5;
            break;
        case Settings::Sqrt:
            friction = std::sqrt(friction1 * friction2);
            break;
    }

    RigidActor* actor1 = const_cast<RigidActor*>(manifold.m_shape1->GetOwner());
    RigidActor* actor2 = const_cast<RigidActor*>(manifold.m_shape2->GetOwner());

    DynamicRigidBody* dyn1 = actor1->AsDynamic();
    DynamicRigidBody* dyn2 = actor2->AsDynamic();

    Vector3 center_of_mass1 = dyn1 ? dyn1->GetCenterOfMassPosition()
                                   : actor1->GetTransform().m_position;
    Vector3 center_of_mass2 = dyn2 ? dyn2->GetCenterOfMassPosition()
                                   : actor2->GetTransform().m_position;

    const Transform& body_trans1 = actor1->GetTransform();
    const Transform& body_trans2 = actor2->GetTransform();

    for (int i = 0; i < manifold.m_contact_num; i++) {
        SolverContact contact;
        contact.m_body1 = actor1;
        contact.m_body2 = actor2;

        contact.m_dyn1 = dyn1;
        contact.m_dyn2 = dyn2;

        contact.m_shape1 = manifold.m_shape1;
        contact.m_shape2 = manifold.m_shape2;

        contact.m_friction = friction;

        Vector3 point1 = manifold.m_contacts1[i];
        Vector3 point2 = manifold.m_contacts2[i];

        contact.m_r1 = point1 - center_of_mass1;
        contact.m_r2 = point2 - center_of_mass2;

        contact.m_local1 = body_trans1.m_rotation.conjugate() *
                           (point1 - body_trans1.m_position);
        contact.m_local2 = body_trans2.m_rotation.conjugate() *
                           (point2 - body_trans2.m_position);

        contact.m_normal = manifold.m_separate_normal.normalized();
        contact.m_depth = std::max<real>(manifold.m_depth[i], 0);

        contact.m_tangent1 = contact.m_normal.unitOrthogonal();
        contact.m_tangent2 =
            contact.m_normal.cross(contact.m_tangent1).normalized();

        contact.m_normal_mass = computeEffectiveMass(contact, contact.m_normal);
        contact.m_tangent_masses[0] =
            computeEffectiveMass(contact, contact.m_tangent1);
        contact.m_tangent_masses[1] =
            computeEffectiveMass(contact, contact.m_tangent2);

        m_contacts.push_back(contact);
    }
}

real RigidSolver::computeEffectiveMass(const SolverContact& contact,
                                       Vector3 dir) const {
    real k = 0;

    if (contact.m_dyn1) {
        Vector3 rn = contact.m_r1.cross(dir);
        k += contact.m_dyn1->GetMassProperties().m_inv_mass +
             rn.dot(contact.m_dyn1->MultiplyInvInertiaWorld(rn));
    }

    if (contact.m_dyn2) {
        Vector3 rn = contact.m_r2.cross(dir);
        k += contact.m_dyn2->GetMassProperties().m_inv_mass +
             rn.dot(contact.m_dyn2->MultiplyInvInertiaWorld(rn));
    }

    return k > 0 ? 1 / k : 0;
}

void RigidSolver::solveVelocityConstraints() {
    for (uint32_t i = 0; i < m_settings.m_iter_num; i++) {
        for (auto& contact : m_contacts) {
            solveNormal(contact);
            solveFriction(contact);
        }
    }
}

Vector3 RigidSolver::pseudoRelativeVelocity(const SolverContact& c) const {
    const Vector3 v2 = c.m_dyn2
                           ? Vector3(c.m_dyn2->m_pseudo_linear +
                                     c.m_dyn2->m_pseudo_angular.cross(c.m_r2))
                           : Vector3::Zero();

    const Vector3 v1 = c.m_dyn1
                           ? Vector3(c.m_dyn1->m_pseudo_linear +
                                     c.m_dyn1->m_pseudo_angular.cross(c.m_r1))
                           : Vector3::Zero();

    return v2 - v1;
}

void RigidSolver::applyPseudoImpulse(SolverContact& contact,
                                     const Vector3& impulse) {
    if (contact.m_dyn2 != nullptr) {
        contact.m_dyn2->m_pseudo_linear +=
            impulse * contact.m_dyn2->GetMassProperties().m_inv_mass;
        contact.m_dyn2->m_pseudo_angular +=
            contact.m_dyn2->MultiplyInvInertiaWorld(
                contact.m_r2.cross(impulse));
    }

    if (contact.m_dyn1 != nullptr) {
        contact.m_dyn1->m_pseudo_linear -=
            impulse * contact.m_dyn1->GetMassProperties().m_inv_mass;
        contact.m_dyn1->m_pseudo_angular -=
            contact.m_dyn1->MultiplyInvInertiaWorld(
                contact.m_r1.cross(impulse));
    }
}

void RigidSolver::solvePositionConstraints(std::span<RigidActor*> actors,
                                           real dt) {
    const real slop = m_settings.m_penetration_slop;
    const real beta = m_settings.m_position_baumgarte;
    const real max_correction = m_settings.m_max_position_correction;

    // 1. clear all pseudo impulse
    for (RigidActor* actor : actors) {
        DynamicRigidBody* body = actor->AsDynamic();
        if (body == nullptr) {
            continue;
        }
        body->m_pseudo_linear.setZero();
        body->m_pseudo_angular.setZero();
    }

    // 2. re-compute r1/r2 by using integrated position
    for (SolverContact& contact : m_contacts) {
        contact.m_position_lambda = 0;

        const Transform& t1 = contact.m_body1->GetTransform();
        const Transform& t2 = contact.m_body2->GetTransform();

        const Vector3 p1 = t1.m_position + t1.m_rotation * contact.m_local1;
        const Vector3 p2 = t2.m_position + t2.m_rotation * contact.m_local2;

        contact.m_r1 = contact.m_dyn1 ? p1 - contact.m_dyn1->GetCenterOfMassPosition()
                          : p1 - t1.m_position;
        contact.m_r2 = contact.m_dyn2 ? p2 - contact.m_dyn2->GetCenterOfMassPosition()
                          : p2 - t2.m_position;

        contact.m_separation = (p2 - p1).dot(contact.m_normal);
    }

    // 3. do position iter by PGS
    for (uint32_t iter = 0; iter < m_settings.m_position_iter_num; ++iter) {
        for (SolverContact& c : m_contacts) {
            if (c.m_normal_mass <= 0) {
                continue;
            }

            // C_n(q)
            const real depth = -c.m_separation - slop;
            if (depth <= 0) {
                continue;
            }

            // d lambda = (new_vn - old_vn)/JM^{-1}J^T
            const real vn_target =
                std::min(beta * depth / dt, max_correction / dt);

            const real vn = pseudoRelativeVelocity(c).dot(c.m_normal);
            const real delta = (vn_target - vn) * c.m_normal_mass;

            const real next = std::max<real>(0, c.m_position_lambda + delta);
            const real applied = next - c.m_position_lambda;
            c.m_position_lambda = next;

            if (applied != 0) {
                applyPseudoImpulse(c, c.m_normal * applied);
            }
        }
    }

    // 4. apply pseudo impulse to body
    for (RigidActor* actor : actors) {
        DynamicRigidBody* body = actor->AsDynamic();
        if (body == nullptr) {
            continue;
        }

        if (body->m_pseudo_linear.isZero() && body->m_pseudo_angular.isZero()) {
            continue;
        }

        Transform trans = body->GetTransform();
        trans.m_position += body->m_pseudo_linear * dt;

        const Vector3 omega_dt = body->m_pseudo_angular * dt;
        const real angle = omega_dt.norm();
        if (angle > kREAL_EPSILON) {
            const Vector3 axis = omega_dt / angle;
            const Quaternion dq(Eigen::AngleAxis<real>(angle, axis));
            trans.m_rotation = (dq * trans.m_rotation).normalized();
        }

        body->Teleport(trans);

        body->m_pseudo_linear.setZero();
        body->m_pseudo_angular.setZero();
    }
}

void RigidSolver::integrateTransforms(std::span<RigidActor*> actors, real dt) {
    for (auto actor : actors) {
        if (actor->GetType() != RigidActor::Type::Dynamic) {
            continue;
        }

        auto body = actor->AsDynamic();

        Transform trans = body->GetTransform();

        trans.m_position += body->GetVelocity() * dt;

        Vector3 angular_velocity = body->GetAngularVelocity();
        Vector3 omega_dt = angular_velocity * dt;
        real angle = omega_dt.norm();

        if (angle > kREAL_EPSILON) {
            auto axis = omega_dt / angle;
            const Quaternion dq(Eigen::AngleAxis<real>(angle, axis));
            trans.m_rotation = (dq * trans.m_rotation).normalized();
        }

        body->Teleport(trans);
    }
}

Vector3 RigidSolver::velocityAt(DynamicRigidBody* body,
                                const Vector3& r) const {
    if (!body) {
        return Vector3::Zero();
    }

    return body->GetVelocity() + body->GetAngularVelocity().cross(r);
}

Vector3 RigidSolver::relativeVelocity(const SolverContact& c) const {
    return velocityAt(c.m_dyn2, c.m_r2) - velocityAt(c.m_dyn1, c.m_r1);
}

void RigidSolver::solveNormal(SolverContact& contact) {
    if (contact.m_normal_mass <= 0) {
        return;
    }

    // J_n u
    const real vn = relativeVelocity(contact).dot(contact.m_normal);

    // -J_n u / JM^{-1}J^T
    const real delta = -vn * contact.m_normal_mass;

    const real old = contact.m_normal_impulse;

    // new p_n
    const real next = std::max<real>(0, old + delta);
    const real applied = next - old;
    contact.m_normal_impulse = next;

    if (applied == 0) {
        return;
    }

    const Vector3 impulse = contact.m_normal * applied;

    applyContactImpulse(contact, impulse);
}

void RigidSolver::solveFriction(SolverContact& contact) {
    const real max_friction = contact.m_friction * contact.m_normal_impulse;

    for (int i = 0; i < 2; ++i) {
        if (contact.m_tangent_masses[i] <= 0) {
            continue;
        }

        const Vector3& tangent =
            i == 0 ? contact.m_tangent1 : contact.m_tangent2;

        const real vt = relativeVelocity(contact).dot(tangent);

        const real delta = -vt * contact.m_tangent_masses[i];

        const real old_impulse = contact.m_tangent_impulses[i];
        const real new_impulse =
            std::clamp(old_impulse + delta, -max_friction, max_friction);

        const real applied = new_impulse - old_impulse;
        contact.m_tangent_impulses[i] = new_impulse;

        if (applied != 0) {
            applyContactImpulse(contact, tangent * applied);
        }
    }
}

void RigidSolver::applyContactImpulse(SolverContact& contact,
                                      const Vector3& impulse) {
    if (contact.m_dyn2 != nullptr) {
        contact.m_dyn2->ApplyImpulse(impulse, contact.m_r2);
    }
    if (contact.m_dyn1 != nullptr) {
        contact.m_dyn1->ApplyImpulse(-impulse, contact.m_r1);
    }
}

int RigidSolver::findMatch(const CachedManifold& cached,
                           const SolverContact& contact, bool flipped,
                           std::array<bool, 4>& used) const {
    constexpr real kMaxDist2 = 0.01 * 0.01;
    constexpr real kMinNormalDot = 0.9;

    const Vector3 local1 = flipped ? contact.m_local2 : contact.m_local1;
    const Vector3 local2 = flipped ? contact.m_local1 : contact.m_local2;
    const Vector3 normal = flipped ? -contact.m_normal : contact.m_normal;

    int best = -1;
    real best_dist2 = kMaxDist2;

    for (size_t i = 0; i < cached.m_points.size() && i < 4; ++i) {
        if (used[i]) {
            continue;
        }

        const CachedPoint& point = cached.m_points[i];

        if (normal.dot(point.m_normal) < kMinNormalDot) {
            continue;
        }

        const real d1 = (local1 - point.m_local1).squaredNorm();
        const real d2 = (local2 - point.m_local2).squaredNorm();
        const real d = std::max(d1, d2);

        if (d < best_dist2) {
            best_dist2 = d;
            best = static_cast<int>(i);
        }
    }

    if (best >= 0) {
        used[best] = true;
    }

    return best;
}

void RigidSolver::warmStartContacts(real impulse_ratio) {
    std::unordered_map<ShapePair, std::array<bool, 4>> used_points;

    for (SolverContact& contact : m_contacts) {
        const ShapePair key{contact.m_shape1, contact.m_shape2};

        const auto it = m_cache.find(key);
        if (it == m_cache.end()) {
            continue;
        }

        const CachedManifold& cached = it->second;
        const bool flipped = cached.m_shape1 != contact.m_shape1;

        auto [used_it, inserted] = used_points.try_emplace(key);
        auto& used = used_it->second;
        if (inserted) {
            used.fill(false);
        }

        const int match = findMatch(cached, contact, flipped, used);
        if (match < 0) {
            continue;
        }

        const CachedPoint& point = cached.m_points[static_cast<size_t>(match)];

        contact.m_normal_impulse = point.m_normal_impulse * impulse_ratio;
        contact.m_tangent_impulses = point.m_tangent_impulses;
        for (real& tangent_impulse : contact.m_tangent_impulses) {
            tangent_impulse *= impulse_ratio;
        }

        const real max_friction = contact.m_friction * contact.m_normal_impulse;

        for (real& tangent_impulse : contact.m_tangent_impulses) {
            tangent_impulse =
                std::clamp(tangent_impulse, -max_friction, max_friction);
        }

        applyContactImpulse(contact,
                            contact.m_normal * contact.m_normal_impulse);
        applyContactImpulse(contact,
                            contact.m_tangent1 * contact.m_tangent_impulses[0]);
        applyContactImpulse(contact,
                            contact.m_tangent2 * contact.m_tangent_impulses[1]);
    }
}

void RigidSolver::storeImpulses() {
    std::unordered_map<ShapePair, CachedManifold> new_manifolds;

    for (SolverContact& contact : m_contacts) {
        const ShapePair key{contact.m_shape1, contact.m_shape2};

        const auto cache_it = m_cache.find(key);
        const Shape* cached_shape1 =
            (cache_it != m_cache.end() && cache_it->second.m_shape1 != nullptr)
                ? cache_it->second.m_shape1
                : contact.m_shape1;

        const bool flipped = cached_shape1 != contact.m_shape1;

        const Vector3 local1 = flipped ? contact.m_local2 : contact.m_local1;
        const Vector3 local2 = flipped ? contact.m_local1 : contact.m_local2;
        const Vector3 normal = flipped ? -contact.m_normal : contact.m_normal;

        auto [it, inserted] = new_manifolds.try_emplace(key);
        auto& new_manifold = it->second;

        if (inserted) {
            new_manifold.m_shape1 = cached_shape1;
        }

        if (new_manifold.m_points.size() >= 4) {
            continue;
        }

        new_manifold.m_points.push_back(CachedPoint{
            local1,
            local2,
            normal,
            contact.m_normal_impulse,
            contact.m_tangent_impulses,
        });
    }

    m_cache = std::move(new_manifolds);
}

}  // namespace toy_physics