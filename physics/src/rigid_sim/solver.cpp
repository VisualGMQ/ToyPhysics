#include "toy_physics/rigid_sim/solver.hpp"

#include "toy_physics/rigid_sim/manifold.hpp"
#include "toy_physics/rigid_sim/rigidbody.hpp"
#include "toy_physics/rigid_sim/shape.hpp"

#include <utility>

namespace toy_physics {

RigidSolver::RigidSolver(const Settings& settings) : m_settings{settings} {}

void RigidSolver::Solve(std::span<RigidActor*> actors, real dt) {
    // Step1: integrate velocities
    integrateVelocities(actors, dt, m_settings.m_use_gyro);

    // Step2: detect contact and generate manifolds
    m_contacts.clear();
    detectContacts(actors, dt);

    // Step3: solve constraints
    solveVelocityConstraints();

    // Step4: integrate transform
    integrateTransforms(actors, dt);
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

void RigidSolver::detectContacts(std::span<RigidActor*> actors, real dt) {
    for (size_t i = 0; i < actors.size(); i++) {
        for (size_t j = i + 1; j < actors.size(); j++) {
            RigidActor* actor1 = actors[i];
            RigidActor* actor2 = actors[j];
            if (actor1->GetType() == RigidActor::Type::Static &&
                actor2->GetType() == RigidActor::Type::Static) {
                continue;
            }

            for (Shape* shape1 : actor1->GetShapes()) {
                for (Shape* shape2 : actor2->GetShapes()) {
                    if (!shape1->GetBoundingBox().IsIntersect(
                            shape2->GetBoundingBox())) {
                        continue;
                    }

                    Manifold manifold;
                    if (!ComputeManifold(*shape1, *shape2, &manifold)) {
                        continue;
                    }

                    if (manifold.m_contact_num == 0) {
                        continue;
                    }

                    convertManifoldToSolverContacts(manifold, dt);
                }
            }
        }
    }
}

void RigidSolver::convertManifoldToSolverContacts(const Manifold& manifold,
                                                  real dt) {
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

    for (int i = 0; i < manifold.m_contact_num; i++) {
        SolverContact contact;
        contact.m_body1 = actor1;
        contact.m_body2 = actor2;

        contact.m_dyn1 = dyn1;
        contact.m_dyn2 = dyn2;

        contact.m_friction = friction;

        Vector3 point1 = manifold.m_contacts1[i];
        Vector3 point2 = manifold.m_contacts2[i];

        contact.m_r1 = point1 - center_of_mass1;
        contact.m_r2 = point2 - center_of_mass2;

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

        // penetration by using baumgarte
        const real penetration =
            std::max<real>(contact.m_depth - m_settings.m_penetration_slop, 0);

        contact.m_bias = std::min(m_settings.m_baumgarte * penetration / dt,
                                  m_settings.m_max_correction_velocity);

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

    // -(J_n u + C_n / dt) / JM^{-1}J^T
    const real delta = (contact.m_bias - vn) * contact.m_normal_mass;

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

}  // namespace toy_physics