#include "toy_physics/rigid_sim/rigidbody.hpp"

#include "toy_physics/common/check.hpp"
#include "toy_physics/foundation.hpp"
#include "toy_physics/geometry/mass_property.hpp"
#include "toy_physics/rigid_sim/shape.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace toy_physics {

RigidActor::RigidActor(Type type, Foundation& foundation)
    : m_type{type}, m_foundation{foundation} {}

RigidActor::~RigidActor() {
    for (auto shape : m_shapes) {
        m_foundation.DestroyShape(*shape);
    }
}

DynamicRigidBody* RigidActor::AsDynamic() {
    return const_cast<DynamicRigidBody*>(std::as_const(*this).AsDynamic());
}

StaticRigidBody* RigidActor::AsStatic() {
    return const_cast<StaticRigidBody*>(std::as_const(*this).AsStatic());
}

const DynamicRigidBody* RigidActor::AsDynamic() const {
    if (GetType() == Type::Dynamic) {
        return static_cast<const DynamicRigidBody*>(this);
    }
    return nullptr;
}

const StaticRigidBody* RigidActor::AsStatic() const {
    if (GetType() == Type::Dynamic) {
        return static_cast<const StaticRigidBody*>(this);
    }
    return nullptr;
}

RigidActor::Type RigidActor::GetType() const {
    return m_type;
}

void RigidActor::Teleport(const Transform& transform) {
    m_transform = transform;

    for (auto shape : m_shapes) {
        shape->updateByOwnerTransform();
    }
}

const Transform& RigidActor::GetTransform() const {
    return m_transform;
}

std::span<Shape*> RigidActor::GetShapes() {
    return m_shapes;
}

std::span<Shape* const> RigidActor::GetShapes() const {
    return m_shapes;
}

void RigidActor::AttachShape(Shape& shape) {
    m_shapes.push_back(&shape);

    for (auto s : m_shapes) {
        s->attachToActor(this);
    }
}

void RigidActor::RemoveShape(Shape& shape) {
    auto it = std::ranges::find_if(m_shapes,
                                   [&](const Shape* o) { return o == &shape; });

    if (it != m_shapes.end()) {
        RemoveShape(it - m_shapes.begin());
    }
}

void RigidActor::RemoveShape(size_t index) {
    m_shapes.erase(m_shapes.begin() + index);
}

DynamicRigidBody::DynamicRigidBody(Foundation& foundation)
    : RigidActor{RigidActor::Type::Dynamic, foundation} {}

Vector3 DynamicRigidBody::GetVelocity() const {
    return m_velocity;
}

Vector3 DynamicRigidBody::GetAngularVelocity() const {
    return m_angular_velocity;
}

void DynamicRigidBody::SetVelocity(const Vector3& velocity) {
    m_velocity = velocity;
}

void DynamicRigidBody::SetAngularVelocity(const Vector3& angular_velocity) {
    m_angular_velocity = angular_velocity;
}

void DynamicRigidBody::AttachShape(Shape& shape) {
    RigidActor::AttachShape(shape);
    ensureMassProperties();
}

void DynamicRigidBody::RemoveShape(size_t index) {
    RigidActor::RemoveShape(index);
    ensureMassProperties();
}

const RigidBodyMassProperties& DynamicRigidBody::GetMassProperties() const {
    return m_mp;
}

Vector3 DynamicRigidBody::MultiplyInvInertiaWorld(Vector3 v) const {
    Quaternion q = m_transform.m_rotation * m_mp.m_inertia_rotation;

    Vector3 local = q.conjugate() * v;
    local = m_mp.m_inv_inertia.cwiseProduct(local);
    return q * local;
}

Vector3 DynamicRigidBody::GetCenterOfMassPosition() const {
    return m_transform.TransformBy(m_mp.m_center_of_mass);
}

Vector3 DynamicRigidBody::GetVelocityAt(const Vector3& world_point) const {
    const Vector3 r = world_point - GetCenterOfMassPosition();
    return m_velocity + m_angular_velocity.cross(r);
}

void DynamicRigidBody::ApplyImpulse(const Vector3& impulse, const Vector3& r) {
    m_velocity += impulse * m_mp.m_inv_mass;
    m_angular_velocity += MultiplyInvInertiaWorld(r.cross(impulse));
}

void DynamicRigidBody::ApplyForce(const Vector3& force) {
    m_force += force;
}

void DynamicRigidBody::ApplyForceAtPoint(const Vector3& force,
                                         const Vector3& world_point) {
    m_force += force;

    const Vector3 r = world_point - GetCenterOfMassPosition();
    m_torque += r.cross(force);
}

void DynamicRigidBody::ApplyTorque(const Vector3& torque) {
    m_torque += torque;
}

Vector3 DynamicRigidBody::GetForce() const {
    return m_force;
}

Vector3 DynamicRigidBody::GetTorque() const {
    return m_torque;
}

void DynamicRigidBody::ClearForces() {
    m_force.setZero();
    m_torque.setZero();
}

struct ShapeMassProperties {
    Vector3 m_center_of_mass{Vector3::Zero()};
    Matrix33 m_inertia{Matrix33::Identity()};
    real m_mass{0};
};

void DynamicRigidBody::ensureMassProperties() {
    m_mp = ComputeRigidBodyMassProperties(m_shapes);
}

RigidBodyMassProperties ComputeRigidBodyMassProperties(
    std::span<const Shape* const> shapes) {
    std::vector<ShapeMassProperties> shape_mp_list;

    for (auto shape : shapes) {
        auto density = shape->GetDensity();

        // Step1: compute geometry mass properties
        auto mp = ComputeMassProperties(*shape->GetGeometry(), density);

        // Step2: transform inertia & center of mass from geometry space to
        // shape space
        const Transform& shape_local_trans = shape->GetLocalTransform();
        const Matrix33 R = shape_local_trans.m_rotation.toRotationMatrix();
        const Vector3 t = shape_local_trans.m_position;

        ShapeMassProperties shape_mp;
        shape_mp.m_center_of_mass = t + R * mp.m_center_of_mass;

        Matrix33 principal_axes = R * mp.m_principle_axes;

        shape_mp.m_inertia = principal_axes * mp.m_inertia.asDiagonal() *
                             principal_axes.transpose();

        shape_mp.m_mass = mp.m_mass;

        // prepare for remaining steps
        shape_mp_list.push_back(shape_mp);
    }

    // Step3: average center of mass
    Vector3 rigid_center_of_mass = Vector3::Zero();
    real mass_sum = 0;

    for (auto& mp : shape_mp_list) {
        if (mp.m_mass <= 0) {
            LOGW("mass <= 0");
        } else {
            mass_sum += mp.m_mass;
            rigid_center_of_mass += mp.m_center_of_mass * mp.m_mass;
        }
    }

    if (IsRealApproxEq<real>(mass_sum, 0)) {
        // NOTE: can't do other steps, just return!
        return {};
    }

    rigid_center_of_mass /= mass_sum;

    // Step4: using `parallel axis theorem`
    // to move inertia from shape space to rigidbody space
    Matrix33 body_inertia = Matrix33::Zero();
    for (auto& mp : shape_mp_list) {
        if (mp.m_mass <= 0) {
            continue;
        }

        const Vector3 d = mp.m_center_of_mass - rigid_center_of_mass;

        const Matrix33 shift =
            Matrix33::Identity() * d.squaredNorm() - d * d.transpose();

        body_inertia += mp.m_inertia + mp.m_mass * shift;
    }

    // Step5: get eigen values & eigen vectors
    // NOTE: SelfAdjointEigenSolver use to decompose symmetric matrix,
    //       faster than EigenSolver
    Eigen::SelfAdjointEigenSolver<Matrix33> solver(body_inertia);

    if (solver.info() != Eigen::Success) {
        return {};
    }

    Vector3 principal_inertia = solver.eigenvalues();
    Matrix33 principal_axes = solver.eigenvectors();

    // float error may cause negative value, force to positive
    principal_inertia = principal_inertia.cwiseMax(0);

    // Make sure that the rotation matrix is a right handed matrix
    if (principal_axes.determinant() < 0) {
        principal_axes.col(2) = -principal_axes.col(2);
    }

    RigidBodyMassProperties result;
    result.m_inv_mass = 1.0 / mass_sum;
    result.m_inv_inertia = Vector3{
        principal_inertia[0] > kREAL_EPSILON ? 1 / principal_inertia[0] : 0,
        principal_inertia[1] > kREAL_EPSILON ? 1 / principal_inertia[1] : 0,
        principal_inertia[2] > kREAL_EPSILON ? 1 / principal_inertia[2] : 0,
    };
    result.m_inertia_rotation = Quaternion(principal_axes).normalized();
    result.m_center_of_mass = rigid_center_of_mass;

    // validate solution is correct
#ifdef TOY_PHYSICS_CHECK
    // I = A * diag(Ix, Iy, Iz) * A^T
    const Matrix33 reconstructed = principal_axes *
                                   principal_inertia.asDiagonal() *
                                   principal_axes.transpose();

    for (int i = 0; i < 3; ++i) {
        TOY_ASSERT(reconstructed.col(i).isApprox(body_inertia.col(i),
                                                 kDefaultRealTolerance));
    }
#endif

    return result;
}

StaticRigidBody::StaticRigidBody(Foundation& foundation)
    : RigidActor{RigidActor::Type::Static, foundation} {}

}  // namespace toy_physics
