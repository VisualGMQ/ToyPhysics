#include "camera.hpp"

Frustum ResizeFrustumInNewWindowSize(
    const Eigen::Vector2d& old_window_size,
    const Eigen::Vector2d& new_window_size, const Frustum& frustum) {
    Frustum new_frustum;
    float new_aspect = new_window_size.x() / float(new_window_size.y());
    float fov_height = frustum.m_near * std::tan((frustum.m_fov * 0.5f).Value());
    float new_fov_height = fov_height * new_window_size.y() / old_window_size.
                           y();
    float new_fov = std::atan(new_fov_height / frustum.m_near) * 2;

    new_frustum.m_fov = new_fov;
    new_frustum.m_aspect = new_aspect;
    new_frustum.m_near = frustum.m_near;
    new_frustum.m_far = frustum.m_far;
    return new_frustum;
}

Frustum Camera::GetFrustum() const {
    return m_frustum;
}

const Eigen::Matrix4f& Camera::GetProject() const {
    return m_project;
}

void Camera::SetProject(Radians fov, float aspect, float near, float far) {
    m_project = CreatePersp(fov, aspect, near, far);
    m_frustum.m_near = near;
    m_frustum.m_far = far;
    m_frustum.m_fov = fov;
    m_frustum.m_aspect = aspect;
}

FlyCamera::FlyCamera(Radians fov, float aspect, float near, float far) {
    SetProject(fov, aspect, near, far);
}

Eigen::Matrix4f FlyCamera::GetView() const {
    return CreateXRotation(-m_pitch) * CreateYRotation(-m_yaw) *
           CreateTranslation(static_cast<Eigen::Vector3f>(-m_position));
}

void FlyCamera::SetYaw(Radians value) {
    m_yaw = value;
}

void FlyCamera::SetPitch(Radians value) {
    m_pitch =
        std::clamp(value, Radians{-EIGEN_PI * 0.5 + 0.001},
                   Radians{EIGEN_PI * 0.5 - 0.001});
}

void FlyCamera::AddYaw(Radians value) {
    m_yaw += value;
}

void FlyCamera::AddPitch(Radians value) {
    m_pitch = std::clamp(m_pitch + value, Radians{-EIGEN_PI * 0.5 + 0.001},
                         Radians{EIGEN_PI * 0.5 - 0.001});
}

Radians FlyCamera::GetYaw() const {
    return m_yaw;
}

Radians FlyCamera::GetPitch() const {
    return m_pitch;
}

void FlyCamera::Move(const Eigen::Vector3f& offset) {
    m_position += offset;
}

void FlyCamera::MoveTo(const Eigen::Vector3f& pos) {
    m_position = pos;
}

const Eigen::Vector3f& FlyCamera::GetPosition() const {
    return m_position;
}

Eigen::Vector3f FlyCamera::GetForward() const {
    return GetRotation() * -Eigen::Vector3f::UnitZ();
}

Eigen::Vector3f FlyCamera::GetUp() const {
    return GetRotation() * Eigen::Vector3f::UnitY();
}

Eigen::Quaternionf FlyCamera::GetRotation() const {
    return Eigen::Quaternionf{
               Eigen::AngleAxisf{m_yaw.Value(), Eigen::Vector3f::UnitY()}} *
           Eigen::Quaternionf{
               Eigen::AngleAxisf{m_pitch.Value(), Eigen::Vector3f::UnitX()}};
}

void FlyCamera::MoveForward(float dist) {
    m_position += GetForward() * dist;
}

void FlyCamera::MoveRightLeft(float dist) {
    auto right_dir = Eigen::Quaternionf{
                         Eigen::AngleAxisf{m_yaw.Value(),
                                           Eigen::Vector3f::UnitY()}} *
                     Eigen::Quaternionf{
                         Eigen::AngleAxisf{m_pitch.Value(),
                                           Eigen::Vector3f::UnitX()}} *
                     Eigen::Vector3f{1, 0, 0};
    m_position += right_dir * dist;
}

void FlyCamera::MoveUpDown(float dist) {
    m_position += GetUp() * dist;
}

OrbitCamera::OrbitCamera(Radians fov, float aspect, float near, float far) {
    SetProject(fov, aspect, near, far);
}

void OrbitCamera::SetTheta(Radians theta) {
    m_theta = theta;
    float l = std::sin(m_phi.Value()) * m_radius;
    m_position.x() = l * std::cos(m_theta.Value());
    m_position.z() = l * std::sin(m_theta.Value());
}

void OrbitCamera::SetPhi(Radians phi) {
    m_phi = std::clamp<float>(phi.Value(), 0.0001, EIGEN_PI - 0.0001);
    m_position.y() = std::cos(m_phi.Value()) * m_radius;
    float l = std::sin(m_phi.Value()) * m_radius;
    m_position.x() = l * std::cos(m_theta.Value());
    m_position.z() = l * std::sin(m_theta.Value());
}

void OrbitCamera::AddTheta(Radians theta) {
    SetTheta(m_theta + theta);
}

void OrbitCamera::AddPhi(Radians phi) {
    SetPhi(m_phi + phi);
}

void OrbitCamera::SetRadius(float radius) {
    constexpr float MinDist = 0.00001;
    m_radius = radius < MinDist ? MinDist : radius;
}

void OrbitCamera::AddRadius(float radius) {
    SetRadius(m_radius + radius);
}

float OrbitCamera::GetRadius() const {
    return m_radius;
}

Radians OrbitCamera::GetTheta() const {
    return m_theta;
}

Radians OrbitCamera::GetPhi() const {
    return m_phi;
}

const Eigen::Vector3f& OrbitCamera::GetPosition() const {
    return m_position;
}

Eigen::Matrix4f OrbitCamera::GetView() const {
    return LookAt(m_target, m_position, Eigen::Vector3f{0, 1, 0});
}