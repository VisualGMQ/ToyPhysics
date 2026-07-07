#pragma once
#include "toy_physics/math.hpp"


struct Frustum {
    float m_near{}, m_far{}, m_aspect = 1.0;
    Radians m_fov = Degrees{45};
};

Frustum ResizeFrustumInNewWindowSize(
    const Eigen::Vector2d& old_window_size,
    const Eigen::Vector2d& new_window_size, const Frustum& frustum);

class Camera {
public:
    void SetProject(Radians fov, float aspect, float near, float far);
    virtual Eigen::Matrix4f GetView() const = 0;
    virtual ~Camera() = default;
    Frustum GetFrustum() const;
    const Eigen::Matrix4f& GetProject() const;
    virtual const Eigen::Vector3f& GetPosition() const = 0;

protected:
    Frustum m_frustum;
    Eigen::Matrix4f m_project = Eigen::Matrix4f::Identity();
};

class FlyCamera : public Camera {
public:
    FlyCamera(Radians fov, float aspect, float near, float far);

    Eigen::Matrix4f GetView() const override;

    void SetYaw(Radians value);

    void SetPitch(Radians value);

    void AddYaw(Radians value);

    void AddPitch(Radians value);

    Radians GetYaw() const;

    Radians GetPitch() const;

    void Move(const Eigen::Vector3f& offset);

    void MoveTo(const Eigen::Vector3f& pos);

    const Eigen::Vector3f& GetPosition() const override;

    Eigen::Vector3f GetForward() const;

    Eigen::Vector3f GetUp() const;

    Eigen::Quaternionf GetRotation() const;

    void MoveForward(float dist);

    void MoveRightLeft(float dist);

    void MoveUpDown(float dist);

private:
    Radians m_pitch{0};
    Radians m_yaw{0};
    Eigen::Vector3f m_position = Eigen::Vector3f::Zero();
};


class OrbitCamera : public Camera {
public:
    OrbitCamera(Radians fov, float aspect, float near, float far);

    void SetTheta(Radians);
    void SetPhi(Radians);
    void AddTheta(Radians);
    void AddPhi(Radians);
    void SetRadius(float);
    void AddRadius(float);
    float GetRadius() const;
    Radians GetTheta() const;
    Radians GetPhi() const;
    const Eigen::Vector3f& GetPosition() const override;

    Eigen::Matrix4f GetView() const override;

private:
    Eigen::Matrix4f m_project;
    Eigen::Vector3f m_target;
    Eigen::Vector3f m_position;
    float m_radius{10};
    Radians m_theta;  // angle in XoZ
    Radians m_phi;    // angle from Y-Axis
    Eigen::Vector2d m_window_size;
};