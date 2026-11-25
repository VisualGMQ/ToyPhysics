#pragma once

#include "example.hpp"

#include "toy_physics/foundation.hpp"
#include "toy_physics/geometry/box.hpp"
#include "toy_physics/geometry/capsule.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/geometry/cylinder.hpp"
#include "toy_physics/rigid_sim/rigidbody.hpp"
#include "toy_physics/rigid_sim/shape.hpp"
#include "toy_physics/world.hpp"

#include "raygui.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <array>
#include <memory>
#include <random>
#include <span>
#include <vector>

namespace rigid_multibody {

inline ::Vector3 ToRay(const toy_physics::Vector3& v) {
    return ::Vector3{v.x(), v.y(), v.z()};
}

inline ::Vector3 ToEulerDegrees(const toy_physics::Quaternion& q) {
    const ::Quaternion rq{q.x(), q.y(), q.z(), q.w()};
    const ::Vector3 euler = QuaternionToEuler(rq);

    return ::Vector3{euler.x * RAD2DEG, euler.y * RAD2DEG,
                     euler.z * RAD2DEG};
}

/**
 * 自己造的一个简单凸包：三棱柱（楔形）。
 *
 * 顶点顺序就是传给 ConvexData 的顺序。
 */
inline const std::array<toy_physics::Vector3, 6>& WedgeVertices() {
    static const std::array<toy_physics::Vector3, 6> kVertices = {
        toy_physics::Vector3{-0.8f, -0.5f, -0.5f},
        toy_physics::Vector3{0.8f, -0.5f, -0.5f},
        toy_physics::Vector3{0.0f, -0.5f, 0.6f},
        toy_physics::Vector3{-0.8f, 0.5f, -0.5f},
        toy_physics::Vector3{0.8f, 0.5f, -0.5f},
        toy_physics::Vector3{0.0f, 0.5f, 0.6f},
    };
    return kVertices;
}

inline void DrawWedge(const toy_physics::Transform& transform,
                      std::span<const toy_physics::Vector3> vertices,
                      ::Color fill, ::Color wire) {
    if (vertices.size() < 6) {
        return;
    }

    static constexpr int kTriangles[8][3] = {
        {0, 1, 2},  // bottom
        {3, 4, 5},  // top
        {0, 3, 4}, {0, 4, 1},
        {1, 4, 5}, {1, 5, 2},
        {2, 5, 3}, {2, 3, 0},
    };

    static constexpr int kEdges[9][2] = {
        {0, 1}, {1, 2}, {2, 0},
        {3, 4}, {4, 5}, {5, 3},
        {0, 3}, {1, 4}, {2, 5},
    };

    std::array<::Vector3, 6> world;
    for (size_t i = 0; i < 6; ++i) {
        world[i] = ToRay(transform.TransformBy(vertices[i]));
    }

    // 不做朝向判断，直接双面绘制，让深度测试决定可见面
    rlDisableBackfaceCulling();
    for (const auto& triangle : kTriangles) {
        DrawTriangle3D(world[triangle[0]], world[triangle[1]],
                       world[triangle[2]], fill);
    }
    rlEnableBackfaceCulling();

    for (const auto& edge : kEdges) {
        DrawLine3D(world[edge[0]], world[edge[1]], wire);
    }
}

}  // namespace rigid_multibody

class RigidMultiBodyExample : public IExample {
public:
    RigidMultiBodyExample(Context& ctx, const std::string& name)
        : IExample{ctx, name} {
        BuildScene();
    }

    ~RigidMultiBodyExample() override {
        DestroyScene();
    }

    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    enum class Kind {
        Box,
        Capsule,
        Cylinder,
        Convex,
    };

    struct Body {
        toy_physics::RigidActor* m_actor{};
        Kind m_kind{Kind::Box};
        toy_physics::Vector3 m_half_extent{toy_physics::Vector3::Ones()};
        toy_physics::real m_radius{0};
        toy_physics::real m_half_height{0};
        std::span<const toy_physics::Vector3> m_hull_vertices{};
        ::Color m_color{WHITE};
    };

    void BuildScene();
    void DestroyScene();
    void ResetScene();

    void DrawBody(const Body& body) const;

    toy_physics::Foundation m_foundation;
    toy_physics::World* m_world{};

    std::vector<Body> m_bodies;
    std::unique_ptr<toy_physics::ConvexData> m_convex_data;

    float m_accumulator{0.f};
    bool m_paused{false};

    static constexpr float kFixedDt = 1.f / 60.f;
    static constexpr int kMaxSubSteps = 4;
};

inline void RigidMultiBodyExample::BuildScene() {
    toy_physics::World::Settings settings;
    settings.m_gravity = toy_physics::Vector3{0.f, -9.8f, 0.f};
    settings.m_solver_settings.m_iter_num = 20;

    m_world = m_foundation.CreateWorld(settings);

    constexpr toy_physics::real kDeg =
        static_cast<toy_physics::real>(3.14159265358979323846 / 180.0);

    // ---- 静态场景 ----

    auto add_static_box = [&](const toy_physics::Vector3& half_extent,
                              const toy_physics::Vector3& position,
                              const toy_physics::Quaternion& rotation,
                              toy_physics::real friction, ::Color color) {
        toy_physics::StaticRigidBody* actor = m_foundation.CreateStatic();
        toy_physics::Shape* shape =
            m_foundation.CreateShape(toy_physics::BoxGeometry{half_extent});
        shape->SetFriction(friction);

        actor->AttachShape(*shape);
        actor->Teleport(toy_physics::Transform{position, rotation});
        m_world->AddRigidActor(*actor);

        m_bodies.push_back(Body{
            actor,
            Kind::Box,
            half_extent,
            0,
            0,
            {},
            color,
        });
    };

    // 地面
    add_static_box(toy_physics::Vector3{7.f, 0.5f, 7.f},
                   toy_physics::Vector3{0.f, -0.5f, 0.f},
                   toy_physics::Quaternion::Identity(), 0.8f,
                   ::Color{85, 85, 95, 255});

    // 四周的低墙，防止圆柱/胶囊一直滚出场地
    add_static_box(toy_physics::Vector3{7.f, 0.6f, 0.25f},
                   toy_physics::Vector3{0.f, 0.6f, -7.f},
                   toy_physics::Quaternion::Identity(), 0.5f,
                   ::Color{70, 70, 80, 255});
    add_static_box(toy_physics::Vector3{7.f, 0.6f, 0.25f},
                   toy_physics::Vector3{0.f, 0.6f, 7.f},
                   toy_physics::Quaternion::Identity(), 0.5f,
                   ::Color{70, 70, 80, 255});
    add_static_box(toy_physics::Vector3{0.25f, 0.6f, 7.f},
                   toy_physics::Vector3{-7.f, 0.6f, 0.f},
                   toy_physics::Quaternion::Identity(), 0.5f,
                   ::Color{70, 70, 80, 255});
    add_static_box(toy_physics::Vector3{0.25f, 0.6f, 7.f},
                   toy_physics::Vector3{7.f, 0.6f, 0.f},
                   toy_physics::Quaternion::Identity(), 0.5f,
                   ::Color{70, 70, 80, 255});

    // 中央平台
    add_static_box(toy_physics::Vector3{2.f, 0.2f, 2.f},
                   toy_physics::Vector3{0.f, 0.6f, 0.f},
                   toy_physics::Quaternion::Identity(), 0.7f,
                   ::Color{105, 105, 115, 255});

    // 左右两个斜坡，把物体滑向中间
    const toy_physics::Quaternion left_ramp(
        Eigen::AngleAxis<toy_physics::real>(-20.f * kDeg,
                                            toy_physics::Vector3::UnitZ()));
    const toy_physics::Quaternion right_ramp(
        Eigen::AngleAxis<toy_physics::real>(20.f * kDeg,
                                            toy_physics::Vector3::UnitZ()));

    add_static_box(toy_physics::Vector3{2.6f, 0.2f, 1.6f},
                   toy_physics::Vector3{-4.2f, 1.3f, 0.f}, left_ramp, 0.5f,
                   ::Color{115, 110, 100, 255});

    add_static_box(toy_physics::Vector3{2.6f, 0.2f, 1.6f},
                   toy_physics::Vector3{4.2f, 1.3f, 0.f}, right_ramp, 0.5f,
                   ::Color{115, 110, 100, 255});

    // ---- 动态物体 ----

    std::mt19937 rng{20261125};
    std::uniform_real_distribution<float> uniform{-1.f, 1.f};

    auto random_rotation = [&](float angle_scale) {
        const toy_physics::Vector3 axis =
            toy_physics::Vector3{uniform(rng), uniform(rng), uniform(rng)}
                .normalized();
        return toy_physics::Quaternion(Eigen::AngleAxis<toy_physics::real>(
            angle_scale * uniform(rng), axis));
    };

    auto add_dynamic = [&](Body body, const toy_physics::Vector3& position,
                           const toy_physics::Quaternion& rotation) {
        toy_physics::DynamicRigidBody* actor = m_foundation.CreateDynamic();
        toy_physics::Shape* shape = nullptr;

        switch (body.m_kind) {
            case Kind::Box:
                shape = m_foundation.CreateShape(
                    toy_physics::BoxGeometry{body.m_half_extent});
                break;
            case Kind::Capsule:
                shape = m_foundation.CreateShape(
                    toy_physics::CapsuleGeometry{body.m_radius,
                                                 body.m_half_height});
                break;
            case Kind::Cylinder:
                shape = m_foundation.CreateShape(
                    toy_physics::CylinderGeometry{body.m_radius,
                                                  body.m_half_height});
                break;
            case Kind::Convex:
                shape = m_foundation.CreateShape(
                    toy_physics::ConvexHullGeometry{*m_convex_data});
                break;
        }

        shape->SetFriction(0.6f);
        shape->SetDensity(1.f);

        actor->AttachShape(*shape);
        actor->Teleport(toy_physics::Transform{position, rotation});
        m_world->AddRigidActor(*actor);

        body.m_actor = actor;
        m_bodies.push_back(body);
    };

    // 凸包数据：自己造的楔形
    m_convex_data = std::make_unique<toy_physics::ConvexData>(
        std::span<const toy_physics::Vector3>(rigid_multibody::WedgeVertices()));

    const std::span<const toy_physics::Vector3> wedge_vertices =
        m_convex_data->GetVertices();

    // 三种基本形状各来几个，再配几个楔形凸包
    for (int i = 0; i < 3; ++i) {
        const float x = -2.6f + 2.6f * static_cast<float>(i % 2);
        const float z = -1.6f + 1.6f * static_cast<float>(i / 2);
        const float y = 4.5f + 0.9f * static_cast<float>(i);

        add_dynamic(Body{
                        nullptr,
                        Kind::Box,
                        toy_physics::Vector3{0.45f, 0.45f, 0.45f},
                        0,
                        0,
                        {},
                        ::Color{220, 90, 90, 255},
                    },
                    toy_physics::Vector3{x, y, z}, random_rotation(0.4f));
    }

    for (int i = 0; i < 2; ++i) {
        add_dynamic(Body{
                        nullptr,
                        Kind::Capsule,
                        toy_physics::Vector3::Zero(),
                        0.35f,
                        0.55f,
                        {},
                        ::Color{90, 180, 120, 255},
                    },
                    toy_physics::Vector3{-2.2f + 4.4f * static_cast<float>(i),
                                         6.2f + 0.8f * static_cast<float>(i),
                                         1.4f},
                    random_rotation(0.6f));
    }

    for (int i = 0; i < 2; ++i) {
        add_dynamic(Body{
                        nullptr,
                        Kind::Cylinder,
                        toy_physics::Vector3::Zero(),
                        0.45f,
                        0.55f,
                        {},
                        ::Color{90, 130, 220, 255},
                    },
                    toy_physics::Vector3{-1.4f + 2.8f * static_cast<float>(i),
                                         7.4f + 0.6f * static_cast<float>(i),
                                         -1.4f},
                    random_rotation(0.6f));
    }

    for (int i = 0; i < 3; ++i) {
        add_dynamic(Body{
                        nullptr,
                        Kind::Convex,
                        toy_physics::Vector3::Zero(),
                        0,
                        0,
                        wedge_vertices,
                        ::Color{210, 160, 80, 255},
                    },
                    toy_physics::Vector3{-1.8f + 1.8f * static_cast<float>(i),
                                         8.6f + 0.7f * static_cast<float>(i),
                                         0.6f * static_cast<float>(i - 1)},
                    random_rotation(0.8f));
    }
}

inline void RigidMultiBodyExample::DestroyScene() {
    for (Body& body : m_bodies) {
        m_foundation.DestroyRigidActor(*body.m_actor);
    }
    m_bodies.clear();

    // 必须等所有 ConvexHullGeometry 的 Shape 销毁之后再释放
    m_convex_data.reset();

    if (m_world != nullptr) {
        m_foundation.DestroyWorld(*m_world);
        m_world = nullptr;
    }
}

inline void RigidMultiBodyExample::ResetScene() {
    DestroyScene();
    BuildScene();
    m_accumulator = 0.f;
}

inline void RigidMultiBodyExample::DrawBody(const Body& body) const {
    const toy_physics::Transform& transform = body.m_actor->GetTransform();
    const ::Vector3 position = rigid_multibody::ToRay(transform.m_position);
    const ::Vector3 rotation =
        rigid_multibody::ToEulerDegrees(transform.m_rotation);

    switch (body.m_kind) {
        case Kind::Box:
            m_ctx.DrawBox(position, rotation,
                          rigid_multibody::ToRay(body.m_half_extent),
                          body.m_color);
            break;
        case Kind::Capsule:
            m_ctx.DrawCapsule(position, rotation,
                              body.m_half_height * 2.f, body.m_radius,
                              body.m_color);
            break;
        case Kind::Cylinder:
            m_ctx.DrawCylinder(position, rotation,
                               body.m_half_height * 2.f, body.m_radius,
                               body.m_color);
            break;
        case Kind::Convex:
            rigid_multibody::DrawWedge(
                transform, body.m_hull_vertices, body.m_color,
                ::Color{120, 85, 35, 255});
            break;
    }
}

inline void RigidMultiBodyExample::OnUpdate(float delta_time) {
    if (m_world == nullptr || m_paused) {
        return;
    }

    m_accumulator += delta_time;

    int sub_steps = 0;
    while (m_accumulator >= kFixedDt && sub_steps < kMaxSubSteps) {
        m_world->Simulate(kFixedDt);
        m_accumulator -= kFixedDt;
        ++sub_steps;
    }

    if (m_accumulator > kFixedDt * 8.f) {
        m_accumulator = 0.f;
    }
}

inline void RigidMultiBodyExample::OnRender3D(float delta_time) {
    (void)delta_time;

    for (const Body& body : m_bodies) {
        DrawBody(body);
    }
}

inline void RigidMultiBodyExample::OnRender2D(float delta_time) {
    (void)delta_time;

    static constexpr float kPanelW = 320.f;
    static constexpr float kPanelX = 10.f;
    const float x = static_cast<float>(GetScreenWidth()) - kPanelW - kPanelX;
    float y = 90.f;

    GuiGroupBox({x, y, kPanelW, 160.f}, "Rigid multibody");
    y += 12.f;

    DrawText(TextFormat("Bodies: %d", static_cast<int>(m_bodies.size())),
             static_cast<int>(x) + 10, static_cast<int>(y), 18, GREEN);
    y += 24.f;

    DrawText("Box / Capsule / Cylinder / ConvexHull",
             static_cast<int>(x) + 10, static_cast<int>(y), 16, GRAY);
    y += 28.f;

    if (GuiButton({x + 10.f, y, kPanelW - 20.f, 30.f},
                  m_paused ? "Resume" : "Pause")) {
        m_paused = !m_paused;
    }
    y += 38.f;

    if (GuiButton({x + 10.f, y, kPanelW - 20.f, 30.f}, "Reset")) {
        ResetScene();
    }
    y += 40.f;

    DrawText("Gravity drops them onto the ramps",
             static_cast<int>(x) + 10, static_cast<int>(y), 16, GRAY);
}
