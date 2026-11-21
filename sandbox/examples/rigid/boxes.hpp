#pragma once

#include "example.hpp"

#include "toy_physics/foundation.hpp"
#include "toy_physics/geometry/box.hpp"
#include "toy_physics/rigid_sim/rigidbody.hpp"
#include "toy_physics/rigid_sim/shape.hpp"
#include "toy_physics/world.hpp"

#include "raymath.h"

#include <cmath>
#include <random>
#include <vector>

namespace rigid_example {

inline ::Vector3 ToRay(const toy_physics::Vector3& v) {
    return ::Vector3{v.x(), v.y(), v.z()};
}

inline ::Vector3 ToEulerDegrees(const toy_physics::Quaternion& q) {
    const ::Quaternion rq{q.x(), q.y(), q.z(), q.w()};
    const ::Vector3 euler = QuaternionToEuler(rq);

    return ::Vector3{euler.x * RAD2DEG, euler.y * RAD2DEG,
                     euler.z * RAD2DEG};
}

}  // namespace rigid_example

class RigidBoxesExample : public IExample {
public:
    RigidBoxesExample(Context& ctx, const std::string& name)
        : IExample{ctx, name} {
        BuildScene();
    }

    ~RigidBoxesExample() override {
        DestroyScene();
    }

    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    struct Body {
        toy_physics::RigidActor* m_actor{};
        toy_physics::Vector3 m_half_extent{toy_physics::Vector3::Ones()};
        ::Color m_color{WHITE};
    };

    void BuildScene();
    void DestroyScene();
    void ResetScene();

    toy_physics::Foundation m_foundation;
    toy_physics::World* m_world{};
    std::vector<Body> m_bodies;

    float m_accumulator{0.f};
    bool m_paused{false};

    int m_layers{7};
    float m_layers_slider{7.f};

    static constexpr float kFixedDt = 1.f / 60.f;
    static constexpr int kMaxSubSteps = 4;
    static constexpr int kMinLayers = 2;
    static constexpr int kMaxLayers = 20;
};

inline void RigidBoxesExample::BuildScene() {
    toy_physics::World::Settings settings;
    settings.m_gravity = toy_physics::Vector3{0.f, -9.8f, 0.f};
    settings.m_solver_settings.m_iter_num = 20;

    m_world = m_foundation.CreateWorld(settings);

    // ---- 静态地面 ----
    {
        toy_physics::StaticRigidBody* ground = m_foundation.CreateStatic();
        toy_physics::Shape* shape =
            m_foundation.CreateShape(toy_physics::BoxGeometry{
                toy_physics::Vector3{12.f, 0.5f, 12.f}});
        shape->SetFriction(0.8f);

        ground->AttachShape(*shape);
        ground->Teleport(toy_physics::Transform{
            toy_physics::Vector3{0.f, -0.5f, 0.f},
            toy_physics::Quaternion::Identity()});

        m_world->AddRigidActor(*ground);

        m_bodies.push_back(Body{
            ground,
            toy_physics::Vector3{12.f, 0.5f, 12.f},
            ::Color{90, 90, 100, 255},
        });
    }

    // ---- 动态方块：小金字塔 ----
    std::mt19937 rng{1337};
    std::uniform_real_distribution<float> uniform{-1.f, 1.f};

    int index = 0;

    auto add_box = [&](toy_physics::Vector3 position,
                       toy_physics::Quaternion rotation) {
        const float half = 0.5f;
        const toy_physics::Vector3 half_extent{half, half, half};

        toy_physics::DynamicRigidBody* body = m_foundation.CreateDynamic();

        toy_physics::Shape* shape = m_foundation.CreateShape(
            toy_physics::BoxGeometry{half_extent});
        shape->SetFriction(0.6f);
        shape->SetDensity(1.f);

        body->AttachShape(*shape);
        body->Teleport(toy_physics::Transform{position, rotation});
        m_world->AddRigidActor(*body);

        const unsigned char r = static_cast<unsigned char>(110 + index * 9);
        const unsigned char g = static_cast<unsigned char>(90 + index * 6);
        const unsigned char b = static_cast<unsigned char>(210 - index * 7);

        m_bodies.push_back(Body{
            body,
            half_extent,
            ::Color{r, g, b, 255},
        });

        ++index;
    };

    const int layers = m_layers;
    for (int layer = 0; layer < layers; ++layer) {
        const int count = layers - layer;
        const float y = 0.5f + 1.01f * static_cast<float>(layer);

        for (int k = 0; k < count; ++k) {
            const float x =
                (static_cast<float>(k) - 0.5f * static_cast<float>(count - 1)) *
                1.03f;

            const toy_physics::Vector3 axis =
                toy_physics::Vector3{uniform(rng), uniform(rng), uniform(rng)}
                    .normalized();
            const toy_physics::Quaternion rotation(
                Eigen::AngleAxis<toy_physics::real>(0.03f * uniform(rng),
                                                    axis));

            add_box(toy_physics::Vector3{x, y, 0.f}, rotation);
        }
    }

    // 侧面再放两个，稍微抬高一点自由落体
    add_box(toy_physics::Vector3{-3.5f, 0.6f, 2.f},
            toy_physics::Quaternion::Identity());
    add_box(toy_physics::Vector3{3.5f, 0.8f, -2.f},
            toy_physics::Quaternion::Identity());
}

inline void RigidBoxesExample::DestroyScene() {
    for (Body& body : m_bodies) {
        m_foundation.DestroyRigidActor(*body.m_actor);
    }
    m_bodies.clear();

    if (m_world != nullptr) {
        m_foundation.DestroyWorld(*m_world);
        m_world = nullptr;
    }
}

inline void RigidBoxesExample::ResetScene() {
    DestroyScene();
    BuildScene();
    m_accumulator = 0.f;
}

inline void RigidBoxesExample::OnUpdate(float delta_time) {
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

    // 累积太多就丢掉，避免掉帧后追帧追到卡死
    if (m_accumulator > kFixedDt * 8.f) {
        m_accumulator = 0.f;
    }
}

inline void RigidBoxesExample::OnRender3D(float delta_time) {
    (void)delta_time;

    for (const Body& body : m_bodies) {
        const toy_physics::Transform& transform = body.m_actor->GetTransform();

        m_ctx.DrawBox(rigid_example::ToRay(transform.m_position),
                      rigid_example::ToEulerDegrees(transform.m_rotation),
                      rigid_example::ToRay(body.m_half_extent),
                      body.m_color);
    }
}

inline void RigidBoxesExample::OnRender2D(float delta_time) {
    (void)delta_time;

    static constexpr float kPanelW = 320.f;
    static constexpr float kPanelX = 10.f;
    const float x = static_cast<float>(GetScreenWidth()) - kPanelW - kPanelX;
    float y = 90.f;

    GuiGroupBox({x, y, kPanelW, 250.f}, "Rigid bodies");
    y += 12.f;

    DrawText(TextFormat("Bodies: %d", static_cast<int>(m_bodies.size())),
             static_cast<int>(x) + 10, static_cast<int>(y), 18, GREEN);
    y += 24.f;

    DrawText(TextFormat("Layers: %d", m_layers),
             static_cast<int>(x) + 10, static_cast<int>(y), 18, GREEN);
    y += 24.f;

    GuiSliderBar({x + 10.f, y, kPanelW - 20.f, 24.f}, nullptr, nullptr,
                 &m_layers_slider, static_cast<float>(kMinLayers),
                 static_cast<float>(kMaxLayers));
    y += 32.f;

    // 拖动滑块时，只要取整后的层数变了就立刻重建场景
    const int wanted_layers =
        static_cast<int>(std::lround(m_layers_slider));
    if (wanted_layers != m_layers) {
        m_layers = wanted_layers;
        m_layers_slider = static_cast<float>(m_layers);
        ResetScene();
    }

    if (GuiButton({x + 10.f, y, kPanelW - 20.f, 30.f},
                  m_paused ? "Resume" : "Pause")) {
        m_paused = !m_paused;
    }
    y += 38.f;

    if (GuiButton({x + 10.f, y, kPanelW - 20.f, 30.f}, "Reset")) {
        ResetScene();
    }
    y += 44.f;

    DrawText("Slide to change layers (2-20)", static_cast<int>(x) + 10,
             static_cast<int>(y), 16, GRAY);
}
