#pragma once
#include "toy_physics/lowlevel/algorithm.hpp"
#include "example.hpp"
#include "examples/raycast/util.hpp"

class ConvexRaycastExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_rotation = {0, 0, 0};
    float m_radius = 3.f;
    float m_half_height = 2.5f;
    RayControl m_ray;
    std::array<toy_physics::RaycastHit, 2> m_hits = {};
    uint16_t m_hit_count = 0;
};

inline void ConvexRaycastExample::OnUpdate(float delta_time) {}

inline void ConvexRaycastExample::OnRender3D(float delta_time) {
    Matrix rot = MatrixRotateXYZ({m_rotation.x * DEG2RAD,
                                  m_rotation.y * DEG2RAD,
                                  m_rotation.z * DEG2RAD});

    // hexagonal prism
    std::array<::Vector3, 12> world;
    std::array<toy_physics::Vector3, 12> pts;
    for (int k = 0; k < 6; ++k) {
        float ang = k * 60.f * DEG2RAD;
        ::Vector3 top = {m_radius * cosf(ang), m_half_height,
                         m_radius * sinf(ang)};
        ::Vector3 bot = {m_radius * cosf(ang), -m_half_height,
                         m_radius * sinf(ang)};
        world[k] = Vector3Add(m_center, Vector3Transform(top, rot));
        world[k + 6] = Vector3Add(m_center, Vector3Transform(bot, rot));
        pts[k] = ToVec3(world[k]);
        pts[k + 6] = ToVec3(world[k + 6]);
    }

    std::array<toy_physics::Vector3, 8> normals;
    for (int k = 0; k < 6; ++k) {
        float mid = (k * 60.f + 30.f) * DEG2RAD;
        normals[k] = ToVec3(Vector3Normalize(
            Vector3Transform({cosf(mid), 0, sinf(mid)}, rot)));
    }
    normals[6] = ToVec3(Vector3Transform({0, 1, 0}, rot));
    normals[7] = ToVec3(Vector3Transform({0, -1, 0}, rot));

    Color face = {0, 0, 255, 76};
    for (int k = 0; k < 6; ++k) {
        int k1 = (k + 1) % 6;
        DrawTriangle3D(world[k], world[k1], world[k1 + 6], face);
        DrawTriangle3D(world[k], world[k1 + 6], world[k + 6], face);
    }
    for (int k = 1; k < 5; ++k) {
        DrawTriangle3D(world[0], world[k + 1], world[k], face);
        DrawTriangle3D(world[6], world[6 + k], world[6 + k + 1], face);
    }
    for (int k = 0; k < 6; ++k) {
        DrawLine3D(world[k], world[(k + 1) % 6], BLUE);
        DrawLine3D(world[k + 6], world[(k + 1) % 6 + 6], BLUE);
        DrawLine3D(world[k], world[k + 6], BLUE);
        DrawSphere(world[k], 0.12f, BLUE);
        DrawSphere(world[k + 6], 0.12f, BLUE);
    }

    ::Vector3 origin = m_ray.GetOrigin(m_center);
    ::Vector3 dir = m_ray.GetDirection();
    DrawRay(origin, dir, m_ray.m_len);

    auto flags = toy_physics::QueryFlags(toy_physics::QueryFlag::Default) |
                 toy_physics::QueryFlag::AllSide;
    m_hit_count = toy_physics::RaycastConvex(ToVec3(origin), ToVec3(dir),
                                             m_ray.m_len, pts, normals, m_hits,
                                             flags);
    DrawRayHits(origin, m_hits, m_hit_count);
}

inline void ConvexRaycastExample::OnRender2D(float delta_time) {
    static constexpr float kPanelW = 300.f;
    static constexpr float kPanelX = 10.f;
    static constexpr float kLabelW = 110.f;
    static constexpr float kCtrlH = 28.f;
    static constexpr float kPad = 6.f;
    static constexpr float kRowH = kCtrlH + kPad;
    static constexpr float kSecPad = 16.f;

    float x = (float)GetScreenWidth() - kPanelW - kPanelX;
    float y = 100.f;

    auto lbl = [](Rectangle r, const char* text) {
        int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
        GuiLabel(r, text);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
    };

    GuiGroupBox({x, y, kPanelW, kRowH * 9 + kSecPad + kPad},
                "Convex (Hex Prism)");
    y += kRowH;

    lbl({x, y, kLabelW, kCtrlH}, "Center X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.x, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.y, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.z, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Radius");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_radius, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Half Height");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_half_height, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Pitch");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_rotation.x, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Yaw");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_rotation.y, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Roll");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_rotation.z, -180, 180);
    y += kRowH + kSecPad;

    m_ray.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH);
    DrawRayInfo(m_hits, m_hit_count, x, y);
    DrawRayLegend(x, y);
}
