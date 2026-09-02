#pragma once
#include "toy_physics/lowlevel/algorithm.hpp"
#include "example.hpp"
#include "examples/raycast/util.hpp"

class TriangleRaycastExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_rotation = {0, 0, 0};
    float m_size = 3.f;
    bool m_cull = false;
    RayControl m_ray;
    uint16_t m_hit_count = 0;
    toy_physics::RaycastHit m_tri_hit = {};
};

inline void TriangleRaycastExample::OnUpdate(float delta_time) {}

inline void TriangleRaycastExample::OnRender3D(float delta_time) {
    ::Vector3 local[3] = {
        {0, 1, 0},
        {-0.866f, -0.5f, 0},
        {0.866f, -0.5f, 0},
    };

    Matrix rot = MatrixRotateXYZ({m_rotation.x * DEG2RAD,
                                  m_rotation.y * DEG2RAD,
                                  m_rotation.z * DEG2RAD});
    ::Vector3 world[3];
    for (int i = 0; i < 3; ++i) {
        ::Vector3 l = {local[i].x * m_size, local[i].y * m_size,
                       local[i].z * m_size};
        world[i] = Vector3Add(m_center, Vector3Transform(l, rot));
    }

    DrawTriangle3D(world[0], world[1], world[2], {0, 0, 255, 80});
    DrawTriangle3D(world[1], world[0], world[2], {0, 0, 255, 80});
    for (int i = 0; i < 3; ++i) {
        DrawLine3D(world[i], world[(i + 1) % 3], BLUE);
        DrawSphere(world[i], 0.15f, BLUE);
    }

    ::Vector3 origin = m_ray.GetOrigin(m_center);
    ::Vector3 dir = m_ray.GetDirection();
    DrawRay(origin, dir, m_ray.m_len);

    std::array<toy_physics::Vector3, 3> verts = {
        ToVec3(world[0]),
        ToVec3(world[1]),
        ToVec3(world[2]),
    };
    auto flags = toy_physics::QueryFlags(toy_physics::QueryFlag::Default) |
                 toy_physics::QueryFlag::UV;
    m_hit_count = m_cull
                      ? toy_physics::RaycastTriangleCullBackFace(
                            ToVec3(origin), ToVec3(dir), m_ray.m_len, verts,
                            &m_tri_hit, flags)
                      : toy_physics::RaycastTriangle(
                            ToVec3(origin), ToVec3(dir), m_ray.m_len, verts,
                            &m_tri_hit, flags);

    if (m_hit_count > 0) {
        ::Vector3 p = FromVec3(m_tri_hit.m_hit);
        DrawSphere(p, 0.25f, RED);
        DrawLine3D(p, Vector3Add(p, Vector3Scale(FromVec3(m_tri_hit.m_normal),
                                                 1.2f)),
                   GREEN);
    }
}

inline void TriangleRaycastExample::OnRender2D(float delta_time) {
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

    GuiGroupBox({x, y, kPanelW, kRowH * 9 + kSecPad + kPad}, "Triangle");
    y += kRowH;

    lbl({x, y, kLabelW, kCtrlH}, "Center X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.x, -30, 30);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.y, -30, 30);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.z, -30, 30);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Size");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_size, 0.1f, 15);
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
    y += kRowH;

    GuiCheckBox({x + 4, y, 24, 24}, "Cull Backface", &m_cull);
    y += kRowH + kSecPad;

    m_ray.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH);

    if (m_hit_count > 0) {
        char buf[96];
        snprintf(buf, sizeof(buf), "HIT: dist=%.2f u=%.2f v=%.2f",
                 m_tri_hit.m_dist, m_tri_hit.m_u, m_tri_hit.m_v);
        DrawText(buf, (int)x + 4, (int)y, 16, GREEN);
        y += 20;
        snprintf(buf, sizeof(buf), "n=(%.2f, %.2f, %.2f)", m_tri_hit.m_normal.x(),
                 m_tri_hit.m_normal.y(), m_tri_hit.m_normal.z());
        DrawText(buf, (int)x + 4, (int)y, 16, GREEN);
    } else {
        DrawText("MISS", (int)x + 4, (int)y, 26, RED);
    }
    y += 32;
    DrawRayLegend(x, y);
}
