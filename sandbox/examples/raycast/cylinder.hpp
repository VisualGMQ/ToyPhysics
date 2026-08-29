#pragma once
#include "example.hpp"
#include "examples/raycast/util.hpp"
#include "toy_physics/algorithm.hpp"

class CylinderRaycastExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_rotation = {0, 0, 0};
    float m_half_height = 4.f;
    float m_radius = 2.f;
    RayControl m_ray;
    std::array<toy_physics::RaycastHit, 2> m_hits = {};
    uint16_t m_hit_count = 0;
};

inline void CylinderRaycastExample::OnUpdate(float delta_time) {}

inline void CylinderRaycastExample::OnRender3D(float delta_time) {
    ::Vector3 origin = m_ray.GetOrigin(m_center);
    ::Vector3 dir = m_ray.GetDirection();

    m_ctx.DrawCylinder(m_center, m_rotation, m_half_height * 2.f, m_radius,
                       BLUE);

    Matrix rot = MatrixRotateXYZ({m_rotation.x * DEG2RAD,
                                  m_rotation.y * DEG2RAD,
                                  m_rotation.z * DEG2RAD});
    ::Vector3 axis = Vector3Normalize(Vector3Transform({0, 1, 0}, rot));

    DrawRay(origin, dir, m_ray.m_len);
    auto flags = toy_physics::QueryFlags(toy_physics::QueryFlag::Default) |
                 toy_physics::QueryFlag::AllSide;
    m_hit_count = toy_physics::RaycastCylinder(
        ToVec3(origin), ToVec3(dir), m_ray.m_len, ToVec3(m_center), ToVec3(axis),
        m_half_height, m_radius, m_hits, flags);
    DrawRayHits(origin, m_hits, m_hit_count);
}

inline void CylinderRaycastExample::OnRender2D(float delta_time) {
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

    GuiGroupBox({x, y, kPanelW, kRowH * 9 + kSecPad + kPad}, "Cylinder");
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
    lbl({x, y, kLabelW, kCtrlH}, "Half Height");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_half_height, 0.5f, 10);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Radius");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_radius, 0.5f, 5);
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
