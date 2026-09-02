#pragma once
#include "toy_physics/lowlevel/algorithm.hpp"
#include "example.hpp"
#include "examples/nearest_points/util.hpp"

class CylinderNearestPointExample : public IExample {
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
    SphericalPoint m_point;
};

inline void CylinderNearestPointExample::OnUpdate(float delta_time) {}

inline void CylinderNearestPointExample::OnRender3D(float delta_time) {
    ::Vector3 p = m_point.ToCartesian(m_center);

    float height = m_half_height * 2.f;
    m_ctx.DrawCylinder(m_center, m_rotation, height, m_radius, BLUE);
    m_ctx.DrawSphere(p, {0, 0, 0}, 0.3f, RED);

    Matrix rot = MatrixRotateXYZ({m_rotation.x * DEG2RAD,
                                  m_rotation.y * DEG2RAD,
                                  m_rotation.z * DEG2RAD});
    ::Vector3 axis = Vector3Normalize(Vector3Transform({0, 1, 0}, rot));

    auto np = toy_physics::GetCylinderNearestPoint(
        ToVec3(p), ToVec3(m_center), ToVec3(axis), m_half_height, m_radius);
    ::Vector3 nearest = FromVec3(np);
    m_ctx.DrawSphere(nearest, {0, 0, 0}, 0.3f, PURPLE);
    DrawLine3D(p, nearest, GREEN);
}

inline void CylinderNearestPointExample::OnRender2D(float delta_time) {
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

    // --- Cylinder ---
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

    m_point.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH, "Point P (rel. Cylinder)");

    y += kSecPad;
    DrawText("o Point P", (int)x + 4, (int)y, 16, RED);
    y += 16;
    DrawText("o Nearest Pt", (int)x + 4, (int)y, 16, PURPLE);
    y += 16;
    DrawText("-- Nearest Line", (int)x + 4, (int)y, 16, GREEN);
}
