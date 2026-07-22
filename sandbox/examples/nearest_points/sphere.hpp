#pragma once
#include "example.hpp"
#include "examples/nearest_points/util.hpp"
#include "toy_physics/algorithm.hpp"

class SphereNearestPointExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    float m_radius = 4.f;
    SphericalPoint m_point;
};

inline void SphereNearestPointExample::OnUpdate(float delta_time) {}

inline void SphereNearestPointExample::OnRender3D(float delta_time) {
    ::Vector3 p = m_point.ToCartesian(m_center);

    m_ctx.DrawSphere(m_center, {0, 0, 0}, m_radius);
    DrawSphere(p, 0.3f, RED);

    auto np = toy_physics::GetSphereNearestPoint(ToVec3(p), ToVec3(m_center),
                                                 m_radius);
    ::Vector3 nearest = FromVec3(np);
    DrawSphere(nearest, 0.3f, PURPLE);
    DrawLine3D(p, nearest, GREEN);
}

inline void SphereNearestPointExample::OnRender2D(float delta_time) {
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

    // --- Sphere ---
    GuiGroupBox({x, y, kPanelW, kRowH * 5 + kSecPad + kPad}, "Sphere");
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
                 nullptr, nullptr, &m_radius, 0.5f, 10);
    y += kRowH + kSecPad;

    m_point.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH, "Point P (rel. Sphere)");

    y += kSecPad;
    DrawText("o Point P", (int)x + 4, (int)y, 16, RED);
    y += 16;
    DrawText("o Nearest Pt", (int)x + 4, (int)y, 16, PURPLE);
    y += 16;
    DrawText("-- Nearest Line", (int)x + 4, (int)y, 16, GREEN);
}
