#pragma once
#include "example.hpp"
#include "examples/nearest_points/util.hpp"
#include "toy_physics/algorithm.hpp"

class OBBNearestPointExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_half_extent = {3, 2, 1};
    ::Vector3 m_rotation = {0, 0, 0};
    SphericalPoint m_point;
};

inline void OBBNearestPointExample::OnUpdate(float delta_time) {}

inline void OBBNearestPointExample::OnRender3D(float delta_time) {
    m_ctx.DrawBox(m_center, m_rotation, m_half_extent, BLUE);

    ::Vector3 p = m_point.ToCartesian(m_center);
    m_ctx.DrawSphere(p, {0, 0, 0}, 0.3f, RED);

    Matrix rot = MatrixRotateXYZ({m_rotation.x * DEG2RAD,
                                  m_rotation.y * DEG2RAD,
                                  m_rotation.z * DEG2RAD});
    std::array<toy_physics::Vector3, 3> axes = {
        ToVec3(Vector3Transform({1, 0, 0}, rot)),
        ToVec3(Vector3Transform({0, 1, 0}, rot)),
        ToVec3(Vector3Transform({0, 0, 1}, rot)),
    };

    auto np = toy_physics::GetOBBNearestPoint(ToVec3(p), ToVec3(m_center),
                                               axes, ToVec3(m_half_extent));
    ::Vector3 nearest = FromVec3(np);
    m_ctx.DrawSphere(nearest, {0, 0, 0}, 0.3f, PURPLE);
    DrawLine3D(p, nearest, GREEN);
}

inline void OBBNearestPointExample::OnRender2D(float delta_time) {
    static constexpr float kPanelW = 240.f;
    static constexpr float kPanelX = 10.f;
    static constexpr float kLabelW = 80.f;
    static constexpr float kCtrlH = 20.f;
    static constexpr float kPad = 4.f;
    static constexpr float kRowH = kCtrlH + kPad;
    static constexpr float kSecPad = 12.f;

    float x = (float)GetScreenWidth() - kPanelW - kPanelX;
    float y = 100.f;

    auto lbl = [](Rectangle r, const char* text) {
        int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
        GuiLabel(r, text);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
    };

    // --- OBB ---
    GuiGroupBox({x, y, kPanelW, kRowH * 10 + kSecPad + kPad}, "OBB");
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
    lbl({x, y, kLabelW, kCtrlH}, "Extent X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_half_extent.x, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Extent Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_half_extent.y, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Extent Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_half_extent.z, 0.5f, 8);
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

    m_point.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH, "Point P (rel. OBB)");

    y += kSecPad;
    DrawText("o Point P", (int)x + 4, (int)y, 12, RED);
    y += 16;
    DrawText("o Nearest Pt", (int)x + 4, (int)y, 12, PURPLE);
    y += 16;
    DrawText("-- Nearest Line", (int)x + 4, (int)y, 12, GREEN);
}
