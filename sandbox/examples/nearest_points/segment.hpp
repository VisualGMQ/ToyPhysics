#pragma once
#include "toy_physics/lowlevel/algorithm.hpp"
#include "example.hpp"
#include "examples/nearest_points/util.hpp"

class SegmentNearestPointExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_rotation = {0, 0, 0};
    float m_half_length = 4.f;
    SphericalPoint m_point;
};

inline void SegmentNearestPointExample::OnUpdate(float delta_time) {}

inline void SegmentNearestPointExample::OnRender3D(float delta_time) {
    Matrix rot = MatrixRotateXYZ({m_rotation.x * DEG2RAD,
                                  m_rotation.y * DEG2RAD,
                                  0});
    ::Vector3 dir = Vector3Normalize(Vector3Transform({0, 1, 0}, rot));
    ::Vector3 p1 = {m_center.x + dir.x * m_half_length,
                    m_center.y + dir.y * m_half_length,
                    m_center.z + dir.z * m_half_length};
    ::Vector3 p2 = {m_center.x - dir.x * m_half_length,
                    m_center.y - dir.y * m_half_length,
                    m_center.z - dir.z * m_half_length};

    DrawLine3D(p1, p2, BLUE);
    m_ctx.DrawSphere(p1, {0, 0, 0}, 0.2f, BLUE);
    m_ctx.DrawSphere(p2, {0, 0, 0}, 0.2f, BLUE);

    ::Vector3 p = m_point.ToCartesian(m_center);
    m_ctx.DrawSphere(p, {0, 0, 0}, 0.3f, RED);

    auto np = toy_physics::GetSegmentNearestPoint(ToVec3(p), ToVec3(p1), ToVec3(p2));
    ::Vector3 nearest = FromVec3(np);
    m_ctx.DrawSphere(nearest, {0, 0, 0}, 0.3f, PURPLE);
    DrawLine3D(p, nearest, GREEN);
}

inline void SegmentNearestPointExample::OnRender2D(float delta_time) {
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

    // --- Segment ---
    GuiGroupBox({x, y, kPanelW, kRowH * 6 + kSecPad + kPad}, "Segment");
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
    lbl({x, y, kLabelW, kCtrlH}, "Half Length");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_half_length, 0.5f, 10);
    y += kRowH;

    lbl({x, y, kLabelW, kCtrlH}, "Pitch");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_rotation.x, -90, 90);
    y += kRowH + kSecPad;

    m_point.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH, "Point P (rel. Segment)");

    y += kSecPad;
    DrawText("o Point P", (int)x + 4, (int)y, 16, RED);
    y += 16;
    DrawText("o Nearest Pt", (int)x + 4, (int)y, 16, PURPLE);
    y += 16;
    DrawText("-- Nearest Line", (int)x + 4, (int)y, 16, GREEN);
}
