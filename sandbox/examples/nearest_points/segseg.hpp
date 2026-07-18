#pragma once
#include "example.hpp"
#include "examples/nearest_points/util.hpp"
#include "toy_physics/algorithm.hpp"

class SegSegNearestPointExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_c1 = {0, 10, -3};
    ::Vector3 m_r1 = {0, 0, 0};
    float m_half1 = 4.f;
    ::Vector3 m_c2 = {0, 10, 3};
    ::Vector3 m_r2 = {30, 0, 0};
    float m_half2 = 4.f;
};

inline void SegSegNearestPointExample::OnUpdate(float delta_time) {}

inline void MakeSegEndpoints(::Vector3 c, ::Vector3 rot, float half,
                              ::Vector3& outA, ::Vector3& outB) {
    Matrix m = MatrixRotateXYZ(
        {rot.x * DEG2RAD, rot.y * DEG2RAD, 0});
    ::Vector3 dir = Vector3Normalize(Vector3Transform({0, 1, 0}, m));
    outA = {c.x + dir.x * half, c.y + dir.y * half, c.z + dir.z * half};
    outB = {c.x - dir.x * half, c.y - dir.y * half, c.z - dir.z * half};
}

inline void SegSegNearestPointExample::OnRender3D(float delta_time) {
    ::Vector3 p1, p2, q1, q2;
    MakeSegEndpoints(m_c1, m_r1, m_half1, p1, p2);
    MakeSegEndpoints(m_c2, m_r2, m_half2, q1, q2);

    DrawLine3D(p1, p2, BLUE);
    DrawSphere(p1, 0.2f, BLUE);
    DrawSphere(p2, 0.2f, BLUE);
    DrawLine3D(q1, q2, BLUE);
    DrawSphere(q1, 0.2f, BLUE);
    DrawSphere(q2, 0.2f, BLUE);

    auto [c1, c2] = toy_physics::GetSegSegNearestPoints(
        ToVec3(p1), ToVec3(p2), ToVec3(q1), ToVec3(q2));
    ::Vector3 nc1 = FromVec3(c1);
    ::Vector3 nc2 = FromVec3(c2);
    m_ctx.DrawSphere(nc1, {0, 0, 0}, 0.3f, WHITE);
    m_ctx.DrawSphere(nc2, {0, 0, 0}, 0.3f, WHITE);
    DrawLine3D(nc1, nc2, GREEN);
}

inline void SegSegNearestPointExample::OnRender2D(float delta_time) {
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

    auto segGUI = [&](const char* title, ::Vector3& c, ::Vector3& r,
                      float& half) {
        GuiGroupBox({x, y, kPanelW, kRowH * 7 + kSecPad + kPad}, title);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Center X");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &c.x, -20, 20);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Center Y");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &c.y, -20, 20);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Center Z");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &c.z, -20, 20);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Half Len");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &half, 0.5f, 10);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Pitch");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &r.x, -90, 90);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Yaw");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &r.y, 0, 360);
        y += kRowH + kSecPad;
    };

    segGUI("Segment A", m_c1, m_r1, m_half1);
    segGUI("Segment B", m_c2, m_r2, m_half2);

    DrawText("o Segments (Blue)", (int)x + 4, (int)y, 12, BLUE);
    y += 16;
    DrawText("o Nearest Pts (White)", (int)x + 4, (int)y, 12, WHITE);
    y += 16;
    DrawText("-- Nearest Line (Green)", (int)x + 4, (int)y, 12, GREEN);
}
