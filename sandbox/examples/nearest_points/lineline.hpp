#pragma once
#include "example.hpp"
#include "examples/nearest_points/util.hpp"
#include "toy_physics/algorithm.hpp"

class LineLineNearestPointExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_p = {0, 10, -3};
    ::Vector3 m_q = {0, 10, 3};
    float m_theta1 = 0, m_phi1 = 0;
    float m_theta2 = 30.f, m_phi2 = 0;
};

inline void LineLineNearestPointExample::OnUpdate(float delta_time) {}

inline ::Vector3 DirFromSpherical(float elev, float azim) {
    float e = elev * DEG2RAD;
    float a = azim * DEG2RAD;
    return {cosf(e) * sinf(a), sinf(e), cosf(e) * cosf(a)};
}

inline void LineLineNearestPointExample::OnRender3D(float delta_time) {
    ::Vector3 d1 = DirFromSpherical(m_theta1, m_phi1);
    ::Vector3 d2 = DirFromSpherical(m_theta2, m_phi2);

    float ext = 50.f;
    DrawLine3D({m_p.x - d1.x * ext, m_p.y - d1.y * ext, m_p.z - d1.z * ext},
               {m_p.x + d1.x * ext, m_p.y + d1.y * ext, m_p.z + d1.z * ext},
               BLUE);
    DrawLine3D({m_q.x - d2.x * ext, m_q.y - d2.y * ext, m_q.z - d2.z * ext},
               {m_q.x + d2.x * ext, m_q.y + d2.y * ext, m_q.z + d2.z * ext},
               BLUE);
    DrawSphere(m_p, 0.2f, BLUE);
    DrawSphere(m_q, 0.2f, BLUE);

    auto [c1, c2] = toy_physics::GetLineLineNearestPoints(
        ToVec3(m_p), ToVec3(d1), ToVec3(m_q), ToVec3(d2));
    ::Vector3 nc1 = FromVec3(c1);
    ::Vector3 nc2 = FromVec3(c2);
    m_ctx.DrawSphere(nc1, {0, 0, 0}, 0.3f, WHITE);
    m_ctx.DrawSphere(nc2, {0, 0, 0}, 0.3f, WHITE);
    DrawLine3D(nc1, nc2, GREEN);
}

inline void LineLineNearestPointExample::OnRender2D(float delta_time) {
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

    auto lineGUI = [&](const char* title, ::Vector3& pt, float& elev,
                       float& azim) {
        GuiGroupBox({x, y, kPanelW, kRowH * 6 + kSecPad + kPad}, title);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Point X");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &pt.x, -20, 20);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Point Y");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &pt.y, -20, 20);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Point Z");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &pt.z, -20, 20);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Dir Elev");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &elev, -90, 90);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Dir Azim");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &azim, 0, 360);
        y += kRowH + kSecPad;
    };

    lineGUI("Line A", m_p, m_theta1, m_phi1);
    lineGUI("Line B", m_q, m_theta2, m_phi2);

    DrawText("o Lines (Blue)", (int)x + 4, (int)y, 12, BLUE);
    y += 16;
    DrawText("o Nearest Pts (White)", (int)x + 4, (int)y, 12, WHITE);
    y += 16;
    DrawText("-- Nearest Line (Green)", (int)x + 4, (int)y, 12, GREEN);
}
