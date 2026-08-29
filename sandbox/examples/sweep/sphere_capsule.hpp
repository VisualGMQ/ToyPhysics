#pragma once
#include "example.hpp"
#include "examples/sweep/util.hpp"
#include "toy_physics/algorithm.hpp"
#include "toy_physics/gjk.hpp"

class SphereCapsuleSweepExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_c1 = {-6, 10, 0};
    float m_r1 = 1.f;
    ::Vector3 m_c2 = {4, 10, 0};
    ::Vector3 m_r2 = {0, 0, 0};
    float m_hh2 = 2.f;
    float m_rv2 = 1.5f;
    SweepControl m_sweep;
    int m_status = 0;
    toy_physics::real m_t = 0;
    ::Vector3 m_hit = {0, 0, 0};
    toy_physics::Vector3 m_nrm = toy_physics::Vector3::Zero();
};

inline void SphereCapsuleSweepExample::OnUpdate(float delta_time) {}

inline void SphereCapsuleSweepExample::OnRender3D(float delta_time) {
    ::Vector3 dir = m_sweep.GetDirection();

    Matrix rot = MatrixRotateXYZ({m_r2.x * DEG2RAD, m_r2.y * DEG2RAD,
                                  m_r2.z * DEG2RAD});
    ::Vector3 axis2 = Vector3Normalize(Vector3Transform({0, 1, 0}, rot));

    toy_physics::SweepHit hit;
    bool ok = toy_physics::SweepSphereCapsule(
        ToVec3(m_c1), m_r1, ToVec3(m_c2), ToVec3(axis2), m_hh2, m_rv2,
        ToVec3(dir), m_sweep.m_len, &hit);
    if (ok && hit.IsInitialOverlap()) {
        m_status = -1;
    } else if (ok) {
        m_status = 1;
        m_t = hit.m_dist / m_sweep.m_len;
        m_hit = FromVec3(hit.m_hit);
        m_nrm = hit.m_normal;
    } else {
        m_status = 0;
    }

    m_ctx.DrawCapsule(m_c2, m_r2, m_hh2 * 2.f, m_rv2, WHITE);
    m_ctx.DrawSphere(m_c1, {0, 0, 0}, m_r1, BLUE);
    if (m_status == 1) {
        m_ctx.DrawSphere(m_hit, {0, 0, 0}, m_r1, GREEN);
        DrawContact(m_hit, m_nrm);
    }
    DrawSweepDir(m_c1, dir, m_sweep.m_len);
}

inline void SphereCapsuleSweepExample::OnRender2D(float delta_time) {
    static constexpr float kPanelW = 300.f;
    static constexpr float kPanelX = 10.f;
    static constexpr float kLabelW = 110.f;
    static constexpr float kCtrlH = 22.f;
    static constexpr float kPad = 4.f;
    static constexpr float kRowH = kCtrlH + kPad;
    static constexpr float kSecPad = 12.f;

    float x = (float)GetScreenWidth() - kPanelW - kPanelX;
    float y = 90.f;

    auto lbl = [](Rectangle r, const char* text) {
        int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
        GuiLabel(r, text);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
    };

    GuiGroupBox({x, y, kPanelW, kRowH * 5 + kSecPad + kPad}, "Sphere (swept)");
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_c1.x, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_c1.y, 0, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_c1.z, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Radius");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_r1, 0.2f, 5);
    y += kRowH + kSecPad;

    GuiGroupBox({x, y, kPanelW, kRowH * 9 + kSecPad + kPad},
                "Capsule (static)");
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_c2.x, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_c2.y, 0, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_c2.z, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Half Height");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_hh2, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Radius");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_rv2, 0.2f, 5);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Pitch");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_r2.x, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Yaw");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_r2.y, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Roll");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_r2.z, -180, 180);
    y += kRowH + kSecPad;

    m_sweep.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH);
    DrawSweepInfo(m_status, m_t, m_nrm, x, y);
    DrawSweepLegend(x, y);
}
