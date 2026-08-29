#pragma once
#include "example.hpp"
#include "examples/sweep/util.hpp"
#include "toy_physics/algorithm.hpp"
#include "toy_physics/gjk.hpp"

class CapsuleOBBSweepExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_c1 = {-6, 10, 0};
    ::Vector3 m_r1 = {0, 0, 0};
    float m_hh1 = 2.f;
    float m_rv1 = 1.f;
    ::Vector3 m_c2 = {4, 10, 0};
    ::Vector3 m_r2 = {0, 0, 0};
    ::Vector3 m_half2 = {2, 2, 2};
    SweepControl m_sweep;
    int m_status = 0;
    toy_physics::real m_t = 0;
    toy_physics::Vector3 m_pos = toy_physics::Vector3::Zero();
    toy_physics::Vector3 m_nrm = toy_physics::Vector3::Zero();
    toy_physics::Vector3 m_w1 = toy_physics::Vector3::Zero();
    toy_physics::Vector3 m_w2 = toy_physics::Vector3::Zero();
};

inline void CapsuleOBBSweepExample::OnUpdate(float delta_time) {}

inline void CapsuleOBBSweepExample::OnRender3D(float delta_time) {
    ::Vector3 dir = m_sweep.GetDirection();

    Matrix rot1 = MatrixRotateXYZ({m_r1.x * DEG2RAD, m_r1.y * DEG2RAD,
                                   m_r1.z * DEG2RAD});
    ::Vector3 axis1 = Vector3Normalize(Vector3Transform({0, 1, 0}, rot1));
    Matrix rot2 = MatrixRotateXYZ({m_r2.x * DEG2RAD, m_r2.y * DEG2RAD,
                                   m_r2.z * DEG2RAD});
    std::array<toy_physics::Vector3, 3> axes = {
        ToVec3(Vector3Transform({1, 0, 0}, rot2)),
        ToVec3(Vector3Transform({0, 1, 0}, rot2)),
        ToVec3(Vector3Transform({0, 0, 1}, rot2)),
    };

    toy_physics::CapsuleSupportFunction s1(ToVec3(m_c1), ToVec3(axis1), m_hh1,
                                           m_rv1);
    toy_physics::OBBSupportFunction s2(ToVec3(m_c2), axes, ToVec3(m_half2));
    m_status = toy_physics::Sweep(s1, s2, ToVec3(dir), m_sweep.m_len, true, &m_t,
                                  &m_pos, &m_nrm, &m_w1, &m_w2);

    m_ctx.DrawBox(m_c2, m_r2, m_half2, WHITE);
    m_ctx.DrawCapsule(m_c1, m_r1, m_hh1 * 2.f, m_rv1, BLUE);
    if (m_status == 1) {
        m_ctx.DrawCapsule(
            Vector3Add(m_c1, Vector3Scale(dir, m_t * m_sweep.m_len)), m_r1,
            m_hh1 * 2.f, m_rv1, GREEN);
        DrawContact(Vector3Add(m_c1, FromVec3(m_pos)), m_nrm);
    }
    if (m_status == -1) {
        DrawWitness(FromVec3(m_w1), FromVec3(m_w2));
    }
    DrawSweepDir(m_c1, dir, m_sweep.m_len);
}

inline void CapsuleOBBSweepExample::OnRender2D(float delta_time) {
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

    GuiGroupBox({x, y, kPanelW, kRowH * 9 + kSecPad + kPad},
                "Capsule (swept)");
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
    lbl({x, y, kLabelW, kCtrlH}, "Half Height");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_hh1, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Radius");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_rv1, 0.2f, 5);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Pitch");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_r1.x, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Yaw");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_r1.y, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Roll");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_r1.z, -180, 180);
    y += kRowH + kSecPad;

    GuiGroupBox({x, y, kPanelW, kRowH * 10 + kSecPad + kPad}, "OBB (static)");
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
    lbl({x, y, kLabelW, kCtrlH}, "Extent X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_half2.x, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Extent Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_half2.y, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Extent Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_half2.z, 0.5f, 8);
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
