#pragma once
#include "example.hpp"
#include "examples/sweep/util.hpp"
#include "toy_physics/algorithm.hpp"
#include "toy_physics/gjk.hpp"

class ConvexConvexSweepExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_c1 = {-6, 10, 0};
    ::Vector3 m_r1 = {0, 0, 0};
    float m_radius1 = 3.f;
    float m_hh1 = 2.5f;
    ::Vector3 m_c2 = {4, 10, 0};
    ::Vector3 m_r2 = {0, 0, 0};
    float m_radius2 = 3.f;
    float m_hh2 = 2.5f;
    SweepControl m_sweep;
    int m_status = 0;
    toy_physics::real m_t = 0;
    toy_physics::Vector3 m_pos = toy_physics::Vector3::Zero();
    toy_physics::Vector3 m_nrm = toy_physics::Vector3::Zero();
    toy_physics::Vector3 m_w1 = toy_physics::Vector3::Zero();
    toy_physics::Vector3 m_w2 = toy_physics::Vector3::Zero();
};

inline void ConvexConvexSweepExample::OnUpdate(float delta_time) {}

inline void ConvexConvexSweepExample::OnRender3D(float delta_time) {
    ::Vector3 dir = m_sweep.GetDirection();

    std::array<::Vector3, 12> world1, world2;
    ComputeHexPrismWorld(m_c1, m_r1, m_radius1, m_hh1, world1);
    ComputeHexPrismWorld(m_c2, m_r2, m_radius2, m_hh2, world2);
    std::array<toy_physics::Vector3, 12> pts1, pts2;
    for (int i = 0; i < 12; ++i) {
        pts1[i] = ToVec3(world1[i]);
        pts2[i] = ToVec3(world2[i]);
    }

    toy_physics::PolygonSupportFunction s1(pts1);
    toy_physics::PolygonSupportFunction s2(pts2);
    m_status = toy_physics::Sweep(s1, s2, ToVec3(dir), m_sweep.m_len, true, &m_t,
                                  &m_pos, &m_nrm, &m_w1, &m_w2);

    DrawHexPrism(world2, BLUE, {0, 0, 255, 76});
    DrawHexPrism(world1, BLUE, {0, 0, 255, 76});
    if (m_status == 1) {
        std::array<::Vector3, 12> moved;
        ComputeHexPrismWorld(
            Vector3Add(m_c1, Vector3Scale(dir, m_t * m_sweep.m_len)), m_r1,
            m_radius1, m_hh1, moved);
        DrawHexPrism(moved, GREEN, {0, 228, 48, 76});
        DrawContact(Vector3Add(m_c1, FromVec3(m_pos)), m_nrm);
    }
    if (m_status == -1) {
        DrawWitness(FromVec3(m_w1), FromVec3(m_w2));
    }
    DrawSweepDir(m_c1, dir, m_sweep.m_len);
}

inline void ConvexConvexSweepExample::OnRender2D(float delta_time) {
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
                "Convex A (swept)");
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
                 &m_radius1, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Half Height");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_hh1, 0.5f, 8);
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

    GuiGroupBox({x, y, kPanelW, kRowH * 9 + kSecPad + kPad},
                "Convex B (static)");
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
    lbl({x, y, kLabelW, kCtrlH}, "Radius");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_radius2, 0.5f, 8);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Half Height");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_hh2, 0.5f, 8);
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
