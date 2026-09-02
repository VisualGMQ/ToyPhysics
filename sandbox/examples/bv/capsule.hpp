#pragma once
#include "toy_physics/lowlevel/bv.hpp"
#include "example.hpp"
#include "examples/bv/util.hpp"

class CapsuleBVExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_rotation = {0, 0, 0};
    float m_radius = 2.f;
    float m_half_height = 4.f;
};

inline void CapsuleBVExample::OnUpdate(float delta_time) {}

inline void CapsuleBVExample::OnRender3D(float delta_time) {
    m_ctx.DrawCapsule(m_center, m_rotation, m_half_height * 2.f, m_radius);

    toy_physics::CapsuleGeometry capsule{m_radius, m_half_height};
    auto bv = toy_physics::BuildBV(capsule, EulerToQuat(m_rotation));
    bv.m_min += ToVec3(m_center);
    bv.m_max += ToVec3(m_center);
    DrawAABBWire(bv);
}

inline void CapsuleBVExample::OnRender2D(float delta_time) {
    static constexpr float kPanelW = 300.f;
    static constexpr float kPanelX = 10.f;
    static constexpr float kLabelW = 110.f;
    static constexpr float kCtrlH = 28.f;
    static constexpr float kPad = 6.f;
    static constexpr float kRowH = kCtrlH + kPad;

    float x = (float)GetScreenWidth() - kPanelW - kPanelX;
    float y = 100.f;

    auto lbl = [](Rectangle r, const char* text) {
        int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
        GuiLabel(r, text);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
    };

    GuiGroupBox({x, y, kPanelW, kRowH * 6 + 30}, "Capsule");
    y += kRowH;

    lbl({x, y, kLabelW, kCtrlH}, "Center X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_center.x, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_center.y, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_center.z, -20, 20);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Radius");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_radius, 0.5f, 5);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Half Height");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &m_half_height, 0.5f, 8);
    y += kRowH;
    RenderRotationGUI(x, y, kPanelW, kLabelW, kCtrlH, m_rotation);
}
