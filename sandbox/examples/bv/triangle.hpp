#pragma once
#include "toy_physics/lowlevel/bv.hpp"
#include "example.hpp"
#include "examples/bv/util.hpp"
#include <array>

class TriangleBVExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_v1 = {-2, -1, 0};
    ::Vector3 m_v2 = {2, -1, 0};
    ::Vector3 m_v3 = {0, 2, 1};
};

inline void TriangleBVExample::OnUpdate(float delta_time) {}

inline void TriangleBVExample::OnRender3D(float delta_time) {
    DrawTriangle3D(m_v1 + m_center, m_v2 + m_center, m_v3 + m_center,
                   LIGHTGRAY);

    std::array<toy_physics::Vector3, 3> verts = {
        ToVec3(m_v1), ToVec3(m_v2), ToVec3(m_v3)};
    auto bv = toy_physics::BuildBV(verts);
    bv.m_min += ToVec3(m_center);
    bv.m_max += ToVec3(m_center);
    DrawAABBWire(bv);
}

inline void TriangleBVExample::OnRender2D(float delta_time) {
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

    auto slider3 = [&](float& yy, const char* title, ::Vector3& v) {
        lbl({x, yy, kLabelW, kCtrlH}, title);
        GuiSliderBar({x + kLabelW, yy, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &v.x, -5, 5);
        yy += kRowH;
        GuiSliderBar({x + kLabelW, yy, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &v.y, -5, 5);
        yy += kRowH;
        GuiSliderBar({x + kLabelW, yy, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &v.z, -5, 5);
        yy += kRowH;
    };

    GuiGroupBox({x, y, kPanelW, kRowH * 13 + 30}, "Triangle");
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
    slider3(y, "V1", m_v1);
    slider3(y, "V2", m_v2);
    slider3(y, "V3", m_v3);
}
