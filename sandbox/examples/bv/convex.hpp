#pragma once
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/lowlevel/bv.hpp"
#include "example.hpp"
#include "examples/bv/util.hpp"
#include <array>

class ConvexBVExample : public IExample {
public:
    ConvexBVExample(Context& ctx, const std::string& name);
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_rotation = {0, 0, 0};
    toy_physics::ConvexFactory m_factory;
    toy_physics::ConvexData* m_data = nullptr;
};

inline ConvexBVExample::ConvexBVExample(Context& ctx,
                                        const std::string& name)
    : IExample{ctx, name} {
    static constexpr float kRadius = 3.f;
    static constexpr float kHalfHeight = 2.5f;

    m_data = m_factory.Create();
    for (int k = 0; k < 6; ++k) {
        float ang = k * 60.f * DEG2RAD;
        m_data->m_vertices.emplace_back(kRadius * cosf(ang), kHalfHeight,
                                        kRadius * sinf(ang));
    }
    for (int k = 0; k < 6; ++k) {
        float ang = k * 60.f * DEG2RAD;
        m_data->m_vertices.emplace_back(kRadius * cosf(ang), -kHalfHeight,
                                        kRadius * sinf(ang));
    }
}

inline void ConvexBVExample::OnUpdate(float delta_time) {}

inline void ConvexBVExample::OnRender3D(float delta_time) {
    toy_physics::Quaternion q = EulerToQuat(m_rotation);
    auto tf = [&](toy_physics::Vector3 v) {
        return FromVec3(q * v + ToVec3(m_center));
    };

    // hexagonal prism: 6 side quads + top/bottom fans
    std::array<::Vector3, 12> world;
    for (size_t i = 0; i < m_data->m_vertices.size(); i++) {
        world[i] = tf(m_data->m_vertices[i]);
    }

    Color face = {0, 0, 255, 76};
    for (int k = 0; k < 6; ++k) {
        int k1 = (k + 1) % 6;
        DrawTriangle3D(world[k], world[k1], world[k1 + 6], face);
        DrawTriangle3D(world[k], world[k1 + 6], world[k + 6], face);
    }
    for (int k = 1; k < 5; ++k) {
        DrawTriangle3D(world[0], world[k + 1], world[k], face);
        DrawTriangle3D(world[6], world[6 + k], world[6 + k + 1], face);
    }
    for (int k = 0; k < 6; ++k) {
        int k1 = (k + 1) % 6;
        DrawLine3D(world[k], world[k1], BLUE);
        DrawLine3D(world[k + 6], world[k1 + 6], BLUE);
        DrawLine3D(world[k], world[k + 6], BLUE);
    }

    toy_physics::ConvexHullGeometry hull{*m_data};
    auto bv = toy_physics::BuildBV(hull, q);
    bv.m_min += ToVec3(m_center);
    bv.m_max += ToVec3(m_center);
    DrawAABBWire(bv);
}

inline void ConvexBVExample::OnRender2D(float delta_time) {
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

    GuiGroupBox({x, y, kPanelW, kRowH * 4 + 30}, "Convex Hull (Hex Prism)");
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
    RenderRotationGUI(x, y, kPanelW, kLabelW, kCtrlH, m_rotation);
}
