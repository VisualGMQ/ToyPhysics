#pragma once
#include "toy_physics/lowlevel/algorithm.hpp"
#include "example.hpp"
#include "examples/raycast/util.hpp"

class AABBRaycastExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_half_extent = {3, 2, 1};
    RayControl m_ray;
    std::array<toy_physics::RaycastHit, 2> m_hits = {};
    uint16_t m_hit_count = 0;
};

inline void AABBRaycastExample::OnUpdate(float delta_time) {}

inline void AABBRaycastExample::OnRender3D(float delta_time) {
    ::Vector3 origin = m_ray.GetOrigin(m_center);
    ::Vector3 dir = m_ray.GetDirection();

    m_ctx.DrawBox(m_center, {0, 0, 0}, m_half_extent, WHITE);

    DrawRay(origin, dir, m_ray.m_len);
    auto flags = toy_physics::QueryFlags(toy_physics::QueryFlag::Default) |
                 toy_physics::QueryFlag::AllSide;
    m_hit_count = toy_physics::RaycastAABB(
        ToVec3(origin), ToVec3(dir), m_ray.m_len, ToVec3(m_center),
        ToVec3(m_half_extent), m_hits, flags);
    DrawRayHits(origin, m_hits, m_hit_count);
}

inline void AABBRaycastExample::OnRender2D(float delta_time) {
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

    GuiGroupBox({x, y, kPanelW, kRowH * 7 + kSecPad + kPad}, "AABB");
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
    y += kRowH + kSecPad;

    m_ray.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH);
    DrawRayInfo(m_hits, m_hit_count, x, y);
    DrawRayLegend(x, y);
}
