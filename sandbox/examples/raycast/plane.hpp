#pragma once
#include "example.hpp"
#include "examples/raycast/util.hpp"
#include "toy_physics/algorithm.hpp"

class PlaneRaycastExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 0, 0};
    float m_pitch = 0.f;
    float m_yaw = 0.f;
    RayControl m_ray;
    bool m_hit = false;
    toy_physics::RaycastHit m_plane_hit = {};
};

inline void PlaneRaycastExample::OnUpdate(float delta_time) {}

inline void PlaneRaycastExample::OnRender3D(float delta_time) {
    Matrix rot = MatrixRotateXYZ({m_pitch * DEG2RAD, m_yaw * DEG2RAD, 0});
    ::Vector3 n = Vector3Normalize(Vector3Transform({0, 1, 0}, rot));

    ::Vector3 up = {0, 1, 0};
    if (fabsf(Vector3DotProduct(n, up)) > 0.999f) up = {1, 0, 0};
    ::Vector3 t = Vector3Normalize(Vector3CrossProduct(n, up));
    ::Vector3 b = Vector3CrossProduct(n, t);

    float h = 8.f;
    ::Vector3 c0 = Vector3Add(Vector3Add(m_center, Vector3Scale(t, -h)),
                              Vector3Scale(b, -h));
    ::Vector3 c1 = Vector3Add(Vector3Add(m_center, Vector3Scale(t, h)),
                              Vector3Scale(b, -h));
    ::Vector3 c2 = Vector3Add(Vector3Add(m_center, Vector3Scale(t, h)),
                              Vector3Scale(b, h));
    ::Vector3 c3 = Vector3Add(Vector3Add(m_center, Vector3Scale(t, -h)),
                              Vector3Scale(b, h));

    Color face = {0, 0, 255, 60};
    DrawTriangle3D(c0, c1, c2, face);
    DrawTriangle3D(c0, c2, c3, face);
    DrawLine3D(c0, c1, GRAY);
    DrawLine3D(c1, c2, GRAY);
    DrawLine3D(c2, c3, GRAY);
    DrawLine3D(c3, c0, GRAY);
    DrawLine3D(m_center, Vector3Add(m_center, Vector3Scale(n, 3.f)), BLUE);

    ::Vector3 origin = m_ray.GetOrigin(m_center);
    ::Vector3 dir = m_ray.GetDirection();
    DrawRay(origin, dir, m_ray.m_len);

    std::array<toy_physics::RaycastHit, 1> hit = {};
    m_hit = toy_physics::RaycastPlane(ToVec3(origin), ToVec3(dir), m_ray.m_len,
                                      ToVec3(m_center), ToVec3(n), hit,
                                      toy_physics::QueryFlag::Default);
    m_plane_hit = hit[0];
    if (m_hit) {
        ::Vector3 p = FromVec3(m_plane_hit.m_hit);
        DrawSphere(p, 0.25f, RED);
        DrawLine3D(p, Vector3Add(p, Vector3Scale(FromVec3(m_plane_hit.m_normal),
                                                 1.2f)),
                   GREEN);
    }
}

inline void PlaneRaycastExample::OnRender2D(float delta_time) {
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

    GuiGroupBox({x, y, kPanelW, kRowH * 6 + kSecPad + kPad}, "Plane");
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
    lbl({x, y, kLabelW, kCtrlH}, "Normal Pitch");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_pitch, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Normal Yaw");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_yaw, -180, 180);
    y += kRowH + kSecPad;

    m_ray.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH);

    if (m_hit) {
        char buf[96];
        snprintf(buf, sizeof(buf), "PLANE HIT: dist=%.2f", m_plane_hit.m_dist);
        DrawText(buf, (int)x + 4, (int)y, 18, GREEN);
    } else {
        DrawText("MISS", (int)x + 4, (int)y, 26, RED);
    }
    y += 32;
    DrawRayLegend(x, y);
}
