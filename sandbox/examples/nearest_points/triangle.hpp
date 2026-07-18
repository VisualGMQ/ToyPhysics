#pragma once
#include "example.hpp"
#include "examples/nearest_points/util.hpp"
#include "toy_physics/algorithm.hpp"

class TriangleNearestPointExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_rotation = {0, 0, 0};
    float m_size = 3.f;
    float m_v_scale[3] = {1, 1, 1};
    SphericalPoint m_point;
};

inline void TriangleNearestPointExample::OnUpdate(float delta_time) {}

inline void TriangleNearestPointExample::OnRender3D(float delta_time) {
    ::Vector3 local[3] = {
        {0, 1, 0}, {-0.866f, -0.5f, 0}, {0.866f, -0.5f, 0},
    };

    Matrix rot = MatrixRotateXYZ({m_rotation.x * DEG2RAD,
                                  m_rotation.y * DEG2RAD,
                                  m_rotation.z * DEG2RAD});
    ::Vector3 world[3];
    for (int i = 0; i < 3; ++i) {
        ::Vector3 l = {local[i].x * m_v_scale[i] * m_size,
                       local[i].y * m_v_scale[i] * m_size,
                       local[i].z * m_v_scale[i] * m_size};
        world[i] = Vector3Add(m_center, Vector3Transform(l, rot));
    }

    DrawTriangle3D(world[0], world[1], world[2],
                   {0, 0, 255, 80});
    DrawTriangle3D(world[1], world[0], world[2],
                   {0, 0, 255, 80});
    for (int i = 0; i < 3; ++i) {
        DrawLine3D(world[i], world[(i + 1) % 3], BLUE);
        m_ctx.DrawSphere(world[i], {0, 0, 0}, 0.2f, BLUE);
    }

    ::Vector3 p = m_point.ToCartesian(m_center);
    m_ctx.DrawSphere(p, {0, 0, 0}, 0.3f, RED);

    auto np = toy_physics::GetTriangleNearestPoint(ToVec3(p), ToVec3(world[0]),
                                                    ToVec3(world[1]),
                                                    ToVec3(world[2]));
    ::Vector3 nearest = FromVec3(np);
    m_ctx.DrawSphere(nearest, {0, 0, 0}, 0.3f, PURPLE);
    DrawLine3D(p, nearest, GREEN);
}

inline void TriangleNearestPointExample::OnRender2D(float delta_time) {
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

    // --- Triangle ---
    GuiGroupBox({x, y, kPanelW, kRowH * 8 + kSecPad + kPad}, "Triangle");
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
    lbl({x, y, kLabelW, kCtrlH}, "Size");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_size, 0.5f, 10);
    y += kRowH;

    lbl({x, y, kLabelW, kCtrlH}, "Pitch");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_rotation.x, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Yaw");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_rotation.y, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Roll");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_rotation.z, -180, 180);
    y += kRowH + kSecPad;

    // --- Vertex Scales ---
    GuiGroupBox({x, y, kPanelW, kRowH * 4 + kSecPad + kPad}, "Vertex Scales");
    y += kRowH;

    for (int i = 0; i < 3; ++i) {
        char label[4];
        snprintf(label, sizeof(label), "V%d", i);
        lbl({x, y, kLabelW, kCtrlH}, label);
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &m_v_scale[i], 0.1f, 3);
        y += kRowH;
    }
    y += kSecPad;

    m_point.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH, "Point P");
    y += kSecPad;
    DrawText("o Point P", (int)x + 4, (int)y, 12, RED);
    y += 16;
    DrawText("o Nearest Pt", (int)x + 4, (int)y, 12, PURPLE);
    y += 16;
    DrawText("-- Nearest Line", (int)x + 4, (int)y, 12, GREEN);
}
