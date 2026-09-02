#pragma once
#include "toy_physics/lowlevel/algorithm.hpp"
#include "example.hpp"
#include "examples/nearest_points/util.hpp"

class TetrahedronNearestPointExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_center = {0, 10, 0};
    ::Vector3 m_rotation = {0, 0, 0};
    float m_size = 3.f;
    float m_v_scale[4] = {1, 1, 1, 1};
    SphericalPoint m_point;
};

inline void TetrahedronNearestPointExample::OnUpdate(float delta_time) {}

inline void TetrahedronNearestPointExample::OnRender3D(float delta_time) {
    // Base tetrahedron vertices in local space (regular tetrahedron)
    ::Vector3 local[4] = {
        {1, 1, 1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1},
    };

    Matrix rot = MatrixRotateXYZ({m_rotation.x * DEG2RAD,
                                  m_rotation.y * DEG2RAD,
                                  m_rotation.z * DEG2RAD});
    ::Vector3 world[4];
    for (int i = 0; i < 4; ++i) {
        ::Vector3 l = {local[i].x * m_v_scale[i] * m_size,
                       local[i].y * m_v_scale[i] * m_size,
                       local[i].z * m_v_scale[i] * m_size};
        world[i] = Vector3Add(m_center, Vector3Transform(l, rot));
    }

    // Draw filled faces
    Color faceColor = {0, 0, 255, 80};
    int faces[4][3] = {
        {1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1},
    };
    for (int i = 0; i < 4; ++i) {
        DrawTriangle3D(world[faces[i][0]], world[faces[i][1]],
                       world[faces[i][2]], faceColor);
    }

    // Draw edges
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            DrawLine3D(world[i], world[j], BLUE);
        }
    }
    for (int i = 0; i < 4; ++i) {
        m_ctx.DrawSphere(world[i], {0, 0, 0}, 0.2f, BLUE);
    }

    ::Vector3 p = m_point.ToCartesian(m_center);
    m_ctx.DrawSphere(p, {0, 0, 0}, 0.3f, RED);

    std::array<toy_physics::Vector3, 4> pts{
        ToVec3(world[0]), ToVec3(world[1]),
        ToVec3(world[2]), ToVec3(world[3])
    };

    auto np = toy_physics::GetTetrahedronNearestPoint(ToVec3(p), pts);
    ::Vector3 nearest = FromVec3(np);
    m_ctx.DrawSphere(nearest, {0, 0, 0}, 0.3f, PURPLE);
    DrawLine3D(p, nearest, GREEN);
}

inline void TetrahedronNearestPointExample::OnRender2D(float delta_time) {
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

    // --- Tetrahedron ---
    GuiGroupBox({x, y, kPanelW, kRowH * 8 + kSecPad + kPad}, "Tetrahedron");
    y += kRowH;

    lbl({x, y, kLabelW, kCtrlH}, "Center X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.x, -30, 30);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.y, -30, 30);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_center.z, -30, 30);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Size");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                 nullptr, nullptr, &m_size, 0.1f, 15);
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
    GuiGroupBox({x, y, kPanelW, kRowH * 5 + kSecPad + kPad}, "Vertex Scales");
    y += kRowH;

    for (int i = 0; i < 4; ++i) {
        char label[4];
        snprintf(label, sizeof(label), "V%d", i);
        lbl({x, y, kLabelW, kCtrlH}, label);
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH},
                     nullptr, nullptr, &m_v_scale[i], 0.0f, 5);
        y += kRowH;
    }
    y += kSecPad;

    m_point.RenderGUI(x, y, kPanelW, kLabelW, kCtrlH, "Point P");
    y += kSecPad;
    DrawText("o Point P", (int)x + 4, (int)y, 16, RED);
    y += 16;
    DrawText("o Nearest Pt", (int)x + 4, (int)y, 16, PURPLE);
    y += 16;
    DrawText("-- Nearest Line", (int)x + 4, (int)y, 16, GREEN);
}
