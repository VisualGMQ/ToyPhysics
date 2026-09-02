#pragma once
#include "toy_physics/lowlevel/algorithm.hpp"
#include "toy_physics/lowlevel/epa.hpp"
#include "example.hpp"
#include "examples/nearest_points/util.hpp"

class MtdTetrahedronsExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    ::Vector3 m_c1 = {0, 10, -1}, m_c2 = {0, 10, 0};
    ::Vector3 m_c1_moved = {0, 10, -1};  // m_c1 after translating along MTD
    ::Vector3 m_r1 = {0, 0, 0}, m_r2 = {0, 0, 0};
    float m_size1 = 3.f, m_size2 = 2.5f;
    float m_scale1[4] = {1, 1, 1, 1};
    float m_scale2[4] = {1, 1, 1, 1};
    float m_success_timer = 0;
    bool m_hit = false;
    bool m_has_points = false;
    toy_physics::Vector3 m_p1{toy_physics::Vector3::Zero()};
    toy_physics::Vector3 m_p2{toy_physics::Vector3::Zero()};
    toy_physics::Vector3 m_penetration{toy_physics::Vector3::Zero()};
};

inline void MtdTetrahedronsExample::OnUpdate(float delta_time) {
    if (m_success_timer > 0) m_success_timer -= delta_time;
}

inline ::Vector3 ComputeTetVertex2(::Vector3 c, ::Vector3 rot, float size,
                                   float scale, ::Vector3 local) {
    ::Vector3 l = {local.x * scale * size, local.y * scale * size,
                   local.z * scale * size};
    Matrix m =
        MatrixRotateXYZ({rot.x * DEG2RAD, rot.y * DEG2RAD, rot.z * DEG2RAD});
    return Vector3Add(c, Vector3Transform(l, m));
}

inline void MtdTetrahedronsExample::OnRender3D(float delta_time) {
    ::Vector3 local[4] = {
        { 1,  1,  1},
        { 1, -1, -1},
        {-1,  1, -1},
        {-1, -1,  1}
    };

    ::Vector3 w1[4], w1_moved[4], w2[4];
    for (int i = 0; i < 4; ++i) {
        w1[i] = ComputeTetVertex2(m_c1, m_r1, m_size1, m_scale1[i], local[i]);
        w1_moved[i] =
            ComputeTetVertex2(m_c1_moved, m_r1, m_size1, m_scale1[i], local[i]);
        w2[i] = ComputeTetVertex2(m_c2, m_r2, m_size2, m_scale2[i], local[i]);
    }

    Color c = m_hit ? RED : BLUE;
    Color faceColor = m_hit ? Color{255, 0, 0, 76} : Color{0, 0, 255, 76};
    Color movedFaceColor = Color{0, 228, 48, 76};

    int faces[4][3] = {
        {1, 2, 3},
        {0, 3, 2},
        {0, 1, 3},
        {0, 2, 1}
    };

    auto drawTet = [&](::Vector3 w[4], Color edgeColor, Color face) {
        for (int i = 0; i < 4; ++i) {
            DrawTriangle3D(w[faces[i][0]], w[faces[i][1]], w[faces[i][2]],
                           face);
        }
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j) DrawLine3D(w[i], w[j], edgeColor);
        for (int i = 0; i < 4; ++i) {
            ::DrawSphere(w[i], 0.2f, edgeColor);
        }
    };

    drawTet(w1, c, faceColor);
    drawTet(w2, c, faceColor);
    drawTet(w1_moved, GREEN, movedFaceColor);

    // Draw penetration depth & witness points
    if (!m_has_points) return;

    ::Vector3 rp1 = FromVec3(m_p1);
    ::Vector3 rp2 = FromVec3(m_p2);
    DrawCylinderEx(rp1, rp2, 0.15f, 0.15f, 12, GREEN);
    ::DrawSphere(rp1, 0.25f, GREEN);
    ::DrawSphere(rp2, 0.25f, DARKGREEN);
}

inline void MtdTetrahedronsExample::OnRender2D(float delta_time) {
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

    auto tetGUI = [&](const char* title, ::Vector3& c, ::Vector3& r, float& sz,
                      float* sc) {
        GuiGroupBox({x, y, kPanelW, kRowH * 13 + kSecPad + kPad}, title);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Center X");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &c.x, -15, 15);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Center Y");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &c.y, 0, 20);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Center Z");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &c.z, -15, 15);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Size");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &sz, 1.f, 8);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Pitch");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &r.x, -180, 180);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Yaw");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &r.y, -180, 180);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Roll");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &r.z, -180, 180);
        y += kRowH + kSecPad;
        for (int i = 0; i < 4; ++i) {
            char buf[8];
            snprintf(buf, sizeof(buf), "V%d", i);
            lbl({x, y, kLabelW, kCtrlH}, buf);
            GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                         nullptr, &sc[i], 0.3f, 2.5f);
            y += kRowH;
        }
        y += kSecPad;
    };

    tetGUI("Tetrahedron A", m_c1, m_r1, m_size1, m_scale1);
    tetGUI("Tetrahedron B", m_c2, m_r2, m_size2, m_scale2);

    // Compute penetration depth & witness points
    ::Vector3 local[4] = {
        { 1,  1,  1},
        { 1, -1, -1},
        {-1,  1, -1},
        {-1, -1,  1}
    };
    ::Vector3 w1[4], w2[4];
    for (int i = 0; i < 4; ++i) {
        w1[i] = ComputeTetVertex2(m_c1, m_r1, m_size1, m_scale1[i], local[i]);
        w2[i] = ComputeTetVertex2(m_c2, m_r2, m_size2, m_scale2[i], local[i]);
    }
    std::array<toy_physics::Vector3, 4> tv1, tv2;
    for (int i = 0; i < 4; ++i) {
        tv1[i] = ToVec3(w1[i]);
        tv2[i] = ToVec3(w2[i]);
    }
    toy_physics::PolygonSupportFunction ps1(tv1), ps2(tv2);
    m_has_points = false;
    m_hit =
        toy_physics::CalcPenetrationDepth(ps1, ps2, m_penetration, m_p1, m_p2,
                                          toy_physics::kDefaultRealTolerance);
    m_has_points = m_hit;

    // Translate tetrahedron 1 along the MTD so the two become separated
    if (m_hit) {
        m_c1_moved = Vector3Add(m_c1, FromVec3(-m_penetration * 1.001f));
    } else {
        m_c1_moved = m_c1;
    }

    y += 4;
    if (m_hit) {
        DrawText("INTERSECT", (int)x + 4, (int)y, 26, RED);
        y += 24;
        char buf[128];
        snprintf(buf, sizeof(buf), "Pen: (%.2f, %.2f, %.2f) len=%.2f",
                 m_penetration.x(), m_penetration.y(), m_penetration.z(),
                 m_penetration.norm());
        DrawText(buf, (int)x + 4, (int)y, 18, GREEN);
    } else {
        DrawText("SEPARATED", (int)x + 4, (int)y, 26, GREEN);
    }

    y += 32;
    if (GuiButton({x + 4, y, kPanelW - 8, kCtrlH}, "Copy Info")) {
        char buf[1024];
        snprintf(
            buf, sizeof(buf),
            R"({"tetrahedron1":{"x1":%.2f,"y1":%.2f,"z1":%.2f,"x2":%.2f,"y2":%.2f,"z2":%.2f,"x3":%.2f,"y3":%.2f,"z3":%.2f,"x4":%.2f,"y4":%.2f,"z4":%.2f},"tetrahedron2":{"x1":%.2f,"y1":%.2f,"z1":%.2f,"x2":%.2f,"y2":%.2f,"z2":%.2f,"x3":%.2f,"y3":%.2f,"z3":%.2f,"x4":%.2f,"y4":%.2f,"z4":%.2f}})",
            w1[0].x, w1[0].y, w1[0].z, w1[1].x, w1[1].y, w1[1].z, w1[2].x,
            w1[2].y, w1[2].z, w1[3].x, w1[3].y, w1[3].z, w2[0].x, w2[0].y,
            w2[0].z, w2[1].x, w2[1].y, w2[1].z, w2[2].x, w2[2].y, w2[2].z,
            w2[3].x, w2[3].y, w2[3].z);
        SetClipboardText(buf);
        m_success_timer = 1.0f;
    }
    if (m_success_timer > 0)
        DrawText("Copy Success", (int)x + 4, (int)y + (int)kRowH, 18, GREEN);
}
