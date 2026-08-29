#pragma once
#include "examples/nearest_points/util.hpp"
#include "toy_physics/algorithm.hpp"
#include "toy_physics/gjk.hpp"
#include <array>
#include <cstdio>

namespace {

struct SweepControl {
    float m_dir_theta = 0.f;
    float m_dir_phi = 90.f;  // +X
    float m_len = 10.f;

    ::Vector3 GetDirection() const {
        float elev = m_dir_theta * DEG2RAD;
        float azim = m_dir_phi * DEG2RAD;
        return {
            cosf(elev) * sinf(azim),
            sinf(elev),
            cosf(elev) * cosf(azim),
        };
    }

    void RenderGUI(float x, float& y, float panelW, float labelW = 110.f,
                   float ctrlH = 22.f) {
        static constexpr float kPad = 4.f;
        static constexpr float kSecPad = 12.f;
        float kRowH = ctrlH + kPad;

        auto lbl = [](Rectangle r, const char* text) {
            int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
            GuiLabel(r, text);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
        };

        GuiGroupBox({x, y, panelW, kRowH * 4 + kSecPad + kPad}, "Sweep");
        y += kRowH;

        lbl({x, y, labelW, ctrlH}, "Dir Elev");
        GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                     &m_dir_theta, 0, 360);
        y += kRowH;
        lbl({x, y, labelW, ctrlH}, "Dir Azim");
        GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                     &m_dir_phi, 0, 360);
        y += kRowH;
        lbl({x, y, labelW, ctrlH}, "Range");
        GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                     &m_len, 0.5f, 30);
        y += kRowH + kSecPad;
    }
};

// hexagonal prism used as the generic convex in the sweep examples
inline void ComputeHexPrismWorld(::Vector3 center, ::Vector3 rotation,
                                 float radius, float half_height,
                                 std::array<::Vector3, 12>& world) {
    Matrix rot = MatrixRotateXYZ(
        {rotation.x * DEG2RAD, rotation.y * DEG2RAD, rotation.z * DEG2RAD});
    for (int k = 0; k < 6; ++k) {
        float ang = k * 60.f * DEG2RAD;
        ::Vector3 top = {radius * cosf(ang), half_height, radius * sinf(ang)};
        ::Vector3 bot = {radius * cosf(ang), -half_height, radius * sinf(ang)};
        world[k] = Vector3Add(center, Vector3Transform(top, rot));
        world[k + 6] = Vector3Add(center, Vector3Transform(bot, rot));
    }
}

inline void DrawHexPrism(const std::array<::Vector3, 12>& world, Color edge,
                         Color face) {
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
        DrawLine3D(world[k], world[(k + 1) % 6], edge);
        DrawLine3D(world[k + 6], world[(k + 1) % 6 + 6], edge);
        DrawLine3D(world[k], world[k + 6], edge);
    }
}

inline void DrawSweepDir(const ::Vector3& from, const ::Vector3& dir,
                         float len) {
    DrawLine3D(from, Vector3Add(from, Vector3Scale(dir, len)), GRAY);
}

inline void DrawContact(const ::Vector3& hit, const toy_physics::Vector3& nrm) {
    DrawSphere(hit, 0.2f, RED);
    ::Vector3 np = Vector3Add(hit, Vector3Scale(FromVec3(nrm), 3.f));
    DrawCylinderEx(hit, np, 0.08f, 0.08f, 8, GREEN);
    DrawSphere(np, 0.15f, GREEN);
}

inline void DrawWitness(const ::Vector3& witness1, const ::Vector3& witness2) {
    DrawSphere(witness1, 0.18f, PURPLE);
    DrawSphere(witness2, 0.18f, ORANGE);
    DrawLine3D(witness1, witness2, YELLOW);
}

inline void DrawSweepInfo(int status, toy_physics::real t,
                          const toy_physics::Vector3& nrm, float x, float& y) {
    if (status == 0) {
        DrawText("MISS", (int)x + 4, (int)y, 26, RED);
        y += 30;
        return;
    }
    if (status == -1) {
        DrawText("INITIAL OVERLAP", (int)x + 4, (int)y, 22, ORANGE);
        y += 26;
        return;
    }
    char buf[96];
    snprintf(buf, sizeof(buf), "HIT: t=%.2f n=(%.2f, %.2f, %.2f)", t, nrm.x(),
             nrm.y(), nrm.z());
    DrawText(buf, (int)x + 4, (int)y, 16, GREEN);
    y += 18;
}

inline void DrawSweepLegend(float x, float& y) {
    y += 8;
    DrawText("-- Sweep Dir", (int)x + 4, (int)y, 16, GRAY);
    y += 16;
    DrawText("o Hit Pt", (int)x + 4, (int)y, 16, RED);
    y += 16;
    DrawText("o Witness A (overlap)", (int)x + 4, (int)y, 16, PURPLE);
    y += 16;
    DrawText("o Witness B (overlap)", (int)x + 4, (int)y, 16, ORANGE);
    y += 16;
    DrawText("-- Witness Line", (int)x + 4, (int)y, 16, YELLOW);
    y += 16;
    DrawText("-- Normal", (int)x + 4, (int)y, 16, GREEN);
    y += 16;
    DrawText("A sweeps (blue), B static", (int)x + 4, (int)y, 16, DARKGRAY);
}

}  // namespace
