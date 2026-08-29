#pragma once
#include <array>
#include <cstdio>
#include "examples/nearest_points/util.hpp"
#include "toy_physics/algorithm.hpp"

namespace {

struct RayControl {
    // origin: spherical coordinates relative to the object center
    float m_dist = 10.f;
    float m_theta = 15.f;
    float m_phi = 45.f;
    // direction: spherical coordinates
    float m_dir_theta = 345.f;
    float m_dir_phi = 225.f;
    float m_len = 20.f;

    ::Vector3 GetOrigin(::Vector3 center) const {
        float elev = m_theta * DEG2RAD;
        float azim = m_phi * DEG2RAD;
        return {
            center.x + m_dist * cosf(elev) * sinf(azim),
            center.y + m_dist * sinf(elev),
            center.z + m_dist * cosf(elev) * cosf(azim),
        };
    }

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
                   float ctrlH = 28.f) {
        static constexpr float kPad = 6.f;
        static constexpr float kSecPad = 16.f;
        float kRowH = ctrlH + kPad;

        auto lbl = [](Rectangle r, const char* text) {
            int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
            GuiLabel(r, text);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
        };

        GuiGroupBox({x, y, panelW, kRowH * 7 + kSecPad + kPad},
                    "Ray (rel. Center)");
        y += kRowH;

        lbl({x, y, labelW, ctrlH}, "Origin Dist");
        GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                     &m_dist, 0.f, 20);
        y += kRowH;
        lbl({x, y, labelW, ctrlH}, "Origin Elev");
        GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                     &m_theta, 0, 360);
        y += kRowH;
        lbl({x, y, labelW, ctrlH}, "Origin Azim");
        GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                     &m_phi, 0, 360);
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
                     &m_len, 0.5f, 40);
        y += kRowH + kSecPad;
    }
};

inline void DrawRay(const ::Vector3& origin, const ::Vector3& dir, float len) {
    DrawLine3D(origin, Vector3Add(origin, Vector3Scale(dir, len)), GRAY);
    DrawSphere(origin, 0.15f, DARKGRAY);
}

inline void DrawRayHits(const ::Vector3& origin,
                        const std::array<toy_physics::RaycastHit, 2>& hits,
                        uint16_t hit_count) {
    for (uint16_t i = 0; i < hit_count; ++i) {
        if (hits[i].IsInitialOverlap()) {
            DrawSphere(origin, 0.4f, ORANGE);
            continue;
        }
        ::Vector3 hit = FromVec3(hits[i].m_hit);
        DrawSphere(hit, 0.25f, RED);
        DrawLine3D(hit, Vector3Add(hit, Vector3Scale(FromVec3(hits[i].m_normal),
                                                     1.2f)),
                   GREEN);
    }
}

inline void DrawRayInfo(const std::array<toy_physics::RaycastHit, 2>& hits,
                        uint16_t hit_count, float x, float& y) {
    if (hit_count == 0) {
        DrawText("MISS", (int)x + 4, (int)y, 26, RED);
        y += 30;
        return;
    }
    for (uint16_t i = 0; i < hit_count; ++i) {
        if (hits[i].IsInitialOverlap()) {
            DrawText("INITIAL OVERLAP", (int)x + 4, (int)y, 22, ORANGE);
            y += 26;
            continue;
        }
        char buf[96];
        snprintf(buf, sizeof(buf), "HIT %d: dist=%.2f n=(%.2f, %.2f, %.2f)", i,
                 hits[i].m_dist, hits[i].m_normal.x(), hits[i].m_normal.y(),
                 hits[i].m_normal.z());
        DrawText(buf, (int)x + 4, (int)y, 16, GREEN);
        y += 18;
    }
}

inline void DrawRayLegend(float x, float& y) {
    y += 10;
    DrawText("-- Ray", (int)x + 4, (int)y, 16, GRAY);
    y += 16;
    DrawText("o Hit Pt", (int)x + 4, (int)y, 16, RED);
    y += 16;
    DrawText("-- Normal", (int)x + 4, (int)y, 16, GREEN);
    y += 16;
    DrawText("o Overlap", (int)x + 4, (int)y, 16, ORANGE);
}

}  // namespace
