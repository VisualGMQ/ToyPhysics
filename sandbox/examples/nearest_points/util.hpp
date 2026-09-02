#pragma once
#include "toy_physics/common/math.hpp"
#include "raygui.h"
#include "raylib.h"
#include "raymath.h"
#include <cstdio>

namespace {

inline toy_physics::Vector3 ToVec3(const ::Vector3& v) { return {v.x, v.y, v.z}; }
inline ::Vector3 FromVec3(const toy_physics::Vector3& v) { return {v.x(), v.y(), v.z()}; }

}  // namespace

struct SphericalPoint {
    float m_dist = 8.f;
    float m_theta = 330.f;
    float m_phi = 45.f;

    ::Vector3 ToCartesian(::Vector3 center) const {
        float elev = m_theta * DEG2RAD;
        float azim = m_phi * DEG2RAD;
        return {
            center.x + m_dist * cosf(elev) * sinf(azim),
            center.y + m_dist * sinf(elev),
            center.z + m_dist * cosf(elev) * cosf(azim),
        };
    }

    void RenderGUI(float x, float& y, float panelW, float labelW = 80.f,
                   float ctrlH = 20.f,
                   const char* title = "Point P (Spherical)") {
        static constexpr float kPad = 6.f;
        static constexpr float kSecPad = 16.f;
        static constexpr float kValH = 18.f;
        float kRowH = kValH + ctrlH + kPad;

        auto lbl = [](Rectangle r, const char* text) {
            int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
            GuiLabel(r, text);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
        };

        GuiGroupBox({x, y, panelW, kRowH * 4 + kSecPad + kPad}, title);
        y += kRowH;

        lbl({x, y + kValH, labelW, ctrlH}, "Distance");
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.1f", m_dist);
            DrawText(buf, (int)(x + labelW), (int)y, 16, GRAY);
            GuiSliderBar({x + labelW, y + kValH, panelW - labelW, ctrlH},
                         nullptr, nullptr, &m_dist, 0.1f, 30);
        }
        y += kRowH;
        lbl({x, y + kValH, labelW, ctrlH}, "Elevation");
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.0f", m_theta);
            DrawText(buf, (int)(x + labelW), (int)y, 16, GRAY);
            GuiSliderBar({x + labelW, y + kValH, panelW - labelW, ctrlH},
                         nullptr, nullptr, &m_theta, 0, 360);
        }
        y += kRowH;
        lbl({x, y + kValH, labelW, ctrlH}, "Azimuth");
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.0f", m_phi);
            DrawText(buf, (int)(x + labelW), (int)y, 16, GRAY);
            GuiSliderBar({x + labelW, y + kValH, panelW - labelW, ctrlH},
                         nullptr, nullptr, &m_phi, 0, 360);
        }
        y += kRowH;
    }
};
