#pragma once
#include <cstdio>
#include "raylib.h"
#include "raymath.h"
#include "raygui.h"
#include "toy_physics/math.hpp"

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
        static constexpr float kPad = 4.f;
        static constexpr float kSecPad = 12.f;
        float kRowH = ctrlH + kPad;

        auto lbl = [](Rectangle r, const char* text) {
            int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
            GuiLabel(r, text);
            GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
        };

        GuiGroupBox({x, y, panelW, kRowH * 4 + kSecPad + kPad}, title);
        y += kRowH;

        lbl({x, y, labelW, ctrlH}, "Distance");
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.1f", m_dist);
            GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH},
                         nullptr, buf, &m_dist, 0.1f, 30);
        }
        y += kRowH;
        lbl({x, y, labelW, ctrlH}, "Elevation");
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.0f", m_theta);
            GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH},
                         nullptr, buf, &m_theta, 0, 360);
        }
        y += kRowH;
        lbl({x, y, labelW, ctrlH}, "Azimuth");
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.0f", m_phi);
            GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH},
                         nullptr, buf, &m_phi, 0, 360);
        }
        y += kRowH;
    }
};
