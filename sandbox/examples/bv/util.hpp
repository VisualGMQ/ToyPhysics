#pragma once
#include "toy_physics/common/math.hpp"
#include "examples/nearest_points/util.hpp"
#include "raygui.h"
#include "raylib.h"

namespace {

inline toy_physics::Quaternion EulerToQuat(::Vector3 rot) {
    using toy_physics::Quaternion;
    using toy_physics::Vector3;
    using toy_physics::real;
    return Quaternion{
               Eigen::AngleAxis<real>(rot.x * DEG2RAD, Vector3::UnitX())} *
           Quaternion{
               Eigen::AngleAxis<real>(rot.y * DEG2RAD, Vector3::UnitY())} *
           Quaternion{
               Eigen::AngleAxis<real>(rot.z * DEG2RAD, Vector3::UnitZ())};
}

inline void DrawAABBWire(const toy_physics::AABB& bv) {
    ::Vector3 c = FromVec3((bv.m_min + bv.m_max) * 0.5);
    ::Vector3 s = FromVec3(bv.m_max - bv.m_min);
    DrawCubeWiresV(c, s, RED);
}

inline void RenderRotationGUI(float x, float& y, float panelW, float labelW,
                              float ctrlH, ::Vector3& rot) {
    auto lbl = [](Rectangle r, const char* text) {
        int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
        GuiLabel(r, text);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
    };
    lbl({x, y, labelW, ctrlH}, "Rot X");
    GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                 &rot.x, -180, 180);
    y += ctrlH + 6.f;
    lbl({x, y, labelW, ctrlH}, "Rot Y");
    GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                 &rot.y, -180, 180);
    y += ctrlH + 6.f;
    lbl({x, y, labelW, ctrlH}, "Rot Z");
    GuiSliderBar({x + labelW, y, panelW - labelW, ctrlH}, nullptr, nullptr,
                 &rot.z, -180, 180);
    y += ctrlH + 6.f;
}

}  // namespace
