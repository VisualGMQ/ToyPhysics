#pragma once
#include "example.hpp"
#include "examples/nearest_points/util.hpp"
#include "toy_physics/geometry/box.hpp"
#include "toy_physics/geometry/capsule.hpp"
#include "toy_physics/geometry/convex_hull.hpp"
#include "toy_physics/geometry/cylinder.hpp"
#include "toy_physics/geometry/sphere.hpp"
#include "toy_physics/lowlevel/manifold.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <variant>
#include <vector>

#include "raymath.h"
#include "rlgl.h"

namespace manifold_example {

enum class ShapeKind {
    Box,
    Sphere,
    Capsule,
    Cylinder,
    Convex,
};

inline const char* ShapeName(ShapeKind kind) {
    switch (kind) {
        case ShapeKind::Box:
            return "OBB";
        case ShapeKind::Sphere:
            return "Sphere";
        case ShapeKind::Capsule:
            return "Capsule";
        case ShapeKind::Cylinder:
            return "Cylinder";
        case ShapeKind::Convex:
            return "Convex";
    }
    return "Shape";
}

struct ShapeState {
    ShapeKind m_kind{ShapeKind::Box};
    ::Vector3 m_center{-1.5f, 10.f, 0.f};
    ::Vector3 m_rotation{0.f, 0.f, 0.f};

    ::Vector3 m_half_extent{2.f, 1.5f, 1.5f};
    float m_radius = 2.f;
    float m_half_height = 1.5f;
};

inline toy_physics::Quaternion ToPhysicsQuaternion(::Vector3 euler_degrees) {
    Matrix matrix =
        MatrixRotateXYZ({euler_degrees.x * DEG2RAD, euler_degrees.y * DEG2RAD,
                         euler_degrees.z * DEG2RAD});
    ::Quaternion q = QuaternionFromMatrix(matrix);
    return toy_physics::Quaternion(q.w, q.x, q.y, q.z);
}

inline toy_physics::Transform ToPhysicsTransform(const ShapeState& state) {
    toy_physics::Transform transform;
    transform.m_position = ToVec3(state.m_center);
    transform.m_rotation = ToPhysicsQuaternion(state.m_rotation);
    return transform;
}

inline void FillHexPrism(toy_physics::ConvexData& data, float radius,
                         float half_height) {
    data.m_vertices.clear();
    for (int k = 0; k < 6; ++k) {
        float angle = k * 60.f * DEG2RAD;
        data.m_vertices.push_back(toy_physics::Vector3{
            radius * cosf(angle), half_height, radius * sinf(angle)});
        data.m_vertices.push_back(toy_physics::Vector3{
            radius * cosf(angle), -half_height, radius * sinf(angle)});
    }
}

using GeometryVariant =
    std::variant<toy_physics::BoxGeometry, toy_physics::SphereGeometry,
                 toy_physics::CapsuleGeometry, toy_physics::CylinderGeometry,
                 toy_physics::ConvexHullGeometry>;

inline GeometryVariant MakeGeometry(const ShapeState& state,
                                    toy_physics::ConvexData& convex_data) {
    switch (state.m_kind) {
        case ShapeKind::Box:
            return GeometryVariant(std::in_place_type<toy_physics::BoxGeometry>,
                                   ToVec3(state.m_half_extent));
        case ShapeKind::Sphere:
            return GeometryVariant(
                std::in_place_type<toy_physics::SphereGeometry>,
                state.m_radius);
        case ShapeKind::Capsule:
            return GeometryVariant(
                std::in_place_type<toy_physics::CapsuleGeometry>,
                state.m_radius, state.m_half_height);
        case ShapeKind::Cylinder:
            return GeometryVariant(
                std::in_place_type<toy_physics::CylinderGeometry>,
                state.m_radius, state.m_half_height);
        case ShapeKind::Convex:
            FillHexPrism(convex_data, state.m_radius, state.m_half_height);
            return GeometryVariant(
                std::in_place_type<toy_physics::ConvexHullGeometry>,
                convex_data);
    }
    return GeometryVariant(std::in_place_type<toy_physics::BoxGeometry>,
                           ToVec3(state.m_half_extent));
}

inline const toy_physics::Geometry& GetGeometry(
    const GeometryVariant& variant) {
    return std::visit(
        [](const auto& geometry) -> const toy_physics::Geometry& {
            return geometry;
        },
        variant);
}

inline void ComputeHexPrismWorld(::Vector3 center, ::Vector3 rotation,
                                 float radius, float half_height,
                                 std::array<::Vector3, 12>& world) {
    Matrix matrix = MatrixRotateXYZ(
        {rotation.x * DEG2RAD, rotation.y * DEG2RAD, rotation.z * DEG2RAD});
    for (int k = 0; k < 6; ++k) {
        float angle = k * 60.f * DEG2RAD;
        ::Vector3 top = {radius * cosf(angle), half_height,
                         radius * sinf(angle)};
        ::Vector3 bottom = {radius * cosf(angle), -half_height,
                            radius * sinf(angle)};
        world[k] = Vector3Add(center, Vector3Transform(top, matrix));
        world[k + 6] = Vector3Add(center, Vector3Transform(bottom, matrix));
    }
}

inline void DrawHexPrism(const std::array<::Vector3, 12>& world, Color edge,
                         Color face) {
    for (int k = 0; k < 6; ++k) {
        int next = (k + 1) % 6;
        DrawTriangle3D(world[k], world[next], world[next + 6], face);
        DrawTriangle3D(world[k], world[next + 6], world[k + 6], face);
    }
    for (int k = 1; k < 5; ++k) {
        DrawTriangle3D(world[0], world[k + 1], world[k], face);
        DrawTriangle3D(world[6], world[6 + k], world[6 + k + 1], face);
    }
    for (int k = 0; k < 6; ++k) {
        int next = (k + 1) % 6;
        DrawLine3D(world[k], world[next], edge);
        DrawLine3D(world[k + 6], world[next + 6], edge);
        DrawLine3D(world[k], world[k + 6], edge);
    }
}

inline void DrawShapeFill(Context& ctx, const ShapeState& state, Color color) {
    switch (state.m_kind) {
        case ShapeKind::Box:
            ctx.DrawBoxFlat(state.m_center, state.m_rotation,
                            state.m_half_extent, color);
            break;
        case ShapeKind::Sphere:
            ctx.DrawSphereFlat(state.m_center, state.m_rotation, state.m_radius,
                               color);
            break;
        case ShapeKind::Capsule:
            ctx.DrawCapsuleFlat(state.m_center, state.m_rotation,
                                state.m_half_height * 2.f, state.m_radius,
                                color);
            break;
        case ShapeKind::Cylinder:
            ctx.DrawCylinderFlat(state.m_center, state.m_rotation,
                                 state.m_half_height * 2.f, state.m_radius,
                                 color);
            break;
        case ShapeKind::Convex: {
            std::array<::Vector3, 12> world;
            ComputeHexPrismWorld(state.m_center, state.m_rotation,
                                 state.m_radius, state.m_half_height, world);
            DrawHexPrism(world, Color{0, 0, 0, 0}, color);
            break;
        }
    }
}

inline void DrawShapeWires(const ShapeState& state, Color color) {
    switch (state.m_kind) {
        case ShapeKind::Box: {
            rlPushMatrix();
            rlTranslatef(state.m_center.x, state.m_center.y, state.m_center.z);
            rlRotatef(state.m_rotation.x, 1.f, 0.f, 0.f);
            rlRotatef(state.m_rotation.y, 0.f, 1.f, 0.f);
            rlRotatef(state.m_rotation.z, 0.f, 0.f, 1.f);
            DrawCubeWiresV(
                {0.f, 0.f, 0.f},
                {state.m_half_extent.x * 2.f, state.m_half_extent.y * 2.f,
                 state.m_half_extent.z * 2.f},
                color);
            rlPopMatrix();
            break;
        }
        case ShapeKind::Sphere:
            DrawSphereWires(state.m_center, state.m_radius, 12, 24, color);
            break;
        case ShapeKind::Capsule: {
            const float half_height = state.m_half_height;
            rlPushMatrix();
            rlTranslatef(state.m_center.x, state.m_center.y, state.m_center.z);
            rlRotatef(state.m_rotation.x, 1.f, 0.f, 0.f);
            rlRotatef(state.m_rotation.y, 0.f, 1.f, 0.f);
            rlRotatef(state.m_rotation.z, 0.f, 0.f, 1.f);
            DrawCylinderWires({0.f, -half_height, 0.f}, state.m_radius,
                              state.m_radius, half_height * 2.f, 16, color);
            DrawSphereWires({0.f, -half_height, 0.f}, state.m_radius, 8, 16,
                            color);
            DrawSphereWires({0.f, half_height, 0.f}, state.m_radius, 8, 16,
                            color);
            rlPopMatrix();
            break;
        }
        case ShapeKind::Cylinder: {
            const float half_height = state.m_half_height;
            rlPushMatrix();
            rlTranslatef(state.m_center.x, state.m_center.y, state.m_center.z);
            rlRotatef(state.m_rotation.x, 1.f, 0.f, 0.f);
            rlRotatef(state.m_rotation.y, 0.f, 1.f, 0.f);
            rlRotatef(state.m_rotation.z, 0.f, 0.f, 1.f);
            DrawCylinderWires({0.f, -half_height, 0.f}, state.m_radius,
                              state.m_radius, half_height * 2.f, 16, color);
            rlPopMatrix();
            break;
        }
        case ShapeKind::Convex: {
            std::array<::Vector3, 12> world;
            ComputeHexPrismWorld(state.m_center, state.m_rotation,
                                 state.m_radius, state.m_half_height, world);
            DrawHexPrism(world, color, Color{0, 0, 0, 0});
            break;
        }
    }
}

}  // namespace manifold_example

class ManifoldPairExampleBase : public IExample {
public:
    ManifoldPairExampleBase(Context& ctx, const std::string& name,
                            manifold_example::ShapeKind kind1,
                            manifold_example::ShapeKind kind2)
        : IExample{ctx, name}, m_kind1{kind1}, m_kind2{kind2} {
        InitState(m_state1, kind1, -1.2f);
        InitState(m_state2, kind2, 1.2f);
    }

    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    static void InitState(manifold_example::ShapeState& state,
                          manifold_example::ShapeKind kind, float x) {
        state.m_kind = kind;
        state.m_center = {x, 10.f, 0.f};
        state.m_rotation = {0.f, 0.f, 0.f};
        switch (kind) {
            case manifold_example::ShapeKind::Box:
                state.m_half_extent = {2.f, 1.5f, 1.5f};
                break;
            case manifold_example::ShapeKind::Sphere:
                state.m_radius = 2.f;
                break;
            case manifold_example::ShapeKind::Capsule:
                state.m_radius = 1.5f;
                state.m_half_height = 1.5f;
                break;
            case manifold_example::ShapeKind::Cylinder:
                state.m_radius = 1.5f;
                state.m_half_height = 2.f;
                break;
            case manifold_example::ShapeKind::Convex:
                state.m_radius = 2.f;
                state.m_half_height = 1.5f;
                break;
        }
    }

    void RenderShapeGUI(float x, float& y, const char* title,
                        manifold_example::ShapeState& state);

    manifold_example::ShapeKind m_kind1;
    manifold_example::ShapeKind m_kind2;
    manifold_example::ShapeState m_state1;
    manifold_example::ShapeState m_state2;
    toy_physics::Manifold m_manifold;
};

inline void ManifoldPairExampleBase::OnRender3D(float delta_time) {
    (void)delta_time;

    toy_physics::ConvexData convex_data1;
    toy_physics::ConvexData convex_data2;
    manifold_example::GeometryVariant geometry1 =
        manifold_example::MakeGeometry(m_state1, convex_data1);
    manifold_example::GeometryVariant geometry2 =
        manifold_example::MakeGeometry(m_state2, convex_data2);

    const toy_physics::Transform transform1 =
        manifold_example::ToPhysicsTransform(m_state1);
    const toy_physics::Transform transform2 =
        manifold_example::ToPhysicsTransform(m_state2);

    toy_physics::ComputeManifold(
        manifold_example::GetGeometry(geometry1), transform1,
        manifold_example::GetGeometry(geometry2), transform2, &m_manifold);

    const Color fill1 = {0, 120, 255, 55};
    const Color wire1 = {60, 160, 255, 230};
    const Color fill2 = {255, 140, 0, 55};
    const Color wire2 = {255, 170, 60, 230};

    // Do not write depth for the translucent shapes so the contact points and
    // normals inside them remain visible.
    rlDisableDepthMask();
    manifold_example::DrawShapeFill(m_ctx, m_state1, fill1);
    manifold_example::DrawShapeFill(m_ctx, m_state2, fill2);
    manifold_example::DrawShapeWires(m_state1, wire1);
    manifold_example::DrawShapeWires(m_state2, wire2);
    rlEnableDepthMask();

    if (!m_manifold.IsValid()) {
        return;
    }

    ::Vector3 center = {0.f, 0.f, 0.f};
    for (int i = 0; i < m_manifold.m_contact_num; ++i) {
        ::Vector3 p1 = FromVec3(m_manifold.m_contacts1[i]);
        ::Vector3 p2 = FromVec3(m_manifold.m_contacts2[i]);

        DrawLine3D(p1, p2, YELLOW);

        // Wireframe shells keep both markers visible even when the two
        // contact points coincide (zero depth / touching contact).
        ::DrawSphereWires(p1, 0.22f, 8, 12, RED);
        ::DrawSphereWires(p2, 0.30f, 8, 12, GREEN);
        ::DrawSphere(p1, 0.07f, RED);
        ::DrawSphere(p2, 0.07f, GREEN);

        center = Vector3Add(center, p1);
    }
    center = Vector3Scale(center,
                          1.f / static_cast<float>(m_manifold.m_contact_num));

    ::Vector3 normal = FromVec3(m_manifold.m_separate_normal);
    ::Vector3 normal_end = Vector3Add(center, Vector3Scale(normal, 3.f));
    DrawCylinderEx(center, normal_end, 0.06f, 0.06f, 8, ORANGE);
    ::DrawSphere(normal_end, 0.15f, ORANGE);
}

inline void ManifoldPairExampleBase::RenderShapeGUI(
    float x, float& y, const char* title, manifold_example::ShapeState& state) {
    static constexpr float kPanelW = 320.f;
    static constexpr float kLabelW = 110.f;
    static constexpr float kCtrlH = 24.f;
    static constexpr float kPad = 4.f;
    static constexpr float kRowH = kCtrlH + kPad;
    static constexpr float kSecPad = 12.f;

    auto lbl = [](Rectangle rect, const char* text) {
        int previous = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
        GuiLabel(rect, text);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, previous);
    };

    const bool is_box = state.m_kind == manifold_example::ShapeKind::Box;
    const int extra_rows =
        is_box ? 3
               : (state.m_kind == manifold_example::ShapeKind::Sphere ? 1 : 2);
    const float group_height =
        kRowH * static_cast<float>(6 + extra_rows) + kSecPad + kPad;

    GuiGroupBox({x, y, kPanelW, group_height}, title);
    y += kRowH;

    lbl({x, y, kLabelW, kCtrlH}, "Center X");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &state.m_center.x, -15.f, 15.f);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Y");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &state.m_center.y, 0.f, 20.f);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Center Z");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &state.m_center.z, -15.f, 15.f);
    y += kRowH;

    lbl({x, y, kLabelW, kCtrlH}, "Pitch");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &state.m_rotation.x, -180.f, 180.f);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Yaw");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &state.m_rotation.y, -180.f, 180.f);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Roll");
    GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr, nullptr,
                 &state.m_rotation.z, -180.f, 180.f);
    y += kRowH;

    if (is_box) {
        lbl({x, y, kLabelW, kCtrlH}, "Size X");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &state.m_half_extent.x, 0.3f, 5.f);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Size Y");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &state.m_half_extent.y, 0.3f, 5.f);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Size Z");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &state.m_half_extent.z, 0.3f, 5.f);
        y += kRowH;
    } else if (state.m_kind == manifold_example::ShapeKind::Sphere) {
        lbl({x, y, kLabelW, kCtrlH}, "Radius");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &state.m_radius, 0.3f, 4.f);
        y += kRowH;
    } else {
        lbl({x, y, kLabelW, kCtrlH}, "Radius");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &state.m_radius, 0.3f, 4.f);
        y += kRowH;
        lbl({x, y, kLabelW, kCtrlH}, "Half Height");
        GuiSliderBar({x + kLabelW, y, kPanelW - kLabelW, kCtrlH}, nullptr,
                     nullptr, &state.m_half_height, 0.3f, 4.f);
        y += kRowH;
    }

    y += kSecPad;
}

inline void ManifoldPairExampleBase::OnRender2D(float delta_time) {
    (void)delta_time;

    static constexpr float kPanelW = 320.f;
    static constexpr float kPanelX = 10.f;
    float x = static_cast<float>(GetScreenWidth()) - kPanelW - kPanelX;
    float y = 90.f;

    RenderShapeGUI(x, y, manifold_example::ShapeName(m_kind1), m_state1);
    RenderShapeGUI(x, y, manifold_example::ShapeName(m_kind2), m_state2);

    if (m_manifold.IsValid()) {
        DrawText(TextFormat("Contacts: %u",
                            static_cast<unsigned>(m_manifold.m_contact_num)),
                 static_cast<int>(x) + 4, static_cast<int>(y), 20, GREEN);
        y += 24.f;
        DrawText(TextFormat("Depth: %.3f",
                            static_cast<float>(m_manifold.m_penetration_depth)),
                 static_cast<int>(x) + 4, static_cast<int>(y), 18, GREEN);
        y += 22.f;
        DrawText(TextFormat("Normal: (%.2f, %.2f, %.2f)",
                            m_manifold.m_separate_normal.x(),
                            m_manifold.m_separate_normal.y(),
                            m_manifold.m_separate_normal.z()),
                 static_cast<int>(x) + 4, static_cast<int>(y), 18, GREEN);
        y += 22.f;
        DrawText("Red: body1 contact", static_cast<int>(x) + 4,
                 static_cast<int>(y), 18, RED);
        y += 20.f;
        DrawText("Green: body2 contact", static_cast<int>(x) + 4,
                 static_cast<int>(y), 18, GREEN);
        y += 20.f;
        DrawText("Yellow: contact pair", static_cast<int>(x) + 4,
                 static_cast<int>(y), 18, GOLD);
    } else {
        DrawText("SEPARATED", static_cast<int>(x) + 4, static_cast<int>(y), 22,
                 GRAY);
    }
}

#define DEFINE_MANIFOLD_EXAMPLE(ClassName, Kind1, Kind2)                     \
    class ClassName : public ManifoldPairExampleBase {                       \
    public:                                                                  \
        ClassName(Context& ctx, const std::string& name)                     \
            : ManifoldPairExampleBase(ctx, name,                             \
                                      manifold_example::ShapeKind::Kind1,    \
                                      manifold_example::ShapeKind::Kind2) {} \
    }

DEFINE_MANIFOLD_EXAMPLE(ManifoldOBBOBBExample, Box, Box);
DEFINE_MANIFOLD_EXAMPLE(ManifoldOBBSphereExample, Box, Sphere);
DEFINE_MANIFOLD_EXAMPLE(ManifoldSphereSphereExample, Sphere, Sphere);
DEFINE_MANIFOLD_EXAMPLE(ManifoldSphereCapsuleExample, Sphere, Capsule);
DEFINE_MANIFOLD_EXAMPLE(ManifoldCapsuleCapsuleExample, Capsule, Capsule);
DEFINE_MANIFOLD_EXAMPLE(ManifoldCylinderCylinderExample, Cylinder, Cylinder);
DEFINE_MANIFOLD_EXAMPLE(ManifoldCylinderCapsuleExample, Cylinder, Capsule);
DEFINE_MANIFOLD_EXAMPLE(ManifoldCapsuleConvexExample, Capsule, Convex);
DEFINE_MANIFOLD_EXAMPLE(ManifoldConvexConvexExample, Convex, Convex);
DEFINE_MANIFOLD_EXAMPLE(ManifoldConvexCylinderExample, Convex, Cylinder);
