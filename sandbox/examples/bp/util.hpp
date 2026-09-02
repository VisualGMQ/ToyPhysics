#pragma once
#include "examples/nearest_points/util.hpp"
#include "raygui.h"
#include "raylib.h"
#include "toy_physics/geometry/bounding_box.hpp"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <utility>
#include <vector>

namespace bp_util {

inline void DrawAABBWire(const toy_physics::AABB& aabb, ::Color color) {
    ::Vector3 center = FromVec3((aabb.m_min + aabb.m_max) * 0.5);
    ::Vector3 size = FromVec3(aabb.m_max - aabb.m_min);
    DrawCubeWiresV(center, size, color);
}

/**
 * Draggable & resizable window panel, mirrors the behavior of the
 * example menu (drag via title bar, resize via bottom-right corner).
 */
struct ResizablePanel {
    static constexpr float kHeaderH = 24.f;
    static constexpr float kResizeSz = 20.f;
    static constexpr float kMinW = 260.f;
    static constexpr float kMinH = 220.f;

    ::Rectangle m_bounds{};
    bool m_drag = false;
    bool m_sizing = false;
    bool m_placed = false;
    ::Vector2 m_drag_start{};
    ::Rectangle m_drag_start_bounds{};

    /**
     * Draw the window frame and update drag/resize state.
     * Call every frame before drawing the panel content.
     *
     * @return true when the mouse is over the panel bounds
     */
    bool Begin(const char* title) {
        if (!m_placed) {
            m_bounds = {(float)GetScreenWidth() - 330.f, 100.f, 320.f, 460.f};
            ClampToScreen();
            m_placed = true;
        }

        ::Vector2 mouse = GetMousePosition();
        ::Rectangle header = {m_bounds.x, m_bounds.y, m_bounds.width, kHeaderH};
        ::Rectangle resizeHit = {m_bounds.x + m_bounds.width - kResizeSz,
                                 m_bounds.y + m_bounds.height - kResizeSz,
                                 kResizeSz, kResizeSz};

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (CheckCollisionPointRec(mouse, resizeHit)) {
                m_sizing = true;
                m_drag_start = mouse;
                m_drag_start_bounds = m_bounds;
                GuiLock();  // keep the press away from controls below
            } else if (CheckCollisionPointRec(mouse, header)) {
                m_drag = true;
                m_drag_start = mouse;
                m_drag_start_bounds = m_bounds;
                GuiLock();  // keep the press away from controls below
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            m_drag = false;
            m_sizing = false;
            GuiUnlock();
        }
        if (m_drag) {
            m_bounds.x = m_drag_start_bounds.x + (mouse.x - m_drag_start.x);
            m_bounds.y = m_drag_start_bounds.y + (mouse.y - m_drag_start.y);
        }
        if (m_sizing) {
            m_bounds.width = std::max(
                kMinW, m_drag_start_bounds.width + (mouse.x - m_drag_start.x));
            m_bounds.height = std::max(
                kMinH, m_drag_start_bounds.height + (mouse.y - m_drag_start.y));
        }
        // the panel may have been dragged or resized beyond the screen:
        // keep it (and its bottom-right resize grip) visible
        ClampToScreen();

        if (GuiWindowBox(m_bounds, title)) {
            m_placed = false;
        }

        return CheckCollisionPointRec(mouse, m_bounds);
    }

    /**
     * Draw the resize grip at the bottom-right corner. Call this last so the
     * panel content never covers it.
     */
    void End() const {
        constexpr float kSize = 18.f;
        float right = m_bounds.x + m_bounds.width - 2.f;
        float bottom = m_bounds.y + m_bounds.height - 2.f;
        ::Rectangle grip = {right - kSize, bottom - kSize, kSize, kSize};

        bool hover = CheckCollisionPointRec(GetMousePosition(), grip);

        DrawRectangleRec(grip, hover ? DARKGRAY : LIGHTGRAY);
        DrawRectangleLinesEx(grip, 1.f, GRAY);
        for (int i = 0; i < 3; ++i) {
            float inset = 4.f + static_cast<float>(i) * 5.f;
            DrawLineEx({right - inset, bottom - 1.f},
                       {right - 1.f, bottom - inset}, 1.5f,
                       hover ? LIGHTGRAY : DARKGRAY);
        }
    }

    void ClampToScreen() {
        m_bounds.width =
            std::max(kMinW, std::min(m_bounds.width, (float)GetScreenWidth()));
        m_bounds.height = std::max(
            kMinH, std::min(m_bounds.height, (float)GetScreenHeight()));
        m_bounds.x = std::max(
            0.f,
            std::min(m_bounds.x, (float)GetScreenWidth() - m_bounds.width));
        m_bounds.y = std::max(
            0.f,
            std::min(m_bounds.y, (float)GetScreenHeight() - m_bounds.height));
    }

    [[nodiscard]] ::Vector2 GetContentTopLeft() const {
        return {m_bounds.x + 4.f, m_bounds.y + kHeaderH + 4.f};
    }
};

struct BvhTreeView {
    int m_selected = -1;
    std::vector<uint8_t> m_expanded;
    ::Vector2 m_scroll{};

    template <typename NodeVec>
    void Reset(const NodeVec& nodes) {
        m_selected = -1;
        m_expanded.assign(nodes.size(), 0);
        if (!m_expanded.empty()) {
            m_expanded[0] = 1;
        }
    }

    template <typename NodeVec>
    void Render(float x, float y, float panelW, float areaH,
                const NodeVec& nodes) {
        constexpr float kCtrlH = 26.f;
        constexpr float kPad = 4.f;
        constexpr float kRowH = kCtrlH + kPad;

        if (m_expanded.size() != nodes.size()) {
            Reset(nodes);
        }

        float inner_h = areaH - kRowH - kPad;
        if (inner_h <= 0) {
            return;
        }

        GuiGroupBox({x, y, panelW, areaH}, "BVH Tree");
        y += kRowH;

        std::vector<std::pair<int, int>> rows;
        std::function<void(int, int)> visit = [&](int node, int depth) {
            if (node < 0 || node >= static_cast<int>(nodes.size()) ||
                rows.size() >= 1024) {
                return;
            }
            rows.emplace_back(node, depth);
            const auto& n = nodes[node];
            if (!n.m_data.IsLeaf() &&
                node < static_cast<int>(m_expanded.size()) &&
                m_expanded[node]) {
                visit(static_cast<int>(n.m_data.GetPositiveChild()), depth + 1);
                visit(static_cast<int>(n.m_data.GetNegativeChild()), depth + 1);
            }
        };
        if (!nodes.empty()) {
            visit(0, 0);
        }

        float content_h = static_cast<float>(rows.size()) * kRowH;
        ::Rectangle content = {0, 0, panelW - kPad, content_h};
        ::Rectangle view = {};
        GuiScrollPanel({x, y, panelW, inner_h}, nullptr, content, &m_scroll,
                       &view);

        BeginScissorMode((int)view.x, (int)view.y, (int)view.width,
                         (int)view.height);
        float row_y = y + m_scroll.y;
        for (const auto& [node, depth] : rows) {
            const auto& n = nodes[node];
            float indent = static_cast<float>(depth) * 18.f;
            ::Rectangle arrow = {x + kPad + indent, row_y, 20.f, kCtrlH};
            ::Rectangle label = {arrow.x + 22.f, row_y,
                                 panelW - 30.f - indent - kPad, kCtrlH};

            if (!n.m_data.IsLeaf()) {
                if (GuiButton(arrow, m_expanded[node] ? "-" : "+")) {
                    m_expanded[node] = !m_expanded[node];
                }
            } else {
                GuiLabel(arrow, "*");
            }

            char buf[64];
            if (n.m_data.IsLeaf()) {
                snprintf(buf, sizeof(buf), "Node %d (leaf, %d objs)", node,
                         n.m_data.GetPrimNum());
            } else {
                snprintf(buf, sizeof(buf), "Node %d", node);
            }

            bool active = (m_selected == node);
            if (active) {
                GuiSetState(STATE_PRESSED);
            }
            int prev_align = GuiGetStyle(BUTTON, TEXT_ALIGNMENT);
            GuiSetStyle(BUTTON, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);
            if (GuiButton(label, buf)) {
                m_selected = node;
            }
            GuiSetStyle(BUTTON, TEXT_ALIGNMENT, prev_align);
            if (active) {
                GuiSetState(STATE_NORMAL);
            }
            row_y += kRowH;
        }
        EndScissorMode();
    }
};

}  // namespace bp_util
