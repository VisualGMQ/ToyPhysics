#include "examples/bp/inc_bvh.hpp"

#include "context.hpp"
#include "examples/bv/util.hpp"
#include "toy_physics/lowlevel/algorithm.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <random>
#include <span>

namespace {

constexpr float kCtrlH = 26.f;
constexpr float kPad = 4.f;
constexpr float kRowH = kCtrlH + kPad;
constexpr float kSecPad = 12.f;
constexpr float kRayLen = 1000.f;

}  // namespace

void IncBvhExample::OnUpdate(float delta_time) {
    if (!m_initialized) {
        return;
    }
    if (!IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        return;
    }

    // ignore clicks over the panel
    ::Vector2 mouse = GetMousePosition();
    if (CheckCollisionPointRec(mouse, m_panel.m_bounds)) {
        return;
    }
    pickShape(mouse);
}

void IncBvhExample::pickShape(::Vector2 mouse) {
    ::Ray ray = GetMouseRay(mouse, m_ctx.GetCamera());

    float best_dist = 0;
    int best_index = -1;
    for (size_t i = 0; i < m_render_items.size(); ++i) {
        float dist = raycastShape(ray, m_render_items[i]);
        if (dist >= 0 && (best_index < 0 || dist < best_dist)) {
            best_dist = dist;
            best_index = static_cast<int>(i);
        }
    }
    m_selected_shape = best_index;
}

float IncBvhExample::raycastShape(const ::Ray& ray,
                                  const RenderItem& item) const {
    using toy_physics::Vector3;

    switch (item.m_type) {
        case toy_physics::Geometry::Type::Sphere: {
            ::RayCollision hit =
                GetRayCollisionSphere(ray, item.m_center, item.m_radius);
            return hit.hit ? hit.distance : -1.f;
        }
        case toy_physics::Geometry::Type::Box: {
            toy_physics::Quaternion q = EulerToQuat(item.m_rotation);
            Vector3 lo =
                q.conjugate() * (ToVec3(ray.position) - ToVec3(item.m_center));
            Vector3 ld = q.conjugate() * ToVec3(ray.direction);
            ::Ray local_ray{FromVec3(lo), FromVec3(ld)};
            ::BoundingBox box{
                {-item.m_half_extent.x, -item.m_half_extent.y,
                 -item.m_half_extent.z},
                { item.m_half_extent.x,  item.m_half_extent.y,
                 item.m_half_extent.z }
            };
            ::RayCollision hit = GetRayCollisionBox(local_ray, box);
            return hit.hit ? hit.distance : -1.f;
        }
        case toy_physics::Geometry::Type::Capsule:
        case toy_physics::Geometry::Type::Cylinder: {
            toy_physics::Quaternion q = EulerToQuat(item.m_rotation);
            Vector3 axis = q * Vector3::UnitY();
            Vector3 center = ToVec3(item.m_center);
            Vector3 half = axis * (item.m_height * 0.5f);

            Vector3 r1 = ToVec3(ray.position);
            Vector3 r2 = r1 + ToVec3(ray.direction) * kRayLen;
            auto [p_on_ray, p_on_axis] = toy_physics::GetSegSegNearestPoints(
                r1, r2, center + half, center - half);

            toy_physics::real dist = (p_on_ray - p_on_axis).norm();
            if (dist > item.m_radius) {
                return -1.f;
            }
            if (item.m_type == toy_physics::Geometry::Type::Cylinder) {
                toy_physics::real s = (p_on_axis - center).dot(axis);
                if (std::abs(s) > item.m_height * 0.5f) {
                    return -1.f;
                }
            }
            return (p_on_ray - r1).norm();
        }
        default:
            return -1.f;
    }
}

std::unique_ptr<toy_physics::Shape> IncBvhExample::createShapeFromItem(
    const RenderItem& item) const {
    switch (item.m_type) {
        case toy_physics::Geometry::Type::Sphere:
            return std::make_unique<toy_physics::Shape>(
                toy_physics::SphereGeometry{item.m_radius});
        case toy_physics::Geometry::Type::Box:
            return std::make_unique<toy_physics::Shape>(
                toy_physics::BoxGeometry{
                    toy_physics::Vector3{item.m_half_extent.x,
                                         item.m_half_extent.y,
                                         item.m_half_extent.z}
            });
        case toy_physics::Geometry::Type::Capsule:
            return std::make_unique<toy_physics::Shape>(
                toy_physics::CapsuleGeometry{item.m_radius,
                                             item.m_height * 0.5f});
        case toy_physics::Geometry::Type::Cylinder:
            return std::make_unique<toy_physics::Shape>(
                toy_physics::CylinderGeometry{item.m_radius,
                                              item.m_height * 0.5f});
        default:
            return nullptr;
    }
}

void IncBvhExample::applyTransform(size_t index) {
    const RenderItem& item = m_render_items[index];
    m_shapes[index]->SetTransform(
        toy_physics::Vector3{item.m_center.x, item.m_center.y, item.m_center.z},
        EulerToQuat(item.m_rotation));
}

void IncBvhExample::addShape(RenderItem item) {
    auto shape = createShapeFromItem(item);
    if (!shape) {
        return;
    }
    auto* raw = shape.get();
    m_shapes.push_back(std::move(shape));
    m_render_items.push_back(item);
    applyTransform(m_shapes.size() - 1);

    // applied by the next Apply
    m_broad_phase->AddObjects(std::span<toy_physics::Shape>{raw, 1});
    m_pending_add_set.insert(raw);
    m_selected_shape = static_cast<int>(m_shapes.size() - 1);
}

void IncBvhExample::bulkAddShapes(int count) {
    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> pos{-40.f, 40.f};
    std::uniform_real_distribution<float> size{0.3f, 2.f};
    std::uniform_real_distribution<float> rot{0.f, 360.f};
    std::uniform_int_distribution<int> kind{0, 3};

    for (int i = 0; i < count; ++i) {
        RenderItem item;
        item.m_center = {pos(rng), pos(rng), pos(rng)};
        item.m_rotation = {rot(rng), rot(rng), rot(rng)};
        switch (kind(rng)) {
            case 0:
                item.m_type = toy_physics::Geometry::Type::Sphere;
                item.m_radius = size(rng);
                break;
            case 1:
                item.m_type = toy_physics::Geometry::Type::Box;
                item.m_half_extent = {size(rng), size(rng), size(rng)};
                break;
            case 2:
                item.m_type = toy_physics::Geometry::Type::Capsule;
                item.m_radius = size(rng);
                item.m_height = size(rng) * 2.f;
                break;
            default:
                item.m_type = toy_physics::Geometry::Type::Cylinder;
                item.m_radius = size(rng);
                item.m_height = size(rng) * 2.f;
                break;
        }
        addShape(item);
    }
}

void IncBvhExample::queueMove(size_t index) {
    applyTransform(index);

    // re-register the shape so its new BV reaches the tree on the next
    // Apply; one pending add suffices, TryRebuild reads the latest BV
    toy_physics::Shape* shape = m_shapes[index].get();
    if (m_pending_add_set.contains(shape)) {
        return;
    }
    m_broad_phase->RemoveObjects(std::span<toy_physics::Shape>{shape, 1});
    m_broad_phase->AddObjects(std::span<toy_physics::Shape>{shape, 1});
    m_pending_add_set.insert(shape);
    m_pending_remove_set.insert(shape);
}

void IncBvhExample::deleteSelectedShape() {
    if (m_selected_shape < 0 ||
        m_selected_shape >= static_cast<int>(m_shapes.size())) {
        return;
    }

    toy_physics::Shape* shape = m_shapes[m_selected_shape].get();
    if (m_pending_add_set.contains(shape)) {
        // its pending add must be applied before it can be removed
        m_delete_requested.insert(shape);
        m_selected_shape = -1;
        return;
    }

    // applied by the next Apply; the shape stays alive in the graveyard
    // until then
    m_broad_phase->RemoveObjects(std::span<toy_physics::Shape>{shape, 1});
    m_pending_remove_set.insert(shape);
    m_graveyard.push_back(std::move(m_shapes[m_selected_shape]));
    m_shapes.erase(m_shapes.begin() + m_selected_shape);
    m_render_items.erase(m_render_items.begin() + m_selected_shape);
    m_selected_shape = -1;
}

void IncBvhExample::applyChanges() {
    size_t applied_upto = m_graveyard.size();

    auto start = std::chrono::steady_clock::now();
    m_broad_phase->ApplyModify();
    auto end = std::chrono::steady_clock::now();
    m_rebuild_time_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    m_pending_add_set.clear();
    m_pending_remove_set.clear();

    // every removal queued before this rebuild has been applied, so the
    // graveyard entries holding those shapes can be released
    if (applied_upto > 0) {
        m_graveyard.erase(m_graveyard.begin(),
                          m_graveyard.begin() + applied_upto);
    }

    // shapes that were deleted while pending-add are in the tree now:
    // queue their removal for the next Apply
    for (toy_physics::Shape* shape : m_delete_requested) {
        auto it = std::find_if(m_shapes.begin(), m_shapes.end(),
                               [&](const auto& p) { return p.get() == shape; });
        if (it == m_shapes.end()) {
            continue;
        }
        size_t index = static_cast<size_t>(it - m_shapes.begin());
        m_broad_phase->RemoveObjects(std::span<toy_physics::Shape>{shape, 1});
        m_pending_remove_set.insert(shape);
        m_graveyard.push_back(std::move(*it));
        m_shapes.erase(it);
        m_render_items.erase(m_render_items.begin() + index);
    }
    m_delete_requested.clear();
    m_tree.m_selected = toy_physics::InvalidTightPoolID;
}

void IncBvhExample::OnRender3D(float delta_time) {
    if (!m_initialized) {
        m_broad_phase = std::make_unique<toy_physics::AABBBroadPhase<
            toy_physics::BVHBuildPolicy::Incremental>>();
        m_initialized = true;
    }

    // collect the shapes covered by the selected tree node
    std::unordered_set<const toy_physics::Shape*> highlighted;
    if (m_tree.m_selected != toy_physics::InvalidTightPoolID &&
        m_broad_phase->HasNode(m_tree.m_selected)) {
        std::function<void(toy_physics::TightPoolID)> collect =
            [&](toy_physics::TightPoolID id) {
                const auto& n = m_broad_phase->GetNode(id);
                if (n.m_is_leaf) {
                    for (uint32_t i = 0; i < n.m_object_count; ++i) {
                        const toy_physics::Shape* shape =
                            m_broad_phase->GetPrimitiveByID(n.m_objects[i]);
                        if (shape) {
                            highlighted.insert(shape);
                        }
                    }
                } else {
                    collect(n.m_child[0]);
                    collect(n.m_child[1]);
                }
            };
        collect(m_tree.m_selected);
    }

    // highlighted objects get a slightly scaled-up green shell drawn first,
    // showing through around the silhouette as a thick green outline
    for (size_t i = 0; i < m_render_items.size(); ++i) {
        const auto& item = m_render_items[i];
        bool hl =
            i < m_shapes.size() && highlighted.contains(m_shapes[i].get());
        switch (item.m_type) {
            case toy_physics::Geometry::Type::Sphere:
                if (hl) {
                    m_ctx.DrawSphereFlat(item.m_center, item.m_rotation,
                                         item.m_radius * 1.1f, GREEN);
                }
                m_ctx.DrawSphere(item.m_center, item.m_rotation, item.m_radius,
                                 RED);
                break;
            case toy_physics::Geometry::Type::Box:
                if (hl) {
                    m_ctx.DrawBoxFlat(item.m_center, item.m_rotation,
                                      Vector3Scale(item.m_half_extent, 1.1f),
                                      GREEN);
                }
                m_ctx.DrawBox(item.m_center, item.m_rotation,
                              item.m_half_extent, BLUE);
                break;
            case toy_physics::Geometry::Type::Cylinder:
                if (hl) {
                    m_ctx.DrawCylinderFlat(item.m_center, item.m_rotation,
                                           item.m_height * 1.1f,
                                           item.m_radius * 1.1f, GREEN);
                }
                m_ctx.DrawCylinder(item.m_center, item.m_rotation,
                                   item.m_height, item.m_radius, ORANGE);
                break;
            case toy_physics::Geometry::Type::Capsule:
                if (hl) {
                    m_ctx.DrawCapsuleFlat(item.m_center, item.m_rotation,
                                          item.m_height * 1.1f,
                                          item.m_radius * 1.1f, GREEN);
                }
                m_ctx.DrawCapsule(item.m_center, item.m_rotation, item.m_height,
                                  item.m_radius, PURPLE);
                break;
            default:
                break;
        }
    }

    if (m_selected_shape >= 0 &&
        m_selected_shape < static_cast<int>(m_shapes.size())) {
        bp_util::DrawAABBWire(m_shapes[m_selected_shape]->GetBoundingBox(),
                              ORANGE);
    }

    const auto& nodes = m_broad_phase->GetNodes();
    m_node_count = nodes.size();
    m_leaf_count = 0;
    for (const auto& node : nodes) {
        if (node.m_is_leaf) {
            ++m_leaf_count;
        }
        if (m_draw_bvh_aabbs) {
            bp_util::DrawAABBWire(node.m_aabb, node.m_is_leaf ? GREEN : RED);
        }
    }

    if (m_tree.m_selected != toy_physics::InvalidTightPoolID &&
        m_broad_phase->HasNode(m_tree.m_selected)) {
        const auto& node = m_broad_phase->GetNode(m_tree.m_selected);
        m_ctx.DrawAABBFlat(node.m_aabb, ::Color{255, 150, 0, 90});
        m_ctx.DrawAABBFlatWires(node.m_aabb, YELLOW);
    }
}

void IncBvhExample::renderCreateSection(float x, float& y, float panelW) {
    GuiGroupBox({x, y, panelW, kRowH * 7 + kPad * 2}, "Create Shape");
    y += kRowH;

    float btnW = (panelW - kPad * 3) * 0.5f;
    if (GuiButton({x + kPad, y, btnW, kCtrlH}, "Add Box")) {
        addShape(RenderItem{
            toy_physics::Geometry::Type::Box, {},
             {},
             {1, 1, 1},
             1, 2
        });
    }
    if (GuiButton({x + kPad * 2 + btnW, y, btnW, kCtrlH}, "Add Sphere")) {
        addShape(
            RenderItem{toy_physics::Geometry::Type::Sphere, {}, {}, {}, 1, 2});
    }
    y += kRowH;
    if (GuiButton({x + kPad, y, btnW, kCtrlH}, "Add Capsule")) {
        addShape(RenderItem{
            toy_physics::Geometry::Type::Capsule, {}, {}, {}, 0.5f, 2});
    }
    if (GuiButton({x + kPad * 2 + btnW, y, btnW, kCtrlH}, "Add Cylinder")) {
        addShape(RenderItem{
            toy_physics::Geometry::Type::Cylinder, {}, {}, {}, 0.5f, 2});
    }
    y += kRowH + kPad;

    float count = static_cast<float>(m_bulk_count);
    char count_buf[16];
    snprintf(count_buf, sizeof(count_buf), "%d", m_bulk_count);
    GuiLabel({x + kPad, y, 44.f, kCtrlH}, "Bulk");
    GuiSliderBar({x + kPad + 48.f, y, panelW - kPad * 2 - 48.f, kCtrlH},
                 nullptr, count_buf, &count, 1.f, 10000.f);
    m_bulk_count = static_cast<int>(count);
    y += kRowH;
    if (GuiButton({x, y, panelW, kCtrlH}, "Bulk Add")) {
        bulkAddShapes(m_bulk_count);
    }
    y += kRowH;

    GuiCheckBox({x + kPad, y, kCtrlH, kCtrlH}, "Draw BVH AABBs",
                &m_draw_bvh_aabbs);
    y += kRowH + kSecPad;

    if (GuiButton({x, y, panelW, kCtrlH}, "Apply")) {
        applyChanges();
    }
    y += kRowH + kSecPad;
}

void IncBvhExample::renderSelectedSection(float x, float& y, float panelW) {
    bool has_selection =
        m_selected_shape >= 0 &&
        m_selected_shape < static_cast<int>(m_render_items.size());

    if (!has_selection) {
        GuiGroupBox({x, y, panelW, kRowH * 2 + kPad * 2}, "Selected Shape");
        y += kRowH;
        GuiLabel({x + kPad, y, panelW - kPad * 2, kCtrlH},
                 "Left click a shape to select it");
        y += kRowH + kSecPad;
        return;
    }

    constexpr float kLabelW = 70.f;
    float kSliderW = panelW - kLabelW - kPad * 2;

    size_t sel = static_cast<size_t>(m_selected_shape);
    RenderItem& item = m_render_items[sel];

    GuiGroupBox({x, y, panelW, kRowH * 8 + kPad * 2}, "Selected Shape");
    y += kRowH;

    auto lbl = [](::Rectangle r, const char* text) {
        int prev = GuiGetStyle(LABEL, TEXT_ALIGNMENT);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, TEXT_ALIGN_RIGHT);
        GuiLabel(r, text);
        GuiSetStyle(LABEL, TEXT_ALIGNMENT, prev);
    };

    float px = item.m_center.x, py = item.m_center.y, pz = item.m_center.z;
    float rx = item.m_rotation.x, ry = item.m_rotation.y,
          rz = item.m_rotation.z;

    lbl({x, y, kLabelW, kCtrlH}, "Pos X");
    GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr, nullptr,
                 &px, -50, 50);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Pos Y");
    GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr, nullptr,
                 &py, -50, 50);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Pos Z");
    GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr, nullptr,
                 &pz, -50, 50);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Rot X");
    GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr, nullptr,
                 &rx, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Rot Y");
    GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr, nullptr,
                 &ry, -180, 180);
    y += kRowH;
    lbl({x, y, kLabelW, kCtrlH}, "Rot Z");
    GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr, nullptr,
                 &rz, -180, 180);
    y += kRowH;

    if (px != item.m_center.x || py != item.m_center.y ||
        pz != item.m_center.z || rx != item.m_rotation.x ||
        ry != item.m_rotation.y || rz != item.m_rotation.z) {
        item.m_center = {px, py, pz};
        item.m_rotation = {rx, ry, rz};
        queueMove(sel);
    }

    if (GuiButton({x, y, panelW, kCtrlH}, "Delete")) {
        deleteSelectedShape();
    }
    y += kRowH + kSecPad;
}

void IncBvhExample::OnRender2D(float delta_time) {
    m_panel.Begin("Incremental BVH");

    float x = m_panel.GetContentTopLeft().x;
    float y = m_panel.GetContentTopLeft().y;
    float panelW = m_panel.m_bounds.width - 8.f;

    renderCreateSection(x, y, panelW);
    renderSelectedSection(x, y, panelW);

    char buf[128];
    snprintf(buf, sizeof(buf), "Objects: %zu (pending +%zu -%zu)",
             m_shapes.size(), m_pending_add_set.size(),
             m_pending_remove_set.size());
    DrawText(buf, (int)x + 4, (int)y, 18, BLACK);
    y += 22;
    snprintf(buf, sizeof(buf), "Nodes: %zu (leaves: %zu)", m_node_count,
             m_leaf_count);
    DrawText(buf, (int)x + 4, (int)y, 18, BLACK);
    y += 22;
    snprintf(buf, sizeof(buf), "TryRebuild: %.2f ms", m_rebuild_time_ms);
    DrawText(buf, (int)x + 4, (int)y, 18, BLACK);
    y += 24;

    float tree_h = m_panel.m_bounds.y + m_panel.m_bounds.height - y - 6.f;
    m_tree.Render(x, y, panelW, tree_h, *this);
    m_panel.End();
}

void IncBvhExample::TreeView::Render(float x, float y, float panelW,
                                     float areaH, const IncBvhExample& self) {
    constexpr float kCtrlH = 26.f;
    constexpr float kPad = 4.f;
    constexpr float kRowH = kCtrlH + kPad;

    float inner_h = areaH - kRowH - kPad;
    if (inner_h <= 0) {
        return;
    }

    GuiGroupBox({x, y, panelW, areaH}, "BVH Tree");
    y += kRowH;

    const auto& bp = *self.m_broad_phase;

    std::vector<std::pair<toy_physics::TightPoolID, int>> rows;
    std::function<void(toy_physics::TightPoolID, int)> visit =
        [&](toy_physics::TightPoolID id, int depth) {
            if (id == toy_physics::InvalidTightPoolID || rows.size() >= 1024) {
                return;
            }
            const auto& node = bp.GetNode(id);
            rows.emplace_back(id, depth);
            if (!node.m_is_leaf && m_expanded[static_cast<uint32_t>(id)]) {
                visit(node.m_child[0], depth + 1);
                visit(node.m_child[1], depth + 1);
            }
        };
    if (bp.GetRoot() != toy_physics::InvalidTightPoolID) {
        visit(bp.GetRoot(), 0);
    }

    // drop the selection when its node is no longer in the tree
    bool selected_visible = false;
    for (const auto& [id, depth] : rows) {
        if (id == m_selected) {
            selected_visible = true;
        }
    }
    if (!selected_visible) {
        m_selected = toy_physics::InvalidTightPoolID;
    }

    float content_h = static_cast<float>(rows.size()) * kRowH;
    ::Rectangle content = {0, 0, panelW - kPad, content_h};
    ::Rectangle view = {};
    GuiScrollPanel({x, y, panelW, inner_h}, nullptr, content, &m_scroll, &view);

    BeginScissorMode((int)view.x, (int)view.y, (int)view.width,
                     (int)view.height);
    float row_y = y + m_scroll.y;
    for (const auto& [id, depth] : rows) {
        const auto& node = bp.GetNode(id);
        float indent = static_cast<float>(depth) * 18.f;
        ::Rectangle arrow = {x + kPad + indent, row_y, 20.f, kCtrlH};
        ::Rectangle label = {arrow.x + 22.f, row_y,
                             panelW - 30.f - indent - kPad, kCtrlH};

        if (!node.m_is_leaf) {
            if (GuiButton(arrow,
                          m_expanded[static_cast<uint32_t>(id)] ? "-" : "+")) {
                m_expanded[static_cast<uint32_t>(id)] =
                    !m_expanded[static_cast<uint32_t>(id)];
            }
        } else {
            GuiLabel(arrow, "*");
        }

        char buf[64];
        if (node.m_is_leaf) {
            snprintf(buf, sizeof(buf), "Node %u (leaf, %u objs)",
                     static_cast<uint32_t>(id), node.m_object_count);
        } else {
            snprintf(buf, sizeof(buf), "Node %u", static_cast<uint32_t>(id));
        }

        bool active = (m_selected == id);
        if (active) {
            GuiSetState(STATE_PRESSED);
        }
        int prev_align = GuiGetStyle(BUTTON, TEXT_ALIGNMENT);
        GuiSetStyle(BUTTON, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);
        if (GuiButton(label, buf)) {
            m_selected = id;
        }
        GuiSetStyle(BUTTON, TEXT_ALIGNMENT, prev_align);
        if (active) {
            GuiSetState(STATE_NORMAL);
        }
        row_y += kRowH;
    }
    EndScissorMode();
}
