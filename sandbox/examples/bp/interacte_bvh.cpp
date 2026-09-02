#include "examples/bp/interacte_bvh.hpp"

#include "context.hpp"
#include "examples/bv/util.hpp"
#include "toy_physics/lowlevel/algorithm.hpp"

#include <cstdio>
#include <span>

namespace {

constexpr float kCtrlH = 26.f;
constexpr float kPad = 4.f;
constexpr float kRowH = kCtrlH + kPad;
constexpr float kSecPad = 12.f;
constexpr float kRayLen = 1000.f;

}  // namespace

void InteracteBvhExample::OnUpdate(float delta_time) {
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

void InteracteBvhExample::pickShape(::Vector2 mouse) {
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

float InteracteBvhExample::raycastShape(const ::Ray& ray,
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

std::unique_ptr<toy_physics::Shape> InteracteBvhExample::createShapeFromItem(
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

void InteracteBvhExample::applyTransform(size_t index) {
    const RenderItem& item = m_render_items[index];
    m_shapes[index]->SetTransform(
        toy_physics::Vector3{item.m_center.x, item.m_center.y, item.m_center.z},
        EulerToQuat(item.m_rotation));
}

void InteracteBvhExample::addShape(RenderItem item) {
    auto shape = createShapeFromItem(item);
    if (!shape) {
        return;
    }
    auto* raw = shape.get();
    m_shapes.push_back(std::move(shape));
    m_render_items.push_back(item);
    applyTransform(m_shapes.size() - 1);

    // deferred until the next Rebuild BVH
    m_broad_phase->AddObjects(std::span<toy_physics::Shape>{raw, 1});
    m_selected_shape = static_cast<int>(m_shapes.size() - 1);
}

void InteracteBvhExample::deleteSelectedShape() {
    if (m_selected_shape < 0 ||
        m_selected_shape >= static_cast<int>(m_shapes.size())) {
        return;
    }

    // deferred until the next Rebuild BVH
    m_broad_phase->RemoveObjects(
        std::span<toy_physics::Shape>{m_shapes[m_selected_shape].get(), 1});
    m_graveyard.push_back(std::move(m_shapes[m_selected_shape]));

    m_shapes.erase(m_shapes.begin() + m_selected_shape);
    m_render_items.erase(m_render_items.begin() + m_selected_shape);
    m_selected_shape = -1;
}

void InteracteBvhExample::recreateSelectedShape() {
    if (m_selected_shape < 0 ||
        m_selected_shape >= static_cast<int>(m_shapes.size())) {
        return;
    }

    auto fresh = createShapeFromItem(m_render_items[m_selected_shape]);
    if (!fresh) {
        return;
    }

    // deferred until the next Rebuild BVH; keep the old shape alive in the
    // graveyard until the pending removal is applied
    m_broad_phase->RemoveObjects(
        std::span<toy_physics::Shape>{m_shapes[m_selected_shape].get(), 1});
    m_graveyard.push_back(std::move(m_shapes[m_selected_shape]));
    m_shapes[m_selected_shape] = std::move(fresh);
    applyTransform(m_selected_shape);

    m_broad_phase->AddObjects(
        std::span<toy_physics::Shape>{m_shapes[m_selected_shape].get(), 1});
}

toy_physics::BVHSplitPolicy InteracteBvhExample::getSplitPolicy() const {
    return m_split_policy_index == 0 ? toy_physics::BVHSplitPolicy::Median
                                     : toy_physics::BVHSplitPolicy::SAH;
}

void InteracteBvhExample::applySplitPolicy() {
    // split policy is fixed at construction: recreate the broad phase with
    // the selected policy and rebuild it over the existing shapes
    m_broad_phase = std::make_unique<
        toy_physics::AABBBroadPhase<toy_physics::BVHBuildPolicy::TopDown>>(
        getSplitPolicy());

    for (const auto& shape : m_shapes) {
        m_broad_phase->AddObjects(
            std::span<toy_physics::Shape>{shape.get(), 1});
    }
    m_broad_phase->ApplyModify();
    m_tree.Reset(m_broad_phase->GetNodes());
    m_applied_policy_index = m_split_policy_index;
}

void InteracteBvhExample::rebuildBvh() {
    // make sure the broad phase matches the currently selected policy
    if (m_applied_policy_index != m_split_policy_index) {
        applySplitPolicy();
    }

    // force a full rebuild: drop everything, then re-register everything
    for (const auto& shape : m_shapes) {
        m_broad_phase->RemoveObjects(
            std::span<toy_physics::Shape>{shape.get(), 1});
    }
    m_broad_phase->ApplyModify();
    // pending removals were applied, removed shapes can be released
    m_graveyard.clear();

    for (const auto& shape : m_shapes) {
        m_broad_phase->AddObjects(
            std::span<toy_physics::Shape>{shape.get(), 1});
    }
    m_broad_phase->ApplyModify();
    m_tree.Reset(m_broad_phase->GetNodes());
}

void InteracteBvhExample::OnRender3D(float delta_time) {
    if (!m_initialized) {
        m_broad_phase = std::make_unique<
            toy_physics::AABBBroadPhase<toy_physics::BVHBuildPolicy::TopDown>>(
            getSplitPolicy());
        m_applied_policy_index = m_split_policy_index;
        m_initialized = true;
    }

    for (const auto& item : m_render_items) {
        switch (item.m_type) {
            case toy_physics::Geometry::Type::Sphere:
                m_ctx.DrawSphere(item.m_center, item.m_rotation, item.m_radius,
                                 RED);
                break;
            case toy_physics::Geometry::Type::Box:
                m_ctx.DrawBox(item.m_center, item.m_rotation,
                              item.m_half_extent, BLUE);
                break;
            case toy_physics::Geometry::Type::Cylinder:
                m_ctx.DrawCylinder(item.m_center, item.m_rotation,
                                   item.m_height, item.m_radius, GREEN);
                break;
            case toy_physics::Geometry::Type::Capsule:
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

    for (size_t i = 0; i < nodes.size(); ++i) {
        const auto& node = nodes[i];
        if (node.m_data.IsLeaf()) {
            ++m_leaf_count;
            bp_util::DrawAABBWire(node.m_aabb, GREEN);
        } else {
            bp_util::DrawAABBWire(node.m_aabb, RED);
        }
    }

    if (m_tree.m_selected >= 0 &&
        m_tree.m_selected < static_cast<int>(nodes.size())) {
        const auto& node = nodes[m_tree.m_selected];
        m_ctx.DrawAABBFlat(node.m_aabb, ::Color{255, 150, 0, 90});
        m_ctx.DrawAABBFlatWires(node.m_aabb, YELLOW);
    }
}

void InteracteBvhExample::renderCreateSection(float x, float& y, float panelW) {
    GuiGroupBox({x, y, panelW, kRowH * 4 + kPad * 2}, "Create Shape");
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
    y += kRowH + kSecPad;

    int prev_policy = m_split_policy_index;
    GuiComboBox({x, y, panelW, kCtrlH}, "Median;SAH", &m_split_policy_index);
    if (m_split_policy_index != prev_policy) {
        applySplitPolicy();
    }
    y += kRowH + kSecPad;

    if (GuiButton({x, y, panelW, kCtrlH}, "Rebuild BVH")) {
        rebuildBvh();
    }
    y += kRowH + kSecPad;
}

void InteracteBvhExample::renderSelectedSection(float x, float& y,
                                                float panelW) {
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

    int row_count = 6;
    switch (item.m_type) {
        case toy_physics::Geometry::Type::Box:
            row_count += 3;
            break;
        case toy_physics::Geometry::Type::Capsule:
        case toy_physics::Geometry::Type::Cylinder:
            row_count += 2;
            break;
        default:
            row_count += 1;
            break;
    }
    row_count += 1;  // delete button

    GuiGroupBox({x, y, panelW, kRowH * (row_count + 1) + kPad * 2},
                "Selected Shape");
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
        applyTransform(sel);
    }

    bool geometry_changed = false;
    switch (item.m_type) {
        case toy_physics::Geometry::Type::Box: {
            float hx = item.m_half_extent.x, hy = item.m_half_extent.y,
                  hz = item.m_half_extent.z;
            lbl({x, y, kLabelW, kCtrlH}, "Half X");
            GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr,
                         nullptr, &hx, 0.1f, 10);
            y += kRowH;
            lbl({x, y, kLabelW, kCtrlH}, "Half Y");
            GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr,
                         nullptr, &hy, 0.1f, 10);
            y += kRowH;
            lbl({x, y, kLabelW, kCtrlH}, "Half Z");
            GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr,
                         nullptr, &hz, 0.1f, 10);
            y += kRowH;
            if (hx != item.m_half_extent.x || hy != item.m_half_extent.y ||
                hz != item.m_half_extent.z) {
                item.m_half_extent = {hx, hy, hz};
                geometry_changed = true;
            }
            break;
        }
        case toy_physics::Geometry::Type::Capsule:
        case toy_physics::Geometry::Type::Cylinder: {
            float r = item.m_radius, h = item.m_height;
            lbl({x, y, kLabelW, kCtrlH}, "Radius");
            GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr,
                         nullptr, &r, 0.1f, 10);
            y += kRowH;
            lbl({x, y, kLabelW, kCtrlH}, "Height");
            GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr,
                         nullptr, &h, 0.1f, 10);
            y += kRowH;
            if (r != item.m_radius || h != item.m_height) {
                item.m_radius = r;
                item.m_height = h;
                geometry_changed = true;
            }
            break;
        }
        default: {
            float r = item.m_radius;
            lbl({x, y, kLabelW, kCtrlH}, "Radius");
            GuiSliderBar({x + kLabelW + kPad, y, kSliderW, kCtrlH}, nullptr,
                         nullptr, &r, 0.1f, 10);
            y += kRowH;
            if (r != item.m_radius) {
                item.m_radius = r;
                geometry_changed = true;
            }
            break;
        }
    }

    if (geometry_changed) {
        recreateSelectedShape();
    }

    if (GuiButton({x, y, panelW, kCtrlH}, "Delete")) {
        deleteSelectedShape();
    }
    y += kRowH + kSecPad;
}

void InteracteBvhExample::OnRender2D(float delta_time) {
    m_panel.Begin("Interacte BVH");

    float x = m_panel.GetContentTopLeft().x;
    float y = m_panel.GetContentTopLeft().y;
    float panelW = m_panel.m_bounds.width - 8.f;

    renderCreateSection(x, y, panelW);
    renderSelectedSection(x, y, panelW);

    char buf[128];
    snprintf(buf, sizeof(buf), "Objects: %zu", m_shapes.size());
    DrawText(buf, (int)x + 4, (int)y, 18, BLACK);
    y += 22;
    snprintf(buf, sizeof(buf), "Nodes: %zu (leaves: %zu)", m_node_count,
             m_leaf_count);
    DrawText(buf, (int)x + 4, (int)y, 18, BLACK);
    y += 24;

    float tree_h = m_panel.m_bounds.y + m_panel.m_bounds.height - y - 6.f;
    m_tree.Render(x, y, panelW, tree_h, m_broad_phase->GetNodes());
    m_panel.End();
}
