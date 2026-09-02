#include "examples/bp/bvh.hpp"

#include "context.hpp"
#include "examples/bv/util.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <random>
#include <span>
#include <unordered_set>

namespace {

constexpr size_t kObjectCount = 10000;
constexpr float kWorldHalfExtent = 75.f;

}  // namespace

void BvhExample::OnUpdate(float delta_time) {}

void BvhExample::rebuildScene() {
    auto& shapes = m_ctx.GetShapes();

    if (!m_broad_phase) {
        m_broad_phase = std::make_unique<
            toy_physics::AABBBroadPhase<toy_physics::BVHBuildPolicy::TopDown>>(
            getSplitPolicy());
        m_applied_policy_index = m_split_policy_index;
    }

    if (!shapes.empty()) {
        for (const auto& shape : shapes) {
            m_broad_phase->RemoveObjects(
                std::span<toy_physics::Shape>{shape.get(), 1});
        }
        // apply removals before destroying the shapes
        m_broad_phase->ApplyModify();
    }
    shapes.clear();
    m_render_items.clear();
    m_node_count = 0;
    m_leaf_count = 0;

    std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> pos_dist{-kWorldHalfExtent,
                                                   kWorldHalfExtent};
    std::uniform_real_distribution<float> size_dist{0.5f, 2.0f};
    std::uniform_real_distribution<float> rot_dist{0.f, 360.f};
    std::uniform_int_distribution<int> kind_dist{0, 3};

    std::vector<toy_physics::Shape*> ptrs;
    ptrs.reserve(kObjectCount);
    shapes.reserve(kObjectCount);
    m_render_items.reserve(kObjectCount);

    for (size_t i = 0; i < kObjectCount; ++i) {
        ::Vector3 center{pos_dist(rng), pos_dist(rng), pos_dist(rng)};
        ::Vector3 rotation{rot_dist(rng), rot_dist(rng), rot_dist(rng)};

        RenderItem item;
        item.m_center = center;
        item.m_rotation = rotation;

        std::unique_ptr<toy_physics::Shape> shape;
        switch (kind_dist(rng)) {
            case 0: {
                float radius = size_dist(rng);
                shape = std::make_unique<toy_physics::Shape>(
                    toy_physics::SphereGeometry{radius});
                item.m_type = toy_physics::Geometry::Type::Sphere;
                item.m_radius = radius;
                break;
            }
            case 1: {
                ::Vector3 half_extent{size_dist(rng), size_dist(rng),
                                      size_dist(rng)};
                shape = std::make_unique<toy_physics::Shape>(
                    toy_physics::BoxGeometry{
                        toy_physics::Vector3{half_extent.x, half_extent.y,
                                             half_extent.z}
                });
                item.m_type = toy_physics::Geometry::Type::Box;
                item.m_half_extent = half_extent;
                break;
            }
            case 2: {
                float radius = size_dist(rng);
                float height = size_dist(rng) * 2.f;
                shape = std::make_unique<toy_physics::Shape>(
                    toy_physics::CylinderGeometry{radius, height * 0.5f});
                item.m_type = toy_physics::Geometry::Type::Cylinder;
                item.m_radius = radius;
                item.m_height = height;
                break;
            }
            default: {
                float radius = size_dist(rng);
                float height = size_dist(rng) * 2.f;
                shape = std::make_unique<toy_physics::Shape>(
                    toy_physics::CapsuleGeometry{radius, height * 0.5f});
                item.m_type = toy_physics::Geometry::Type::Capsule;
                item.m_radius = radius;
                item.m_height = height;
                break;
            }
        }

        shape->SetTransform(toy_physics::Vector3{center.x, center.y, center.z},
                            EulerToQuat(rotation));
        ptrs.push_back(shape.get());
        shapes.push_back(std::move(shape));
        m_render_items.push_back(item);
    }

    for (toy_physics::Shape* shape : ptrs) {
        m_broad_phase->AddObjects(std::span<toy_physics::Shape>{shape, 1});
    }
    m_broad_phase->ApplyModify();
    m_tree.Reset(m_broad_phase->GetNodes());
}

toy_physics::BVHSplitPolicy BvhExample::getSplitPolicy() const {
    return m_split_policy_index == 0 ? toy_physics::BVHSplitPolicy::Median
                                     : toy_physics::BVHSplitPolicy::SAH;
}

void BvhExample::applySplitPolicy() {
    // split policy is fixed at construction: recreate the broad phase with
    // the selected policy and rebuild it over the existing shapes
    m_broad_phase = std::make_unique<
        toy_physics::AABBBroadPhase<toy_physics::BVHBuildPolicy::TopDown>>(
        getSplitPolicy());

    const auto& shapes = m_ctx.GetShapes();
    for (const auto& shape : shapes) {
        m_broad_phase->AddObjects(
            std::span<toy_physics::Shape>{shape.get(), 1});
    }
    m_broad_phase->ApplyModify();
    m_tree.Reset(m_broad_phase->GetNodes());
    m_applied_policy_index = m_split_policy_index;
}

void BvhExample::rebuildBvh() {
    // make sure the broad phase matches the currently selected policy
    if (m_applied_policy_index != m_split_policy_index) {
        applySplitPolicy();
    }

    const auto& shapes = m_ctx.GetShapes();

    // force a full rebuild: drop everything, then re-register everything
    for (const auto& shape : shapes) {
        m_broad_phase->RemoveObjects(
            std::span<toy_physics::Shape>{shape.get(), 1});
    }
    m_broad_phase->ApplyModify();

    for (const auto& shape : shapes) {
        m_broad_phase->AddObjects(
            std::span<toy_physics::Shape>{shape.get(), 1});
    }

    auto start = std::chrono::steady_clock::now();
    m_broad_phase->ApplyModify();
    auto end = std::chrono::steady_clock::now();
    m_rebuild_time_ms =
        std::chrono::duration<double, std::milli>(end - start).count();
    m_tree.Reset(m_broad_phase->GetNodes());
}

void BvhExample::OnRender3D(float delta_time) {
    if (!m_initialized) {
        rebuildScene();
        m_initialized = true;
    }

    const auto& nodes = m_broad_phase->GetNodes();
    m_node_count = nodes.size();
    m_leaf_count = 0;
    for (const auto& node : nodes) {
        if (node.m_data.IsLeaf()) {
            ++m_leaf_count;
        }
        if (m_draw_bvh_aabbs) {
            bp_util::DrawAABBWire(node.m_aabb,
                                  node.m_data.IsLeaf() ? GREEN : RED);
        }
    }

    // collect the shapes covered by the selected node
    std::unordered_set<const toy_physics::Shape*> highlighted;
    const toy_physics::BoundingBox* selected_aabb = nullptr;
    if (m_tree.m_selected >= 0 &&
        m_tree.m_selected < static_cast<int>(nodes.size())) {
        const auto& node = nodes[m_tree.m_selected];
        selected_aabb = &node.m_aabb;

        std::function<std::pair<uint32_t, uint32_t>(uint32_t)> prim_range =
            [&](uint32_t idx) {
                const auto& n = nodes[idx];
                if (n.m_data.IsLeaf()) {
                    uint32_t off = n.m_data.GetPrimOffset();
                    return std::make_pair(off, off + n.m_data.GetPrimNum());
                }
                auto [lmin, lmax] = prim_range(n.m_data.GetPositiveChild());
                auto [rmin, rmax] = prim_range(n.m_data.GetNegativeChild());
                return std::make_pair(std::min(lmin, rmin),
                                      std::max(lmax, rmax));
            };
        auto [pmin, pmax] =
            prim_range(static_cast<uint32_t>(m_tree.m_selected));

        for (uint32_t p = pmin; p < pmax; ++p) {
            highlighted.insert(m_broad_phase->GetPrimitiveByOrder(p));
        }
    }

    // highlighted objects get a slightly scaled-up green shell drawn first,
    // showing through around the silhouette as a thick green outline
    const auto& ctx_shapes = m_ctx.GetShapes();
    for (size_t i = 0; i < m_render_items.size(); ++i) {
        const auto& item = m_render_items[i];
        bool hl =
            i < ctx_shapes.size() && highlighted.contains(ctx_shapes[i].get());

        switch (item.m_type) {
            case toy_physics::Geometry::Type::Sphere:
                if (hl) {
                    m_ctx.DrawSphereFlat(item.m_center, item.m_rotation,
                                         item.m_radius * 1.1f, GREEN);
                }
                if (m_draw_shapes) {
                    m_ctx.DrawSphere(item.m_center, item.m_rotation,
                                     item.m_radius, RED);
                }
                break;
            case toy_physics::Geometry::Type::Box:
                if (hl) {
                    m_ctx.DrawBoxFlat(item.m_center, item.m_rotation,
                                      Vector3Scale(item.m_half_extent, 1.1f),
                                      GREEN);
                }
                if (m_draw_shapes) {
                    m_ctx.DrawBox(item.m_center, item.m_rotation,
                                  item.m_half_extent, BLUE);
                }
                break;
            case toy_physics::Geometry::Type::Cylinder:
                if (hl) {
                    m_ctx.DrawCylinderFlat(item.m_center, item.m_rotation,
                                           item.m_height * 1.1f,
                                           item.m_radius * 1.1f, GREEN);
                }
                if (m_draw_shapes) {
                    m_ctx.DrawCylinder(item.m_center, item.m_rotation,
                                       item.m_height, item.m_radius, ORANGE);
                }
                break;
            case toy_physics::Geometry::Type::Capsule:
                if (hl) {
                    m_ctx.DrawCapsuleFlat(item.m_center, item.m_rotation,
                                          item.m_height * 1.1f,
                                          item.m_radius * 1.1f, GREEN);
                }
                if (m_draw_shapes) {
                    m_ctx.DrawCapsule(item.m_center, item.m_rotation,
                                      item.m_height, item.m_radius, PURPLE);
                }
                break;
            default:
                break;
        }
    }

    // draw the selected node AABB after all objects so its translucency
    // blends over them and stays clearly visible
    if (selected_aabb) {
        m_ctx.DrawAABBFlat(*selected_aabb, ::Color{255, 150, 0, 90});
        m_ctx.DrawAABBFlatWires(*selected_aabb, YELLOW);
    }
}

void BvhExample::OnRender2D(float delta_time) {
    static constexpr float kCtrlH = 28.f;
    static constexpr float kPad = 6.f;
    static constexpr float kRowH = kCtrlH + kPad;
    static constexpr float kSecPad = 16.f;

    m_panel.Begin("BVH Broad Phase");

    float x = m_panel.GetContentTopLeft().x;
    float y = m_panel.GetContentTopLeft().y;
    float panelW = m_panel.m_bounds.width - 8.f;

    GuiCheckBox({x, y, kCtrlH, kCtrlH}, "Draw Shapes", &m_draw_shapes);
    y += kRowH;
    GuiCheckBox({x, y, kCtrlH, kCtrlH}, "Draw BVH AABBs", &m_draw_bvh_aabbs);
    y += kRowH + kSecPad;

    if (GuiButton({x, y, panelW, kCtrlH}, "Regenerate Scene")) {
        rebuildScene();
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

    char buf[128];
    snprintf(buf, sizeof(buf), "Objects: %zu", m_ctx.GetShapes().size());
    DrawText(buf, (int)x, (int)y, 18, BLACK);
    y += 22;
    snprintf(buf, sizeof(buf), "Nodes: %zu (leaves: %zu)", m_node_count,
             m_leaf_count);
    DrawText(buf, (int)x, (int)y, 18, BLACK);
    y += 22;
    snprintf(buf, sizeof(buf), "Rebuild: %.2f ms", m_rebuild_time_ms);
    DrawText(buf, (int)x, (int)y, 18, BLACK);
    y += 22;
    DrawText("Green: leaf AABBs", (int)x, (int)y, 18, BLACK);
    y += 22;
    DrawText("Red: internal node AABBs", (int)x, (int)y, 18, BLACK);
    y += 24;

    float tree_h = m_panel.m_bounds.y + m_panel.m_bounds.height - y - 6.f;
    m_tree.Render(x, y, panelW, tree_h, m_broad_phase->GetNodes());
    m_panel.End();
}
