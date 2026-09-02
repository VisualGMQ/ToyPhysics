#pragma once
#include "example.hpp"
#include "examples/bp/util.hpp"
#include "raylib.h"
#include "toy_physics/bp/bvh_incresement.hpp"
#include "toy_physics/geometry/geometry.hpp"

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class IncBvhExample : public IExample {
public:
    using IExample::IExample;

    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
    void OnRender2D(float delta_time) override;

private:
    struct RenderItem {
        toy_physics::Geometry::Type m_type{};
        ::Vector3 m_center{};
        ::Vector3 m_rotation{};
        ::Vector3 m_half_extent{1, 1, 1};
        float m_radius{1};
        float m_height{2};
    };

    struct TreeView {
        toy_physics::TightPoolID m_selected{toy_physics::InvalidTightPoolID};
        std::unordered_map<uint32_t, bool> m_expanded;
        ::Vector2 m_scroll{};

        void Render(float x, float y, float panelW, float areaH,
                    const IncBvhExample& self);
    };

    std::unique_ptr<toy_physics::Shape> createShapeFromItem(
        const RenderItem& item) const;
    void applyTransform(size_t index);
    void addShape(RenderItem item);
    void bulkAddShapes(int count);
    void deleteSelectedShape();
    void queueMove(size_t index);
    void applyChanges();
    void pickShape(::Vector2 mouse);
    float raycastShape(const ::Ray& ray, const RenderItem& item) const;
    void renderCreateSection(float x, float& y, float panelW);
    void renderSelectedSection(float x, float& y, float panelW);

    bool m_initialized = false;
    bool m_draw_bvh_aabbs = true;
    int m_selected_shape = -1;
    int m_bulk_count = 100;
    float m_rebuild_time_ms = 0.f;
    size_t m_node_count = 0;
    size_t m_leaf_count = 0;

    std::unique_ptr<
        toy_physics::AABBBroadPhase<toy_physics::BVHBuildPolicy::Incremental>>
        m_broad_phase;
    std::vector<std::unique_ptr<toy_physics::Shape>> m_shapes;
    std::vector<std::unique_ptr<toy_physics::Shape>> m_graveyard;
    std::vector<RenderItem> m_render_items;
    std::unordered_set<toy_physics::Shape*> m_pending_add_set;
    std::unordered_set<toy_physics::Shape*> m_pending_remove_set;
    std::unordered_set<toy_physics::Shape*> m_delete_requested;
    bp_util::ResizablePanel m_panel;
    TreeView m_tree;
};
