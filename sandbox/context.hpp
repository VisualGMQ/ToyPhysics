#pragma once
#include "example.hpp"
#include "raygui.h"
#include "raylib.h"
#include "toy_physics/shape.hpp"
#include <array>
#include <memory>
#include <vector>

#include <string>
#include <unordered_map>

class Context {
public:
    static void Init();
    static void Destroy();
    static Context& GetInst();

    virtual ~Context();

    void Initialize();
    void Shutdown();

    void Update();

    bool ShouldExit() const;
    void Exit();

    template <typename T, typename... Args>
    void RegisterExample(Args&&... args) {
        auto example = std::make_unique<T>(std::forward<Args>(args)...);
        if (!m_cur_example) {
            m_cur_example = example.get();
        }
        m_examples.emplace_back(std::move(example));
    }

    toy_physics::Shape* CreateShape(const toy_physics::BoxGeometry& geometry);
    toy_physics::Shape* CreateShape(
        const toy_physics::SphereGeometry& geometry);
    toy_physics::Shape* CreateShape(
        const toy_physics::CapsuleGeometry& geometry);
    toy_physics::Shape* CreateShape(
        const toy_physics::CylinderGeometry& geometry);
    toy_physics::Shape* CreateShape(
        const toy_physics::ConvexHullGeometry& geometry);

    std::vector<std::unique_ptr<toy_physics::Shape>>& GetShapes();

    [[nodiscard]] const Camera3D& GetCamera() const { return m_camera; }

    void DrawBox(Vector3 center, Vector3 rotation, Vector3 halfExtent,
                 Color color = WHITE) const;
    void DrawSphere(Vector3 center, Vector3 rotation, float radius,
                    Color color = WHITE) const;
    void DrawCapsule(Vector3 center, Vector3 rotation, float height,
                     float radius, Color color = WHITE) const;
    void DrawCylinder(Vector3 center, Vector3 rotation, float height,
                      float radius, Color color = WHITE) const;

    void DrawAABBFlat(const toy_physics::AABB& aabb, Color color) const;
    void DrawAABBFlatWires(const toy_physics::AABB& aabb, Color color) const;

    void DrawBoxFlat(Vector3 center, Vector3 rotation, Vector3 halfExtent,
                     Color color) const;
    void DrawSphereFlat(Vector3 center, Vector3 rotation, float radius,
                        Color color) const;
    void DrawCapsuleFlat(Vector3 center, Vector3 rotation, float height,
                         float radius, Color color) const;
    void DrawCylinderFlat(Vector3 center, Vector3 rotation, float height,
                          float radius, Color color) const;

private:
    static std::unique_ptr<Context> instance;

    std::unique_ptr<toy_physics::ShapeFactory> m_shape_factory;
    std::vector<std::unique_ptr<toy_physics::Shape>> m_shapes;

    void renderUpdate();
    void logicUpdate(float delta_time);
    void initCamera();
    void handleCameraModeSwitch();
    int getCameraMode() const;
    void showHelpMsg() const;
    void renderExampleMenu();

    bool m_should_exit = true;
    Camera3D m_camera = {0};
    float m_camera_move_speed = 0.001;
    size_t m_camera_mode_index = 0;
    bool m_use_camera = true;

    mutable Model m_model_box = {0};
    mutable Model m_model_sphere = {0};
    mutable Model m_model_cylinder = {0};
    Mesh m_sphere_mesh = {0};
    Mesh m_cylinder_mesh = {0};
    Texture2D m_color_texture = {0};
    Shader m_lighting_shader = {0};
    int m_loc_light_dir = -1;
    int m_loc_view_pos = -1;
    int m_loc_light_color = -1;
    int m_loc_ambient_color = -1;
    int m_loc_specular_strength = -1;
    int m_loc_shininess = -1;
    Shader m_flat_shader = {0};
    int m_loc_flat_mvp = -1;
    int m_loc_flat_color = -1;

    std::vector<std::unique_ptr<IExample>> m_examples;
    IExample* m_cur_example = nullptr;

    void handleToggleCamera();
    bool isCameraEnable() const;
    void initTexture();
    void initLightingShader();
    void initFlatShader();
    void loadModels();
    void enableFlatShader(Color color) const;
    void setFlatMvp() const;

    static constexpr std::array<int, 2> m_camera_modes = {
        CAMERA_FREE,
        CAMERA_ORBITAL,
    };

    static constexpr std::array<const char*, 2> m_camera_mode_names = {
        "Free",
        "Orbital",
    };
};

#define SCONTEXT ::Context::GetInst()