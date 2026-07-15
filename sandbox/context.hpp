#pragma once
#include "raylib.h"
#include <memory>
#include <array>

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

private:
    static std::unique_ptr<Context> instance;

    void renderUpdate();
    void logicUpdate(float delta_time);
    void initCamera();
    void handleCameraModeSwitch();
    int getCameraMode() const;
    void showHelpMsg() const;

    bool m_should_exit = true;
    Camera3D m_camera = {0};
    float m_camera_move_speed = 0.001;
    size_t m_camera_mode_index = 0;
    bool m_use_camera = true;

    mutable Model m_model_box = {0};
    mutable Model m_model_sphere = {0};
    mutable Model m_model_cylinder = {0};
    mutable Model m_model_semi_sphere = {0};
    Texture2D m_color_texture = {0};
    Shader m_lighting_shader = {0};
    int m_loc_light_dir = -1;
    int m_loc_view_pos = -1;
    int m_loc_light_color = -1;
    int m_loc_ambient_color = -1;
    int m_loc_specular_strength = -1;
    int m_loc_shininess = -1;

    void handleToggleCamera();
    bool isCameraEnable() const;
    void initTexture();
    void initLightingShader();
    void loadModels();

    void drawBox(Vector3 center, Vector3 rotation, Vector3 halfExtent, Color color) const;
    void drawSphere(Vector3 center, Vector3 rotation, float radius, Color color) const;
    void drawCapsule(Vector3 center, Vector3 rotation, float height, float radius, Color color) const;
    void drawCylinder(Vector3 center, Vector3 rotation, float height, float radius, Color color) const;

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