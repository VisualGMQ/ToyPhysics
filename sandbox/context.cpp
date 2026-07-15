#include "context.hpp"

#include "raymath.h"
#include "rlgl.h"

#include <fstream>

#include "toy_physics/log.hpp"

constexpr float WINDOW_INIT_W = 1024;
constexpr float WINDOW_INIT_H = 720;
const char* WINDOW_TITLE = "ToyPhysics Sandbox";

std::unique_ptr<Context> Context::instance;

Context& Context::GetInst() {
    return *instance;
}

Context::~Context() {}

void Context::Initialize() {
    m_should_exit = false;

    InitWindow(WINDOW_INIT_W, WINDOW_INIT_H, WINDOW_TITLE);
    SetWindowState(FLAG_WINDOW_RESIZABLE);
    SetTargetFPS(120);
    initTexture();
    initLightingShader();
    loadModels();
    initCamera();
}

void Context::initCamera() {
    m_camera.position = Vector3{0.0, 10.0, 10.0};
    m_camera.target = Vector3{0.0, 0.0, 0.0};
    m_camera.up = Vector3{0.0, 1.0, 0.0};
    m_camera.fovy = 45.0;
    m_camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();
}

void Context::initTexture() {
    Image img = GenImageColor(2, 2, BLANK);
    Color* pixels = (Color*)img.data;
    pixels[0] = RED;
    pixels[1] = GREEN;
    pixels[2] = BLUE;
    pixels[3] = YELLOW;
    m_color_texture = LoadTextureFromImage(img);
    UnloadImage(img);
}

void Context::initLightingShader() {
    m_lighting_shader = LoadShader("sandbox/assets/vert.glsl", "sandbox/assets/frag.glsl");

    m_loc_light_dir         = GetShaderLocation(m_lighting_shader, "lightDir");
    m_loc_view_pos          = GetShaderLocation(m_lighting_shader, "viewPos");
    m_loc_light_color       = GetShaderLocation(m_lighting_shader, "lightColor");
    m_loc_ambient_color     = GetShaderLocation(m_lighting_shader, "ambientColor");
    m_loc_specular_strength = GetShaderLocation(m_lighting_shader, "specularStrength");
    m_loc_shininess         = GetShaderLocation(m_lighting_shader, "shininess");
}

void Context::loadModels() {
    m_model_box = LoadModel("sandbox/assets/cube.obj");
    m_model_sphere = LoadModel("sandbox/assets/sphere.obj");
    m_model_cylinder = LoadModel("sandbox/assets/cylinder.obj");
    m_model_semi_sphere = LoadModel("sandbox/assets/semi-sphere.obj");

    m_model_box.materials[0].shader = m_lighting_shader;
    m_model_box.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    m_model_box.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture =
        m_color_texture;
    m_model_sphere.materials[0].shader = m_lighting_shader;
    m_model_sphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    m_model_sphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture =
        m_color_texture;
    m_model_cylinder.materials[0].shader = m_lighting_shader;
    m_model_cylinder.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    m_model_cylinder.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture =
        m_color_texture;
    m_model_semi_sphere.materials[0].shader = m_lighting_shader;
    m_model_semi_sphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    m_model_semi_sphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture =
        m_color_texture;
}

void Context::handleCameraModeSwitch() {
    if (IsKeyPressed(KEY_TAB)) {
        m_camera_mode_index = (m_camera_mode_index + 1) % m_camera_modes.size();
    }
}

int Context::getCameraMode() const {
    return m_camera_modes[m_camera_mode_index];
}

void Context::showHelpMsg() const {
    const std::string camera_mode_name =
        m_camera_mode_names[m_camera_mode_index];

    DrawText("Press ESC to quit", 20, 20, 20, BLACK);
    const std::string camera_mode_text =
        "Tab to switch camera mode: " + camera_mode_name;
    DrawText(camera_mode_text.c_str(), 20, 50, 20, BLACK);
    DrawText("Press L-Alt to toggle mouse", 20, 80, 20, BLACK);
}

void Context::handleToggleCamera() {
    if (!IsKeyPressed(KEY_LEFT_ALT)) {
        return;
    }

    m_use_camera = !m_use_camera;
    if (m_use_camera) {
        DisableCursor();
    } else {
        EnableCursor();
    }
}

bool Context::isCameraEnable() const {
    return m_use_camera;
}

void Context::Shutdown() {
    m_should_exit = true;

    UnloadModel(m_model_box);
    UnloadModel(m_model_sphere);
    UnloadModel(m_model_cylinder);
    UnloadModel(m_model_semi_sphere);
    UnloadTexture(m_color_texture);
    UnloadShader(m_lighting_shader);
    CloseWindow();
}

void Context::renderUpdate() {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    // 3d mode
    BeginMode3D(m_camera);
    {
        float lightDir[3] = {0.5f, -1.0f, -0.5f};
        float lightColor[3] = {1.0f, 1.0f, 1.0f};
        float ambientColor[3] = {0.15f, 0.15f, 0.15f};
        float viewPos[3] = {m_camera.position.x, m_camera.position.y, m_camera.position.z};
        float specStrength = 0.5f;
        float shininess = 32.0f;

        SetShaderValue(m_lighting_shader, m_loc_light_dir, lightDir, SHADER_UNIFORM_VEC3);
        SetShaderValue(m_lighting_shader, m_loc_view_pos, viewPos, SHADER_UNIFORM_VEC3);
        SetShaderValue(m_lighting_shader, m_loc_light_color, lightColor, SHADER_UNIFORM_VEC3);
        SetShaderValue(m_lighting_shader, m_loc_ambient_color, ambientColor, SHADER_UNIFORM_VEC3);
        SetShaderValue(m_lighting_shader, m_loc_specular_strength, &specStrength, SHADER_UNIFORM_FLOAT);
        SetShaderValue(m_lighting_shader, m_loc_shininess, &shininess, SHADER_UNIFORM_FLOAT);

        drawBox(Vector3{0, 0, 0}, Vector3{0, 0, 0}, Vector3{2, 1, 1}, WHITE);
        drawSphere(Vector3{5, 0, 0}, Vector3{0, 0, 0}, 1.0f, WHITE);
        drawCylinder(Vector3{-5, 0, 0}, Vector3{0, 0, 0}, 2.0f, 1.0f, WHITE);
        drawCapsule(Vector3{0, 0, -5}, Vector3{90, 0, 0}, 2.0f, 1.0f, WHITE);
        DrawGrid(100, 1.0f);
    }
    EndMode3D();

    // 2d mode
    {
        showHelpMsg();
    }
    EndDrawing();
}

void Context::drawBox(Vector3 center, Vector3 rotation, Vector3 halfExtent,
                      Color color) const {
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    rlScalef(halfExtent.x * 2.0f, halfExtent.y * 2.0f, halfExtent.z * 2.0f);
    DrawModel(m_model_box, Vector3{0, 0, 0}, 1.0f, color);
    rlPopMatrix();
}

void Context::drawSphere(Vector3 center, Vector3 rotation, float radius, Color color) const {
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    float s = radius * 2.0f;
    rlScalef(s, s, s);
    DrawModel(m_model_sphere, Vector3{0, 0, 0}, 1.0f, color);
    rlPopMatrix();
}

void Context::drawCylinder(Vector3 center, Vector3 rotation, float height, float radius, Color color) const {
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    rlScalef(radius * 2.0f, height, radius * 2.0f);
    DrawModel(m_model_cylinder, Vector3{0, 0, 0}, 1.0f, color);
    rlPopMatrix();
}

void Context::drawCapsule(Vector3 center, Vector3 rotation, float height, float radius, Color color) const {
    float halfHeight = height * 0.5f;

    // cylinder
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    rlScalef(radius * 2.0f, height, radius * 2.0f);
    DrawModel(m_model_cylinder, Vector3{0, 0, 0}, 1.0f, color);
    rlPopMatrix();

    // top semi-sphere
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    rlTranslatef(0.0f, halfHeight, 0.0f);
    rlScalef(radius * 2.0f, radius * 2.0f, radius * 2.0f);
    DrawModel(m_model_semi_sphere, Vector3{0, 0, 0}, 1.0f, color);
    rlPopMatrix();

    // bottom semi-sphere (flipped down)
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    rlTranslatef(0.0f, -halfHeight, 0.0f);
    rlRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    rlScalef(radius * 2.0f, radius * 2.0f, radius * 2.0f);
    DrawModel(m_model_semi_sphere, Vector3{0, 0, 0}, 1.0f, color);
    rlPopMatrix();
}

void Context::logicUpdate(float delta_time) {
    handleToggleCamera();
    if (isCameraEnable()) {
        UpdateCamera(&m_camera, getCameraMode());

        // reset target to origin point
        if (IsKeyPressed(KEY_Z)) {
            m_camera.target = Vector3{0, 0, 0};
        }

        handleCameraModeSwitch();
    }
}

void Context::Update() {
    while (!WindowShouldClose()) {
        logicUpdate(GetFrameTime());
        renderUpdate();
    }
}

bool Context::ShouldExit() const {
    return m_should_exit;
}

void Context::Exit() {
    m_should_exit = true;
}

void Context::Init() {
    if (!instance) {
        instance = std::unique_ptr<Context>(new Context());
    } else {
        LOGW("inited context singleton twice!");
    }
}

void Context::Destroy() {
    instance.reset();
}