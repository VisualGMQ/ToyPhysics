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
    GuiLoadStyleDefault();
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
    m_lighting_shader =
        LoadShader("sandbox/assets/vert.glsl", "sandbox/assets/frag.glsl");

    m_loc_light_dir = GetShaderLocation(m_lighting_shader, "lightDir");
    m_loc_view_pos = GetShaderLocation(m_lighting_shader, "viewPos");
    m_loc_light_color = GetShaderLocation(m_lighting_shader, "lightColor");
    m_loc_ambient_color = GetShaderLocation(m_lighting_shader, "ambientColor");
    m_loc_specular_strength =
        GetShaderLocation(m_lighting_shader, "specularStrength");
    m_loc_shininess = GetShaderLocation(m_lighting_shader, "shininess");
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

    std::string example_hint_str = "No Example";
    if (m_cur_example) {
        example_hint_str = std::string{"Example: "} + m_cur_example->GetName();
    }
    constexpr size_t font_size = 20;
    DrawText(example_hint_str.c_str(), 20, 20, font_size, ORANGE);
    DrawText("Press ESC to quit", 20, 40, font_size, BLACK);
    const std::string camera_mode_text =
        "Tab to switch camera mode: " + camera_mode_name;
    DrawText(camera_mode_text.c_str(), 20, 60, font_size, BLACK);
    DrawText("Press L-Alt to toggle mouse", 20, 80, font_size, BLACK);
}

void Context::renderExampleMenu() {
    static constexpr float kHeaderHeight = 24.f;
    static constexpr float kBtnHeight = 28.f;
    static constexpr float kBtnPad = 2.f;
    static constexpr float kMinW = 150.f;
    static constexpr float kMinH = kHeaderHeight + kBtnHeight;
    static constexpr float kResizeSz = 16.f;

    static bool visible = true;
    static Rectangle bounds = {10.f, 100.f, 220.f, 300.f};
    static bool drag = false;
    static bool sizing = false;
    static Vector2 dragStart = {};
    static Rectangle dragStartBounds = {};
    static Vector2 scroll = {};

    if (IsKeyPressed(KEY_M)) visible = !visible;
    if (!visible) return;

    Vector2 mouse = GetMousePosition();

    Rectangle header = {bounds.x, bounds.y, bounds.width, kHeaderHeight};
    Rectangle resizeHit = {bounds.x + bounds.width - kResizeSz,
                           bounds.y + bounds.height - kResizeSz, kResizeSz,
                           kResizeSz};

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        if (CheckCollisionPointRec(mouse, resizeHit)) {
            sizing = true;
            dragStart = mouse;
            dragStartBounds = bounds;
        } else if (CheckCollisionPointRec(mouse, header)) {
            drag = true;
            dragStart = mouse;
            dragStartBounds = bounds;
        }
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        drag = false;
        sizing = false;
    }
    if (drag) {
        bounds.x = dragStartBounds.x + (mouse.x - dragStart.x);
        bounds.y = dragStartBounds.y + (mouse.y - dragStart.y);
    }
    if (sizing) {
        bounds.width =
            std::max(dragStartBounds.width + (mouse.x - dragStart.x), kMinW);
        bounds.height =
            std::max(dragStartBounds.height + (mouse.y - dragStart.y), kMinH);
    }

    GuiWindowBox(bounds, "Examples");

    Rectangle closeBtn = {bounds.x + bounds.width - 24.f, bounds.y + 3.f, 18.f,
                          18.f};
    if (GuiButton(closeBtn, "#143#")) {
        visible = false;
        return;
    }

    Rectangle contentArea = {bounds.x, bounds.y + kHeaderHeight, bounds.width,
                             bounds.height - kHeaderHeight};
    float scrollBarW = (float)GuiGetStyle(SCROLLBAR, ARROWS_SIZE) + 8.f;
    float btnW = contentArea.width - scrollBarW - kBtnPad * 2;
    float contentH = m_examples.size() * (kBtnHeight + kBtnPad);
    Rectangle content = {0, 0, btnW + kBtnPad,
                         std::max(contentH, contentArea.height)};
    Rectangle view = {};

    GuiScrollPanel(contentArea, nullptr, content, &scroll, &view);

    BeginScissorMode((int)view.x, (int)view.y, (int)view.width,
                     (int)view.height);

    float bx = bounds.x + kBtnPad + scroll.x;
    float by = bounds.y + kHeaderHeight + kBtnPad + scroll.y;
    int i = 0;
    for (const auto& example : m_examples) {
        Rectangle btn = {bx, by + i * (kBtnHeight + kBtnPad), btnW, kBtnHeight};
        bool active = (example.get() == m_cur_example);
        if (active) GuiSetState(STATE_PRESSED);
        if (GuiButton(btn, example->GetName().c_str())) {
            m_cur_example = example.get();
        }
        if (active) GuiSetState(STATE_NORMAL);
        i++;
    }

    EndScissorMode();

    GuiLabel(resizeHit, "#033#");
    Rectangle hintRect = {bounds.x + 4.f, bounds.y + bounds.height - 14.f,
                          bounds.width - kResizeSz, 14.f};
    int prevColor = GuiGetStyle(LABEL, TEXT_COLOR_NORMAL);
    GuiSetStyle(LABEL, TEXT_COLOR_NORMAL, 0x00bb00ff);
    GuiLabel(hintRect, "M: Toggle");
    GuiSetStyle(LABEL, TEXT_COLOR_NORMAL, prevColor);
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

    m_examples.clear();
    m_cur_example = nullptr;

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

    BeginMode3D(m_camera);
    {
        float lightDir[3] = {0.5f, -1.0f, -0.5f};
        float lightColor[3] = {1.0f, 1.0f, 1.0f};
        float ambientColor[3] = {0.15f, 0.15f, 0.15f};
        float viewPos[3] = {m_camera.position.x, m_camera.position.y,
                            m_camera.position.z};
        float specStrength = 0.5f;
        float shininess = 32.0f;

        SetShaderValue(m_lighting_shader, m_loc_light_dir, lightDir,
                       SHADER_UNIFORM_VEC3);
        SetShaderValue(m_lighting_shader, m_loc_view_pos, viewPos,
                       SHADER_UNIFORM_VEC3);
        SetShaderValue(m_lighting_shader, m_loc_light_color, lightColor,
                       SHADER_UNIFORM_VEC3);
        SetShaderValue(m_lighting_shader, m_loc_ambient_color, ambientColor,
                       SHADER_UNIFORM_VEC3);
        SetShaderValue(m_lighting_shader, m_loc_specular_strength,
                       &specStrength, SHADER_UNIFORM_FLOAT);
        SetShaderValue(m_lighting_shader, m_loc_shininess, &shininess,
                       SHADER_UNIFORM_FLOAT);

        if (m_cur_example) {
            m_cur_example->OnRender3D(GetFrameTime());
        }

        DrawGrid(1000, 1.0f);
    }
    EndMode3D();

    // 2d mode
    if (m_cur_example) {
        m_cur_example->OnRender2D(GetFrameTime());
    }
    showHelpMsg();
    renderExampleMenu();

    EndDrawing();
}

void Context::DrawBox(Vector3 center, Vector3 rotation, Vector3 halfExtent,
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

void Context::DrawSphere(Vector3 center, Vector3 rotation, float radius,
                         Color color) const {
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

void Context::DrawCylinder(Vector3 center, Vector3 rotation, float height,
                           float radius, Color color) const {
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    rlScalef(radius * 2.0f, height, radius * 2.0f);
    DrawModel(m_model_cylinder, Vector3{0, 0, 0}, 1.0f, color);
    rlPopMatrix();
}

void Context::DrawCapsule(Vector3 center, Vector3 rotation, float height,
                          float radius, Color color) const {
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
    } else if (getCameraMode() == CAMERA_ORBITAL) {
        UpdateCamera(&m_camera, getCameraMode());
    }

    if (m_cur_example) {
        m_cur_example->OnUpdate(delta_time);
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