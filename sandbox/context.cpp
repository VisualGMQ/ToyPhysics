#include "context.hpp"

#include "raymath.h"
#include "rlgl.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>

#include "toy_physics/common/log.hpp"

constexpr float WINDOW_INIT_W = 1960;
constexpr float WINDOW_INIT_H = 1280;
const char* WINDOW_TITLE = "ToyPhysics Sandbox";

std::unique_ptr<Context> Context::instance;

Context& Context::GetInst() {
    return *instance;
}

Context::~Context() {}

void Context::Initialize() {
    m_should_exit = false;
    m_shape_factory = std::make_unique<toy_physics::ShapeFactory>();

    InitWindow(WINDOW_INIT_W, WINDOW_INIT_H, WINDOW_TITLE);
    SetWindowState(FLAG_WINDOW_RESIZABLE);
    SetTargetFPS(60);
    GuiLoadStyleDefault();
    GuiSetStyle(DEFAULT, TEXT_SIZE, 20);
    initTexture();
    initLightingShader();
    initFlatShader();
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
#ifdef TOY_PHYSICS_PLATFORM_WEB
    m_lighting_shader = LoadShader("sandbox/assets/vert_web.glsl",
                                   "sandbox/assets/frag_web.glsl");
#else
    m_lighting_shader =
        LoadShader("sandbox/assets/vert.glsl", "sandbox/assets/frag.glsl");
#endif

    m_loc_light_dir = GetShaderLocation(m_lighting_shader, "lightDir");
    m_loc_view_pos = GetShaderLocation(m_lighting_shader, "viewPos");
    m_loc_light_color = GetShaderLocation(m_lighting_shader, "lightColor");
    m_loc_ambient_color = GetShaderLocation(m_lighting_shader, "ambientColor");
    m_loc_specular_strength =
        GetShaderLocation(m_lighting_shader, "specularStrength");
    m_loc_shininess = GetShaderLocation(m_lighting_shader, "shininess");
}

void Context::initFlatShader() {
#ifdef TOY_PHYSICS_PLATFORM_WEB
    m_flat_shader = LoadShader("sandbox/assets/vert_flat_web.glsl",
                               "sandbox/assets/frag_flat_web.glsl");
#else
    m_flat_shader = LoadShader("sandbox/assets/vert_flat.glsl",
                               "sandbox/assets/frag_flat.glsl");
#endif

    m_loc_flat_mvp = GetShaderLocation(m_flat_shader, "mvp");
    m_loc_flat_color = GetShaderLocation(m_flat_shader, "uColor");
}

void Context::loadModels() {
    Mesh box_mesh = GenMeshCube(1.0f, 1.0f, 1.0f);
    m_model_box = LoadModelFromMesh(box_mesh);

    m_sphere_mesh = GenMeshSphere(0.5f, 64, 64);
    m_model_sphere = LoadModelFromMesh(m_sphere_mesh);

    m_cylinder_mesh = GenMeshCylinder(0.5f, 1.0f, 64);
    for (int i = 0; i < m_cylinder_mesh.vertexCount; i++) {
        m_cylinder_mesh.vertices[i * 3 + 1] -= 0.5f;
    }
    UpdateMeshBuffer(m_cylinder_mesh, 0, m_cylinder_mesh.vertices,
                     m_cylinder_mesh.vertexCount * 3 * sizeof(float), 0);
    m_model_cylinder = LoadModelFromMesh(m_cylinder_mesh);

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
    constexpr size_t font_size = 26;
    DrawText(example_hint_str.c_str(), 20, 24, font_size, ORANGE);
    DrawText("Press ESC to quit", 20, 60, font_size, BLACK);
    const std::string camera_mode_text =
        "Tab to switch camera mode: " + camera_mode_name;
    DrawText(camera_mode_text.c_str(), 20, 96, font_size, BLACK);
    DrawText("Press L-Alt to toggle mouse", 20, 132, font_size, BLACK);
}

void Context::renderExampleMenu() {
    static constexpr float kHeaderHeight = 32.f;
    static constexpr float kBtnHeight = 38.f;
    static constexpr float kBtnPad = 3.f;
    static constexpr float kFooterH = kBtnHeight + kBtnPad * 2;
    static constexpr float kMinW = 200.f;
    static constexpr float kMinH = kHeaderHeight + kFooterH + kBtnHeight;
    static constexpr float kResizeSz = 20.f;

    static bool visible = true;
    static Rectangle bounds = {10.f, 200.f, 220.f, 300.f};
    static bool drag = false;
    static bool sizing = false;
    static Vector2 dragStart = {};
    static Rectangle dragStartBounds = {};
    static Vector2 scroll = {};
    static std::map<std::string, bool> s_expanded;

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

    if (GuiWindowBox(bounds, "Examples")) {
        visible = false;
        return;
    }

    // collect categories from the registered example names ("Category/Name")
    std::vector<std::string> categories;
    std::vector<int> counts;
    for (const auto& example : m_examples) {
        const std::string& name = example->GetName();
        size_t slash = name.find('/');
        if (slash == std::string::npos) continue;
        std::string category = name.substr(0, slash);
        auto it = std::find(categories.begin(), categories.end(), category);
        if (it == categories.end()) {
            categories.push_back(category);
            counts.push_back(1);
            s_expanded.try_emplace(category, true);
        } else {
            ++counts[static_cast<size_t>(it - categories.begin())];
        }
    }

    float scrollBarW = (float)GuiGetStyle(SCROLLBAR, ARROWS_SIZE) + 8.f;
    float btnW = bounds.width - scrollBarW - kBtnPad * 2;
    float footerTop = bounds.y + bounds.height - kFooterH;

    int totalRows = 0;
    for (size_t c = 0; c < categories.size(); ++c) {
        totalRows += 1 + (s_expanded[categories[c]] ? counts[c] : 0);
    }

    Rectangle contentArea = {bounds.x, bounds.y + kHeaderHeight, bounds.width,
                             bounds.height - kHeaderHeight - kFooterH};
    float contentH = totalRows * (kBtnHeight + kBtnPad);
    Rectangle content = {0, 0, btnW + kBtnPad,
                         std::max(contentH, contentArea.height)};
    Rectangle view = {};

    GuiScrollPanel(contentArea, nullptr, content, &scroll, &view);

    BeginScissorMode((int)view.x, (int)view.y, (int)view.width,
                     (int)view.height);

    float bx = bounds.x + kBtnPad + scroll.x;
    float by = bounds.y + kHeaderHeight + kBtnPad + scroll.y;
    int row = 0;
    for (size_t c = 0; c < categories.size(); ++c) {
        const std::string& category = categories[c];
        bool expanded = s_expanded[category];

        // category header: click to expand/collapse this group only
        Rectangle hdr = {bx, by + row * (kBtnHeight + kBtnPad), btnW,
                         kBtnHeight};
        char hdrText[64];
        snprintf(hdrText, sizeof(hdrText), "%s %s",
                 expanded ? "#120#" : "#115#", category.c_str());
        int prevAlign = GuiGetStyle(BUTTON, TEXT_ALIGNMENT);
        GuiSetStyle(BUTTON, TEXT_ALIGNMENT, TEXT_ALIGN_LEFT);
        if (GuiButton(hdr, hdrText)) {
            s_expanded[category] = !expanded;
        }
        GuiSetStyle(BUTTON, TEXT_ALIGNMENT, prevAlign);
        ++row;

        if (!expanded) continue;
        for (const auto& example : m_examples) {
            const std::string& name = example->GetName();
            if (name.rfind(category + "/", 0) != 0) continue;
            std::string label = name.substr(category.size() + 1);
            Rectangle btn = {bx, by + row * (kBtnHeight + kBtnPad), btnW,
                             kBtnHeight};
            bool active = (example.get() == m_cur_example);
            if (active) GuiSetState(STATE_PRESSED);
            if (GuiButton(btn, label.c_str())) {
                m_cur_example = example.get();
            }
            if (active) GuiSetState(STATE_NORMAL);
            ++row;
        }
    }

    EndScissorMode();

    // uncategorized examples (e.g. RenderTest) stay outside the category
    // list as an always-visible button
    for (const auto& example : m_examples) {
        const std::string& name = example->GetName();
        if (name.find('/') != std::string::npos) continue;
        Rectangle btn = {bounds.x + kBtnPad, footerTop + kBtnPad, btnW,
                         kBtnHeight};
        bool active = (example.get() == m_cur_example);
        if (active) GuiSetState(STATE_PRESSED);
        if (GuiButton(btn, name.c_str())) {
            m_cur_example = example.get();
        }
        if (active) GuiSetState(STATE_NORMAL);
    }

    GuiLabel({bounds.x + bounds.width - 16.f, bounds.y + bounds.height - 16.f,
              16.f, 16.f},
             "#033#");
    Rectangle hintRect = {bounds.x + 4.f, bounds.y + bounds.height - 18.f,
                          bounds.width - kResizeSz, 18.f};
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
    m_shapes.clear();
    m_shape_factory.reset();

    UnloadModel(m_model_box);
    UnloadModel(m_model_sphere);
    UnloadModel(m_model_cylinder);
    UnloadTexture(m_color_texture);
    UnloadShader(m_lighting_shader);
    UnloadShader(m_flat_shader);
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
    DrawModel(m_model_sphere, Vector3{0, 0, 0}, 1.0f, color);
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
    DrawModel(m_model_sphere, Vector3{0, 0, 0}, 1.0f, color);
    rlPopMatrix();
}

void Context::enableFlatShader(Color color) const {
    BeginShaderMode(m_flat_shader);
    rlEnableColorBlend();
    rlSetBlendMode(RL_BLEND_ALPHA);
    float color4[4] = {color.r / 255.f, color.g / 255.f, color.b / 255.f,
                       color.a / 255.f};
    SetShaderValue(m_flat_shader, m_loc_flat_color, color4,
                   SHADER_UNIFORM_VEC4);
}

void Context::setFlatMvp() const {
    Matrix mvp =
        MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    SetShaderValueMatrix(m_flat_shader, m_loc_flat_mvp, mvp);
}

void Context::DrawAABBFlat(const toy_physics::AABB& aabb, Color color) const {
    Vector3 center = {(aabb.m_min.x() + aabb.m_max.x()) * 0.5f,
                      (aabb.m_min.y() + aabb.m_max.y()) * 0.5f,
                      (aabb.m_min.z() + aabb.m_max.z()) * 0.5f};
    Vector3 size = {aabb.m_max.x() - aabb.m_min.x(),
                    aabb.m_max.y() - aabb.m_min.y(),
                    aabb.m_max.z() - aabb.m_min.z()};

    enableFlatShader(color);
    setFlatMvp();
    rlDisableDepthMask();
    DrawCubeV(center, size, WHITE);
    rlEnableDepthMask();
    EndShaderMode();
}

void Context::DrawAABBFlatWires(const toy_physics::AABB& aabb,
                                Color color) const {
    Vector3 center = {(aabb.m_min.x() + aabb.m_max.x()) * 0.5f,
                      (aabb.m_min.y() + aabb.m_max.y()) * 0.5f,
                      (aabb.m_min.z() + aabb.m_max.z()) * 0.5f};
    Vector3 size = {aabb.m_max.x() - aabb.m_min.x(),
                    aabb.m_max.y() - aabb.m_min.y(),
                    aabb.m_max.z() - aabb.m_min.z()};

    enableFlatShader(color);
    setFlatMvp();
    DrawCubeWiresV(center, size, WHITE);
    EndShaderMode();
}

void Context::DrawBoxFlat(Vector3 center, Vector3 rotation, Vector3 halfExtent,
                          Color color) const {
    enableFlatShader(color);
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    setFlatMvp();
    DrawCubeV(
        Vector3{0, 0, 0},
        Vector3{halfExtent.x * 2.0f, halfExtent.y * 2.0f, halfExtent.z * 2.0f},
        WHITE);
    rlPopMatrix();
    EndShaderMode();
}

void Context::DrawSphereFlat(Vector3 center, Vector3 rotation, float radius,
                             Color color) const {
    enableFlatShader(color);
    setFlatMvp();
    ::DrawSphere(center, radius, WHITE);
    EndShaderMode();
}

void Context::DrawCylinderFlat(Vector3 center, Vector3 rotation, float height,
                               float radius, Color color) const {
    enableFlatShader(color);
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    setFlatMvp();
    // raylib's DrawCylinder spans [0, height] from its position, so shift
    // the base down by half the height to keep it centered
    ::DrawCylinder(Vector3{0, -height * 0.5f, 0}, radius, radius, height, 24,
                   WHITE);
    rlPopMatrix();
    EndShaderMode();
}

void Context::DrawCapsuleFlat(Vector3 center, Vector3 rotation, float height,
                              float radius, Color color) const {
    enableFlatShader(color);
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    rlRotatef(rotation.x, 1.0f, 0.0f, 0.0f);
    rlRotatef(rotation.y, 0.0f, 1.0f, 0.0f);
    rlRotatef(rotation.z, 0.0f, 0.0f, 1.0f);
    setFlatMvp();
    // mirror the lit capsule construction: centered cylinder section plus
    // one sphere at each end
    ::DrawCylinder(Vector3{0, -height * 0.5f, 0}, radius, radius, height, 24,
                   WHITE);
    ::DrawSphere(Vector3{0, -height * 0.5f, 0}, radius, WHITE);
    ::DrawSphere(Vector3{0, height * 0.5f, 0}, radius, WHITE);
    rlPopMatrix();
    EndShaderMode();
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

toy_physics::Shape* Context::CreateShape(
    const toy_physics::BoxGeometry& geometry) {
    return m_shape_factory->Create(geometry);
}

toy_physics::Shape* Context::CreateShape(
    const toy_physics::SphereGeometry& geometry) {
    return m_shape_factory->Create(geometry);
}

toy_physics::Shape* Context::CreateShape(
    const toy_physics::CapsuleGeometry& geometry) {
    return m_shape_factory->Create(geometry);
}

toy_physics::Shape* Context::CreateShape(
    const toy_physics::CylinderGeometry& geometry) {
    return m_shape_factory->Create(geometry);
}

toy_physics::Shape* Context::CreateShape(
    const toy_physics::ConvexHullGeometry& geometry) {
    return m_shape_factory->Create(geometry);
}

std::vector<std::unique_ptr<toy_physics::Shape>>& Context::GetShapes() {
    return m_shapes;
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