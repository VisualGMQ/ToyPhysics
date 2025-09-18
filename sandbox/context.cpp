#include "context.hpp"

#include <fstream>

#include "sdl_call.hpp"
#include "toy_physics/log.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

constexpr float WINDOW_INIT_W = 1024;
constexpr float WINDOW_INIT_H = 720;

std::unique_ptr<Context> Context::instance;

Context& Context::GetInst() {
    return *instance;
}

Context::~Context() {}

void Context::InitSystem() {
    LOGT("system init");
    SDL_CALL(SDL_Init(SDL_INIT_EVENTS | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK |
                      SDL_INIT_GAMEPAD));
}

void Context::ShutdownSystem() {
    SDL_Quit();
    LOGT("system shutdown");
}

void Context::Initialize() {
    m_should_exit = false;

    m_gpu_device =
        SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV, true, nullptr);
    if (!m_gpu_device) {
        LOGE("create gpu device failed");
    }

    m_window = SDL_CreateWindow("ToyPhysics Sandbox", WINDOW_INIT_W,
                                WINDOW_INIT_H, SDL_WINDOW_RESIZABLE);
    if (!m_window) {
        LOGE("create window failed");
    }

    if (!SDL_ClaimWindowForGPUDevice(m_gpu_device, m_window)) {
        LOGE("window not support SDL GPU");
    }

    m_vertex_shader =
        loadSDLGPUShader("sandbox/vert.spv", SDL_GPU_SHADERSTAGE_VERTEX, 0, 1);
    m_fragment_shader = loadSDLGPUShader("sandbox/frag.spv",
                                         SDL_GPU_SHADERSTAGE_FRAGMENT, 1, 0);
    m_gpu_sampler = createSampler();
    m_gpu_depth_texture = createDepthTexture(WINDOW_INIT_W, WINDOW_INIT_H);
    m_graphics_pipeline = createGraphicsPipeline();

    uint32_t white_color = 0xFFFFFFFF;
    m_gpu_white_texture = createImageTexture(&white_color, 1, 1);
    std::array<uint32_t, 4> colorful = {
        0xFF3232AC,
        0xFF30BE6A,
        0xFFE16E5B,
        0xFF525659,
    };
    m_gpu_colorful_texture = createImageTexture(colorful.data(), 2, 2);

    m_camera = std::make_unique<FlyCamera>(
        Radians{Degrees{30.0f}}, WINDOW_INIT_W / WINDOW_INIT_H, 0.01f, 1000.0f);

    m_cube_mesh = loadModel("sandbox/cube.obj");
    m_sphere_mesh = loadModel("sandbox/sphere.obj");
    m_semi_sphere_mesh = loadModel("sandbox/semi-sphere.obj");
    m_cylinder_mesh = loadModel("sandbox/cylinder.obj");
    SDL_CALL(SDL_SetWindowRelativeMouseMode(m_window, true));
}

void Context::Shutdown() {
    m_should_exit = true;

    SDL_WaitForGPUIdle(m_gpu_device);
    SDL_ReleaseGPUBuffer(m_gpu_device, m_cube_mesh.m_buffer);
    SDL_ReleaseGPUBuffer(m_gpu_device, m_sphere_mesh.m_buffer);
    SDL_ReleaseGPUBuffer(m_gpu_device, m_semi_sphere_mesh.m_buffer);
    SDL_ReleaseGPUBuffer(m_gpu_device, m_cylinder_mesh.m_buffer);
    SDL_ReleaseGPUGraphicsPipeline(m_gpu_device, m_graphics_pipeline);
    SDL_ReleaseGPUSampler(m_gpu_device, m_gpu_sampler);
    SDL_ReleaseGPUTexture(m_gpu_device, m_gpu_depth_texture);
    SDL_ReleaseGPUTexture(m_gpu_device, m_gpu_white_texture);
    SDL_ReleaseGPUTexture(m_gpu_device, m_gpu_colorful_texture);
    SDL_ReleaseGPUShader(m_gpu_device, m_vertex_shader);
    SDL_ReleaseGPUShader(m_gpu_device, m_fragment_shader);
    SDL_DestroyGPUDevice(m_gpu_device);
    SDL_DestroyWindow(m_window);
}

SDL_GPUShader* Context::loadSDLGPUShader(const char* filename,
                                         SDL_GPUShaderStage stage,
                                         uint32_t sampler_num,
                                         uint32_t uniform_buffer_num) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        LOGE("shader file {} load failed", filename);
        return nullptr;
    }

    std::vector<char> content{std::istreambuf_iterator<char>(file),
                              std::istreambuf_iterator<char>()};

    SDL_GPUShaderCreateInfo ci;
    ci.code = (Uint8*)content.data();
    ci.code_size = content.size();
    ci.entrypoint = "main";
    ci.format = SDL_GPU_SHADERFORMAT_SPIRV;
    ci.num_samplers = sampler_num;
    ci.num_uniform_buffers = uniform_buffer_num;
    ci.num_storage_buffers = 0;
    ci.num_storage_textures = 0;
    ci.stage = stage;

    SDL_GPUShader* shader = SDL_CreateGPUShader(m_gpu_device, &ci);
    if (!shader) {
        SDL_LogError(SDL_LOG_CATEGORY_GPU,
                     "create gpu shader from %s failed: %s", filename,
                     SDL_GetError());
        return {};
    }
    return shader;
}

SDL_GPUGraphicsPipeline* Context::createGraphicsPipeline() {
    SDL_GPUGraphicsPipelineCreateInfo ci{};

    SDL_GPUVertexAttribute attributes[3];

    // position attribute
    {
        attributes[0].location = 0;
        attributes[0].buffer_slot = 0;
        attributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
        attributes[0].offset = 0;
    }

    // uv attribute
    {
        attributes[1].location = 1;
        attributes[1].buffer_slot = 0;
        attributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
        attributes[1].offset = sizeof(float) * 3;
    }

    // normal attribute
    {
        attributes[2].location = 2;
        attributes[2].buffer_slot = 0;
        attributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
        attributes[2].offset = sizeof(float) * 5;
    }

    ci.vertex_input_state.vertex_attributes = attributes;
    ci.vertex_input_state.num_vertex_attributes = std::size(attributes);

    SDL_GPUVertexBufferDescription buffer_desc;
    buffer_desc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    buffer_desc.instance_step_rate = 1;
    buffer_desc.slot = 0;
    buffer_desc.pitch = sizeof(float) * 8;

    ci.vertex_input_state.num_vertex_buffers = 1;
    ci.vertex_input_state.vertex_buffer_descriptions = &buffer_desc;

    ci.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    ci.vertex_shader = m_vertex_shader;
    ci.fragment_shader = m_fragment_shader;

    ci.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_BACK;
    ci.rasterizer_state.front_face = SDL_GPU_FRONTFACE_CLOCKWISE;
    ci.rasterizer_state.fill_mode = SDL_GPU_FILLMODE_FILL;

    ci.multisample_state.enable_mask = false;
    ci.multisample_state.sample_count = SDL_GPU_SAMPLECOUNT_1;

    ci.target_info.num_color_targets = 1;
    ci.target_info.has_depth_stencil_target = true;
    ci.target_info.depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D16_UNORM;

    // depth stencil state
    SDL_GPUDepthStencilState state{};
    state.back_stencil_state.compare_op = SDL_GPU_COMPAREOP_NEVER;
    state.back_stencil_state.pass_op = SDL_GPU_STENCILOP_ZERO;
    state.back_stencil_state.fail_op = SDL_GPU_STENCILOP_ZERO;
    state.back_stencil_state.depth_fail_op = SDL_GPU_STENCILOP_ZERO;
    state.compare_op = SDL_GPU_COMPAREOP_LESS;
    state.enable_depth_test = true;
    state.enable_depth_write = true;
    state.enable_stencil_test = false;
    state.compare_mask = 0xFF;
    state.write_mask = 0xFF;
    ci.depth_stencil_state = state;

    SDL_GPUColorTargetDescription desc;
    desc.blend_state.alpha_blend_op = SDL_GPU_BLENDOP_ADD;
    desc.blend_state.color_blend_op = SDL_GPU_BLENDOP_ADD;
    desc.blend_state.color_write_mask =
        SDL_GPU_COLORCOMPONENT_A | SDL_GPU_COLORCOMPONENT_R |
        SDL_GPU_COLORCOMPONENT_G | SDL_GPU_COLORCOMPONENT_B;
    desc.blend_state.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
    desc.blend_state.src_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
    desc.blend_state.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
    desc.blend_state.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ZERO;
    desc.blend_state.enable_blend = true;
    desc.blend_state.enable_color_write_mask = false;
    desc.format = SDL_GetGPUSwapchainTextureFormat(m_gpu_device, m_window);

    ci.target_info.color_target_descriptions = &desc;

    return SDL_CreateGPUGraphicsPipeline(m_gpu_device, &ci);
}

SDL_GPUTexture* Context::createDepthTexture(int w, int h) {
    SDL_GPUTextureCreateInfo texture_ci;
    texture_ci.format = SDL_GPU_TEXTUREFORMAT_D16_UNORM;
    texture_ci.height = h;
    texture_ci.width = w;
    texture_ci.layer_count_or_depth = 1;
    texture_ci.num_levels = 1;
    texture_ci.sample_count = SDL_GPU_SAMPLECOUNT_1;
    texture_ci.type = SDL_GPU_TEXTURETYPE_2D;
    texture_ci.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

    return SDL_CreateGPUTexture(m_gpu_device, &texture_ci);
}

SDL_GPUTexture* Context::createImageTexture(uint32_t* data, int w, int h) {
    size_t image_size = 4 * w * h;
    SDL_GPUTransferBufferCreateInfo transfer_buffer_ci;
    transfer_buffer_ci.size = image_size;
    transfer_buffer_ci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;

    SDL_GPUTransferBuffer* transfer_buffer =
        SDL_CreateGPUTransferBuffer(m_gpu_device, &transfer_buffer_ci);
    void* ptr = SDL_MapGPUTransferBuffer(m_gpu_device, transfer_buffer, false);
    memcpy(ptr, data, image_size);
    SDL_UnmapGPUTransferBuffer(m_gpu_device, transfer_buffer);

    SDL_GPUTextureCreateInfo texture_ci;
    texture_ci.format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
    texture_ci.height = h;
    texture_ci.width = w;
    texture_ci.layer_count_or_depth = 1;
    texture_ci.num_levels = 1;
    texture_ci.sample_count = SDL_GPU_SAMPLECOUNT_1;
    texture_ci.type = SDL_GPU_TEXTURETYPE_2D;
    texture_ci.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;

    SDL_GPUTexture* texture = SDL_CreateGPUTexture(m_gpu_device, &texture_ci);

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_gpu_device);
    SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);

    SDL_GPUTextureTransferInfo transfer_info;
    transfer_info.offset = 0;
    transfer_info.pixels_per_row = w;
    transfer_info.rows_per_layer = h;
    transfer_info.transfer_buffer = transfer_buffer;

    SDL_GPUTextureRegion region;
    region.w = w;
    region.h = h;
    region.x = 0;
    region.y = 0;
    region.layer = 0;
    region.mip_level = 0;
    region.z = 0;
    region.d = 1;
    region.texture = texture;

    SDL_UploadToGPUTexture(copy_pass, &transfer_info, &region, false);
    SDL_EndGPUCopyPass(copy_pass);

    SDL_SubmitGPUCommandBuffer(cmd);

    SDL_ReleaseGPUTransferBuffer(m_gpu_device, transfer_buffer);
    return texture;
}

SDL_GPUSampler* Context::createSampler() {
    SDL_GPUSamplerCreateInfo ci;
    ci.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    ci.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    ci.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
    ci.enable_anisotropy = false;
    ci.compare_op = SDL_GPU_COMPAREOP_ALWAYS;
    ci.enable_compare = false;
    ci.mag_filter = SDL_GPU_FILTER_NEAREST;
    ci.min_filter = SDL_GPU_FILTER_NEAREST;
    ci.max_lod = 1.0;
    ci.min_lod = 1.0;
    ci.mip_lod_bias = 0.0;
    ci.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;

    return SDL_CreateGPUSampler(m_gpu_device, &ci);
}

GPUMesh Context::loadModel(const std::string& filename) {
    CPUMesh model;
    model.Load(filename);

    if (!model) {
        return {};
    }

    SDL_GPUBuffer* buffer = createAndUploadVertexData(model);
    return {buffer, model.m_vertices.size()};
}

SDL_GPUBuffer* Context::createAndUploadVertexData(const CPUMesh& model) {
    SDL_GPUTransferBufferCreateInfo transfer_buffer_ci;
    transfer_buffer_ci.size = sizeof(Vertex) * model.m_vertices.size();
    transfer_buffer_ci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;

    SDL_GPUTransferBuffer* transfer_buffer =
        SDL_CreateGPUTransferBuffer(m_gpu_device, &transfer_buffer_ci);
    void* ptr = SDL_MapGPUTransferBuffer(m_gpu_device, transfer_buffer, false);
    memcpy(ptr, model.m_vertices.data(),
           sizeof(Vertex) * model.m_vertices.size());
    SDL_UnmapGPUTransferBuffer(m_gpu_device, transfer_buffer);

    SDL_GPUBufferCreateInfo gpu_buffer_ci;
    gpu_buffer_ci.size = sizeof(Vertex) * model.m_vertices.size();
    gpu_buffer_ci.usage = SDL_GPU_BUFFERUSAGE_VERTEX;

    SDL_GPUBuffer* buffer = SDL_CreateGPUBuffer(m_gpu_device, &gpu_buffer_ci);

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_gpu_device);
    SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);
    SDL_GPUTransferBufferLocation location;
    location.offset = 0;
    location.transfer_buffer = transfer_buffer;
    SDL_GPUBufferRegion region;
    region.buffer = buffer;
    region.offset = 0;
    region.size = sizeof(Vertex) * model.m_vertices.size();
    SDL_UploadToGPUBuffer(copy_pass, &location, &region, false);
    SDL_EndGPUCopyPass(copy_pass);

    SDL_SubmitGPUCommandBuffer(cmd);

    SDL_ReleaseGPUTransferBuffer(m_gpu_device, transfer_buffer);

    return buffer;
}

void Context::renderUpdate() {
    bool is_minimized = SDL_GetWindowFlags(m_window) & SDL_WINDOW_MINIMIZED;
    if (is_minimized) {
        return;
    }

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_gpu_device);
    SDL_GPUTexture* swapchain_texture = nullptr;
    Uint32 width, height;

    if (!SDL_AcquireGPUSwapchainTexture(cmd, m_window, &swapchain_texture,
                                        &width, &height)) {
        LOGE("SDL swapchain texture acquire failed! {}", SDL_GetError());
    }

    if (!swapchain_texture) {
        return;
    }

    SDL_GPUColorTargetInfo color_target_info{};
    color_target_info.clear_color.r = 0.1;
    color_target_info.clear_color.g = 0.1;
    color_target_info.clear_color.b = 0.1;
    color_target_info.clear_color.a = 1;
    color_target_info.load_op = SDL_GPU_LOADOP_CLEAR;
    color_target_info.mip_level = 0;
    color_target_info.store_op = SDL_GPU_STOREOP_STORE;
    color_target_info.texture = swapchain_texture;
    color_target_info.cycle = true;
    color_target_info.layer_or_depth_plane = 0;
    color_target_info.cycle_resolve_texture = false;

    SDL_GPUDepthStencilTargetInfo depth_target_info{};
    depth_target_info.clear_depth = 1;
    depth_target_info.cycle = false;
    depth_target_info.load_op = SDL_GPU_LOADOP_CLEAR;
    depth_target_info.store_op = SDL_GPU_STOREOP_DONT_CARE;
    depth_target_info.texture = m_gpu_depth_texture;

    SDL_GPURenderPass* render_pass =
        SDL_BeginGPURenderPass(cmd, &color_target_info, 1, &depth_target_info);
    SDL_BindGPUGraphicsPipeline(render_pass, m_graphics_pipeline);

    SDL_GPUTextureSamplerBinding sampler_binding;
    sampler_binding.texture = m_gpu_colorful_texture;
    sampler_binding.sampler = m_gpu_sampler;
    SDL_BindGPUFragmentSamplers(render_pass, 0, &sampler_binding, 1);

    int window_width, window_height;
    SDL_GetWindowSize(m_window, &window_width, &window_height);
    SDL_GPUViewport viewport;
    viewport.x = 0;
    viewport.y = 0;
    viewport.w = window_width;
    viewport.h = window_height;
    viewport.min_depth = 0;
    viewport.max_depth = 1;
    SDL_SetGPUViewport(render_pass, &viewport);

    for (auto& model : m_models) {
        SDL_GPUBufferBinding binding;
        binding.buffer = model.m_mesh->m_buffer;
        binding.offset = 0;
        SDL_BindGPUVertexBuffers(render_pass, 0, &binding, 1);

        MVP mvp;
        mvp.proj = m_camera->GetProject();
        mvp.view = m_camera->GetView();
        mvp.model = model.m_transform.ToMatrix();
        SDL_PushGPUVertexUniformData(cmd, 0, &mvp, sizeof(MVP));

        SDL_DrawGPUPrimitives(render_pass, model.m_mesh->m_vertex_count, 1, 0,
                              0);
    }

    SDL_EndGPURenderPass(render_pass);

    if (!SDL_SubmitGPUCommandBuffer(cmd)) {
        LOGE("SDL submit command buffer failed! {}", SDL_GetError());
    }

    m_models.clear();
}

void Context::logicUpdate(float delta_time) {
    static Radians rotate_x;
    static Radians rotate_y;

    rotate_x += 0.001;
    rotate_y += 0.001;

    Transform transform;
    transform.m_position = {0, 0, -5};
    transform.m_rotation =
        Eigen::Quaternionf{
            Eigen::AngleAxisf{rotate_x.Value(), Eigen::Vector3f::UnitX()}
    } *
        Eigen::Quaternionf{
            Eigen::AngleAxisf{rotate_y.Value(), Eigen::Vector3f::UnitY()}};
    drawCube(transform);

    transform.m_position.x() = -2;
    drawSphere(transform);

    transform.m_position.x() = 2;
    drawCylinder(transform);

    transform.m_position.x() = -4;
    drawCapsule(transform);

    if (dynamic_cast<FlyCamera*>(m_camera.get())) {
        handleFlyCamera(delta_time);
    } else {
        // handleOrbitCamera(delta_time);
    }
}

void Context::drawCube(const Transform& transform) {
    drawModel(m_cube_mesh, transform);
}

void Context::drawSphere(const Transform& transform) {
    drawModel(m_sphere_mesh, transform);
}

void Context::drawSemiSphere(const Transform& transform) {
    drawModel(m_semi_sphere_mesh, transform);
}

void Context::drawCylinder(const Transform& transform) {
    drawModel(m_cylinder_mesh, transform);
}

void Context::drawCapsule(const Transform& transform) {
    drawModel(m_cylinder_mesh, transform);

    Eigen::Vector3f rotated_axis =
        transform.m_rotation * Eigen::Vector3f::UnitY();

    {
        Transform trans = transform;
        trans.m_position += rotated_axis * transform.m_scale.y() * 0.5;
        drawModel(m_semi_sphere_mesh, trans);
    }

    {
        Transform trans = transform;
        trans.m_position -= rotated_axis * transform.m_scale.y() * 0.5;
        Eigen::Vector3f rotated_x_axis =
            transform.m_rotation * Eigen::Vector3f::UnitX();
        trans.m_rotation =
            Eigen::Quaternionf{
                Eigen::AngleAxisf{EIGEN_PI, rotated_x_axis}
        } *
            trans.m_rotation;
        drawModel(m_semi_sphere_mesh, trans);
    }
}

void Context::drawModel(const GPUMesh& mesh, const Transform& transform) {
    Model model;
    model.m_mesh = &mesh;
    model.m_transform = transform;
    m_models.push_back(model);
}

void Context::handleFlyCamera(float delta_time) {
    FlyCamera* camera = static_cast<FlyCamera*>(m_camera.get());
    const bool* key_states = SDL_GetKeyboardState(nullptr);
    float dist = m_camera_move_speed * delta_time;
    if (key_states[SDL_SCANCODE_A]) {
        camera->MoveRightLeft(-dist);
    }
    if (key_states[SDL_SCANCODE_D]) {
        camera->MoveRightLeft(dist);
    }
    if (key_states[SDL_SCANCODE_W]) {
        camera->MoveForward(dist);
    }
    if (key_states[SDL_SCANCODE_E]) {
        camera->MoveUpDown(dist);
    }
    if (key_states[SDL_SCANCODE_Q]) {
        camera->MoveUpDown(-dist);
    }
    if (key_states[SDL_SCANCODE_S]) {
        camera->MoveForward(-dist);
    }

    constexpr float rotate_speed = 0.001;

    float x, y;
    SDL_GetRelativeMouseState(&x, &y);
    if (m_is_mouse_relative_mode) {
        camera->AddYaw(-x * rotate_speed * delta_time);
        camera->AddPitch(y * rotate_speed * delta_time);
    }
}

void Context::Update() {
    static Uint64 cur_time = 0;

    Uint64 time = SDL_GetTicksNS();
    Uint64 elapse_time = time - cur_time;
    cur_time = time;

    float nanoseconds = std::max(elapse_time / 1000000.0f, 0.000001f);

    logicUpdate(nanoseconds);
    renderUpdate();
}

void Context::HandleEvents(const SDL_Event& event) {
    if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
        auto window_id = SDL_GetWindowID(m_window);
        if (event.window.windowID == window_id) {
            m_should_exit = true;
        }
    }

    if (event.type == SDL_EVENT_WINDOW_RESIZED) {
        SDL_ReleaseGPUTexture(m_gpu_device, m_gpu_depth_texture);
        m_gpu_depth_texture =
            createDepthTexture(event.window.data1, event.window.data2);

        if (m_camera) {
            int w, h;
            SDL_GetWindowSize(m_window, &w, &h);
            auto new_frustum = ResizeFrustumInNewWindowSize(
                {w, h}, {event.window.data1, event.window.data2},
                m_camera->GetFrustum());
            m_camera->SetProject(new_frustum.m_fov, new_frustum.m_aspect,
                                 new_frustum.m_near, new_frustum.m_far);
        }
    }

    if (event.type == SDL_EVENT_KEY_DOWN) {
        if (event.key.key == SDLK_LALT) {
            m_is_mouse_relative_mode = !m_is_mouse_relative_mode;
            SDL_SetWindowRelativeMouseMode(m_window, m_is_mouse_relative_mode);
        }
    }

    if (event.type == SDL_EVENT_MOUSE_WHEEL) {
        m_camera_move_speed += event.wheel.y * 0.01;
        m_camera_move_speed = std::max(m_camera_move_speed, 0.0001f);
    }
}

bool Context::ShouldExit() const {
    return m_should_exit;
}

void Context::Exit() {
    m_should_exit = true;
}

void CPUMesh::Load(const std::string& filename) {
    m_vertices.clear();

    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./";

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(filename, reader_config)) {
        if (!reader.Error().empty()) {
            LOGE("TinyObjReader: {}", reader.Error());
        }
        return;
    }

    if (!reader.Warning().empty()) {
        LOGW("TinyObjReader: {}", reader.Warning());
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                Vertex vertex;

                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                vertex.m_position.x() =
                    attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                vertex.m_position.y() =
                    attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                vertex.m_position.z() =
                    attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                // normal data
                if (idx.normal_index >= 0) {
                    vertex.m_normal.x() =
                        attrib.normals[3 * size_t(idx.normal_index) + 0];
                    vertex.m_normal.y() =
                        attrib.normals[3 * size_t(idx.normal_index) + 1];
                    vertex.m_normal.z() =
                        attrib.normals[3 * size_t(idx.normal_index) + 2];
                }

                // texcoord data
                if (idx.texcoord_index >= 0) {
                    vertex.m_uv.x() =
                        attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
                    vertex.m_uv.y() =
                        attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];
                }

                m_vertices.push_back(vertex);
            }
            index_offset += fv;
        }
    }
}

Eigen::Matrix4f Transform::ToMatrix() const {
    return CreateTranslation(m_position) * CreateRotation(m_rotation) *
           CreateScale(m_scale);
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
