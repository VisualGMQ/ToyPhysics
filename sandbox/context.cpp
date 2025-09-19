#include "context.hpp"

#include <fstream>

#include "sdl_call.hpp"
#include "toy_physics/log.hpp"

std::unique_ptr<Context> Context::instance;

Context& Context::GetInst() {
    return *instance;
}

Context::~Context() {
}

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

    m_gpu_device = SDL_CreateGPUDevice(
        SDL_GPU_SHADERFORMAT_SPIRV, true, nullptr);
    if (!m_gpu_device) {
        LOGE("create gpu device failed");
    }

    m_window = SDL_CreateWindow("ToyPhysics Sandbox", 1024, 720,
                                SDL_WINDOW_RESIZABLE);
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
    createSampler();
    createDepthTexture(1024, 720);
    createGraphicsPipeline();
}

void Context::Shutdown() {
    m_should_exit = true;

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


void Context::createGraphicsPipeline() {
    SDL_GPUGraphicsPipelineCreateInfo ci{};

    SDL_GPUVertexAttribute attributes[2];

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

    ci.vertex_input_state.vertex_attributes = attributes;
    ci.vertex_input_state.num_vertex_attributes = std::size(attributes);

    SDL_GPUVertexBufferDescription buffer_desc;
    buffer_desc.input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    buffer_desc.instance_step_rate = 1;
    buffer_desc.slot = 0;
    buffer_desc.pitch = sizeof(float) * 5;

    ci.vertex_input_state.num_vertex_buffers = 1;
    ci.vertex_input_state.vertex_buffer_descriptions = &buffer_desc;

    ci.primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    ci.vertex_shader = m_vertex_shader;
    ci.fragment_shader = m_fragment_shader;

    ci.rasterizer_state.cull_mode = SDL_GPU_CULLMODE_NONE;
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

    m_graphics_pipeline = SDL_CreateGPUGraphicsPipeline(m_gpu_device, &ci);
}

void Context::createDepthTexture(int w, int h) {
    SDL_GPUTextureCreateInfo texture_ci;
    texture_ci.format = SDL_GPU_TEXTUREFORMAT_D16_UNORM;
    texture_ci.height = h;
    texture_ci.width = w;
    texture_ci.layer_count_or_depth = 1;
    texture_ci.num_levels = 1;
    texture_ci.sample_count = SDL_GPU_SAMPLECOUNT_1;
    texture_ci.type = SDL_GPU_TEXTURETYPE_2D;
    texture_ci.usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;

    m_gpu_depth_texture = SDL_CreateGPUTexture(m_gpu_device, &texture_ci);
}

void Context::createSampler() {
    SDL_GPUSamplerCreateInfo ci;
    ci.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    ci.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    ci.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    ci.enable_anisotropy = false;
    ci.compare_op = SDL_GPU_COMPAREOP_ALWAYS;
    ci.enable_compare = false;
    ci.mag_filter = SDL_GPU_FILTER_LINEAR;
    ci.min_filter = SDL_GPU_FILTER_LINEAR;
    ci.max_lod = 1.0;
    ci.min_lod = 1.0;
    ci.mip_lod_bias = 0.0;
    ci.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_LINEAR;

    m_gpu_sampler = SDL_CreateGPUSampler(m_gpu_device, &ci);
}

void Context::renderUpdate() {
    bool is_minimized = SDL_GetWindowFlags(m_window) & SDL_WINDOW_MINIMIZED;
    if (is_minimized) {
        return;
    }

    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(m_gpu_device);
    SDL_GPUTexture* swapchain_texture = nullptr;
    Uint32 width, height;

    if (!SDL_WaitAndAcquireGPUSwapchainTexture(cmd, m_window,
                                               &swapchain_texture,
                                               &width, &height)) {
        LOGE("SDL swapchain texture acquire failed! {}",
             SDL_GetError());
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

    // SDL_GPUBufferBinding binding;
    // binding.buffer = gPlaneVertexBuffer;
    // binding.offset = 0;
    // SDL_BindGPUVertexBuffers(render_pass, 0, &binding, 1);

    // SDL_GPUTextureSamplerBinding sampler_binding;
    // sampler_binding.texture = m_gpu_texture;
    // sampler_binding.sampler = m_gpu_sampler;
    // SDL_BindGPUFragmentSamplers(render_pass, 0, &sampler_binding, 1);

    // MVP mvp;
    // mvp.proj = m_camera->GetProject();
    // mvp.view = m_camera->GetView();
    // // TODO: model
    // SDL_PushGPUVertexUniformData(cmd, 0, &mvp, sizeof(MVP));

    // int window_width, window_height;
    // SDL_GetWindowSize(m_window, &window_width, &window_height);

    // SDL_GPUViewport viewport;
    // viewport.x = 0;
    // viewport.y = 0;
    // viewport.w = window_width;
    // viewport.h = window_height;
    // viewport.min_depth = 0;
    // viewport.max_depth = 1;
    // SDL_SetGPUViewport(render_pass, &viewport);
    // SDL_DrawGPUPrimitives(render_pass, 36, 1, 0, 0);

    SDL_EndGPURenderPass(render_pass);

    if (!SDL_SubmitGPUCommandBuffer(cmd)) {
        LOGE("SDL submit command buffer failed! {}", SDL_GetError());
    }
}

void Context::logicUpdate() {
}

void Context::Update() {
    logicUpdate();
    renderUpdate();
}

void Context::HandleEvents(const SDL_Event& event) {
    if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) {
        auto window_id = SDL_GetWindowID(m_window);
        if (event.window.windowID == window_id) {
            m_should_exit = true;
        }
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
