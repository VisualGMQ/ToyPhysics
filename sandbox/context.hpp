#pragma once
#include <memory>
#include "Eigen/Dense"

#include "camera.hpp"
#include "SDL3/SDL.h"

struct MVP {
    Eigen::Matrix4f proj = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
};

class Context {
public:
    static void Init();
    static void Destroy();
    static Context& GetInst();

    virtual ~Context();

    void InitSystem();
    void ShutdownSystem();
    void Initialize();
    void Shutdown();

    void Update();
    void HandleEvents(const SDL_Event&);

    bool ShouldExit() const;
    void Exit();

    std::unique_ptr<Camera> m_camera;

private:
    bool m_should_exit = true;
 
    static std::unique_ptr<Context> instance;

    SDL_GPUShader* loadSDLGPUShader(const char* filename,
                                    SDL_GPUShaderStage stage,
                                    uint32_t sampler_num,
                                    uint32_t uniform_buffer_num);
    void createGraphicsPipeline();
    void createDepthTexture(int w, int h);
    void createSampler();

    void renderUpdate();
    void logicUpdate();
     
    SDL_Window* m_window{};
    SDL_GPUDevice* m_gpu_device{};
    SDL_GPUTexture* m_gpu_depth_texture{};
    SDL_GPUTexture* m_gpu_texture{};
    SDL_GPUSampler* m_gpu_sampler{};
    SDL_GPUShader* m_vertex_shader{};
    SDL_GPUShader* m_fragment_shader{};
    SDL_GPUGraphicsPipeline* m_graphics_pipeline{};
};

#define SCONTEXT ::Context::GetInst()