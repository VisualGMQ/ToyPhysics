#pragma once
#include <memory>
#include "Eigen/Dense"
#include "vertex.hpp"

#include "camera.hpp"
#include "SDL3/SDL.h"

struct MVP {
    Eigen::Matrix4f proj = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
};

struct CPUMesh {
    std::vector<Vertex> m_vertices;

    void Load(const std::string& filename);
    operator bool() const {
        return !m_vertices.empty();
    }
};

struct GPUMesh {
    SDL_GPUBuffer* m_buffer{};
    size_t m_vertex_count{};
};

struct Transform {
    Eigen::Vector3f m_position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf m_rotation = Eigen::Quaternionf::Identity();
    Eigen::Vector3f m_scale = Eigen::Vector3f::Ones();

    Eigen::Matrix4f ToMatrix() const;
};

struct Model {
    const GPUMesh* m_mesh{};

    Transform m_transform;
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
    SDL_GPUGraphicsPipeline* createGraphicsPipeline();
    SDL_GPUTexture* createDepthTexture(int w, int h);
    SDL_GPUTexture* createImageTexture(uint32_t* color, int w, int h);
    SDL_GPUSampler* createSampler();
    GPUMesh loadModel(const std::string& filename);
    SDL_GPUBuffer* createAndUploadVertexData(const CPUMesh&);

    void renderUpdate();
    void logicUpdate(float delta_time);
    void drawCube(const Transform& transform);
    void drawSphere(const Transform& transform);
    void drawSemiSphere(const Transform& transform);
    void drawCylinder(const Transform& transform);
    void drawCapsule(const Transform& transform);
    void drawModel(const GPUMesh&, const Transform& transform);
    void handleFlyCamera(float delta_time);

    SDL_Window* m_window{};
    SDL_GPUDevice* m_gpu_device{};
    SDL_GPUTexture* m_gpu_depth_texture{};
    SDL_GPUTexture* m_gpu_white_texture{};
    SDL_GPUTexture* m_gpu_colorful_texture{};
    SDL_GPUSampler* m_gpu_sampler{};
    SDL_GPUShader* m_vertex_shader{};
    SDL_GPUShader* m_fragment_shader{};
    SDL_GPUGraphicsPipeline* m_graphics_pipeline{};
    bool m_is_mouse_relative_mode = true;
    float m_camera_move_speed = 0.001;

    // meshes
    GPUMesh m_cube_mesh;
    GPUMesh m_sphere_mesh;
    GPUMesh m_semi_sphere_mesh;
    GPUMesh m_cylinder_mesh;

    std::vector<Model> m_models;
};

#define SCONTEXT ::Context::GetInst()