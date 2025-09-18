#include "context.hpp"

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

    m_window = SDL_CreateWindow("ToyPhysics Sandbox", 1024, 720,
                                SDL_WINDOW_RESIZABLE);
    if (!m_window) {
        LOGE("create window failed");
    }
}

void Context::Shutdown() {
    m_should_exit = true;

    SDL_DestroyWindow(m_window);
}

void Context::Update() {
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
