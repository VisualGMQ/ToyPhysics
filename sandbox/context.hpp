#pragma once
#include <memory>
#include "SDL3/SDL.h"

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

    SDL_Window* m_window{};

private:
    bool m_should_exit = true;
 
    static std::unique_ptr<Context> instance;
};

#define SCONTEXT ::Context::GetInst()