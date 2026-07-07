#define SDL_MAIN_USE_CALLBACKS
#include "context.hpp"
#include "SDL3/SDL.h"
#include "SDL3/SDL_main.h"
#include "spdlog/spdlog.h"
#include "toy_physics/log.hpp"

SDL_AppResult SDL_AppInit(void **appstate, int argc, char **argv) {
    Context::Init();
    Context::GetInst().InitSystem();
    Context::GetInst().Initialize();

    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate) {
    if (SCONTEXT.ShouldExit()) {
        return SDL_APP_SUCCESS;
    }
    SCONTEXT.Update();
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
    SCONTEXT.HandleEvents(*event);
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
    Context::GetInst().Shutdown();
    Context::GetInst().ShutdownSystem();
    Context::Destroy();
}