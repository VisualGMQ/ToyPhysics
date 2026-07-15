#define SDL_MAIN_USE_CALLBACKS
#include "context.hpp"
#include "spdlog/spdlog.h"
#include "toy_physics/log.hpp"

int main(int argc, char** argv) {
    Context::Init();
    auto& inst = Context::GetInst();
    inst.Initialize();
    inst.Update();
    inst.Shutdown();
    Context::Destroy();
    return 0;
}