#pragma once
#include "example.hpp"

class RenderTestExample : public IExample {
public:
    using IExample::IExample;
    void OnUpdate(float delta_time) override;
    void OnRender3D(float delta_time) override;
private:
};