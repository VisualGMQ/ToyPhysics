#pragma once
#include <string>

class Context;

class IExample {
public:
    explicit IExample(Context& ctx, const std::string& name)
        : m_ctx{ctx}, m_name{name} {}

    virtual ~IExample() = default;

    virtual void OnUpdate(float delta_time) {}

    virtual void OnRender2D(float delta_time) {}

    virtual void OnRender3D(float delta_time) {}

    [[nodiscard]] const std::string& GetName() const { return m_name; }

protected:
    Context& m_ctx;

private:
    std::string m_name;
};