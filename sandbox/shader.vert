#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec2 inUV;
layout(location = 2) in vec3 inNormal;

layout(location = 0) out vec2 fragUV;
layout(location = 1) out vec3 fragNormal;

layout(set = 1, binding = 0) uniform MVP {
    mat4 proj;
    mat4 view;
    mat4 model;
} mvp;

void main() {
    gl_Position = mvp.proj * mvp.view * mvp.model * vec4(inPosition, 1.0);
    fragUV = inUV;
    fragNormal = mat3(transpose(inverse(mvp.model))) * inNormal;
}