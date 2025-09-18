#version 450

layout(location = 0) in vec2 fragUV;
layout(location = 1) in vec3 fragNormal;

layout(location = 0) out vec4 outColor;

layout(set = 2, binding = 0) uniform sampler2D mySampler;

const vec3 directLightDir = normalize(vec3(-1, -1, -1));
const float ambient = 0.4;
const float directLightStrength = 0.6;

void main() {
    float diffuse = max(dot(fragNormal, directLightDir), 0);
    outColor = texture(mySampler, fragUV) * (ambient + diffuse * directLightStrength);
}