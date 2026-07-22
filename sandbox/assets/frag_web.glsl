#version 100
precision mediump float;
varying vec3 fragPosition;
varying vec2 fragTexCoord;
varying vec3 fragNormal;
varying vec4 fragColor;
uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec3 lightDir;
uniform vec3 viewPos;
uniform vec3 lightColor;
uniform vec3 ambientColor;
uniform float specularStrength;
uniform float shininess;
void main() {
    vec4 texelColor = texture2D(texture0, fragTexCoord);
    vec4 color = texelColor * colDiffuse * fragColor;
    vec3 ambient = ambientColor * color.rgb;
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(-lightDir);
    float diff = max(dot(N, L), 0.0);
    vec3 diffuse = diff * lightColor * color.rgb;
    vec3 V = normalize(viewPos - fragPosition);
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), shininess);
    vec3 specular = spec * specularStrength * lightColor;
    gl_FragColor = vec4(ambient + diffuse + specular, color.a);
}
