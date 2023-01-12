#version 330

layout (location = 0) in vec3 Position;

uniform mat4 WVP;

out vec3 color;

void main() {
    gl_Position = WVP * vec4(Position, 1.0);
    color = vec3(sin(Position.x), cos(Position.y), tan(Position.z));
}
