#version 330 core
layout (location = 0) in vec3 aPos;   // the position variable has attribute position 0

uniform mat4 modelView;
uniform mat4 projection;

void main() {
    gl_Position = modelView * vec4(aPos, 1.0f);
}
