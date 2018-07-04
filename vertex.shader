#version 430 core

layout(location = 0) in vec3 VertexPos;
layout(location = 1) in vec3 VertexColor;
layout(location = 2) in vec3 VertexNormal;

uniform mat4 MVP;

out vec3 FragmentColor;

void main(){
    gl_Position = MVP * vec4(VertexPos,1.0);
    gl_PointSize = 1.0f;
    FragmentColor = VertexColor;
}
