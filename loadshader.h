#ifndef LOADSHADER_H
#define LOADSHADER_H

#include "includes.h"

static char* readShaderSource(const char* shaderFile);
// Create a GLSL program object from vertex and fragment shader files
GLuint LoadShaders(const char* vShaderFile, const char* fShaderFile);

void uniformRegister(mat4& var,GLuint shaderID,const char * string);
void uniformRegister(mat3& var,GLuint shaderID,const char * string);
void uniformRegister(vec4 var,GLuint shaderID,const char * string);
void uniformRegister(vec3 var,GLuint shaderID,const char * string);
void uniformRegister(GLfloat var,GLuint shaderID,const char * string);
void uniformRegister(GLint var,GLuint shaderID,const char * string);
void printMatrix(mat4 matrix);

#endif // LOADSHADER_H
