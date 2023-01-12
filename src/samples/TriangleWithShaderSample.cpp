#include "TriangleWithShaderSample.h"

#include <Eigen/Core>
#include <stddef.h>

using vec3f = Eigen::Vector3f;

TriangleWithShaderSample::TriangleWithShaderSample(QWidget* parent)
    : AbstractOpenGLSample(parent) {}

void TriangleWithShaderSample::paintGL() {
    constexpr size_t  numberOfVertices		   = 3;
    constexpr size_t  numberOfBufferObjects	   = 1;
    constexpr GLuint  defaultVertexAttribIndex = 0;
    constexpr size_t  numberOfDataComponents   = 3;
    constexpr size_t  dataSpacing			   = 0;
    constexpr GLvoid* dataOffset			   = static_cast<GLvoid*>(0);
    constexpr size_t  offsetOfFirstData		   = 0;
    constexpr size_t  numberOfShaders		   = 2;
    constexpr size_t  numberOfShaderSources	   = 2;

    vec3f vertices[numberOfVertices];
    vertices[0] = vec3f(.5, .3, .0);
    vertices[1] = vec3f(-.5, .3, .0);
    vertices[2] = vec3f(.0, -.5, .0);

    GLuint VBOs[numberOfBufferObjects];
    glGenBuffers(numberOfBufferObjects, VBOs);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);

    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    GLuint shaderProgram = glCreateProgram();

    GLuint shaders[numberOfShaders];
    shaders[0] = glCreateShader(GL_VERTEX_SHADER);
    shaders[1] = glCreateShader(GL_FRAGMENT_SHADER);

    const GLchar* shaderSources[numberOfShaderSources];
    GLint		  shaderSourceLengths[numberOfShaderSources];

    shaderSources[0] = R"(#version 330
layout (location = 0) in vec3 Position;
out vec4 lerpColor;
void main() {
    gl_Position = vec4(Position, 1.0);
    lerpColor = vec4(clamp((Position + 1) / 2, 0.0, 1.0), 1.0);
})";

    shaderSources[1] = R"(#version 330
in vec4 lerpColor;
out vec4 FragColor;
void main() {
    FragColor = lerpColor;
})";

    for (int i = 0; i < numberOfShaderSources; ++i) {
        shaderSourceLengths[i] = strlen(shaderSources[i]);
    }

    glShaderSource(shaders[0], 1, &shaderSources[0], &shaderSourceLengths[0]);
    glShaderSource(shaders[1], 1, &shaderSources[1], &shaderSourceLengths[1]);

    for (int i = 0; i < numberOfShaders; ++i) {
        glCompileShader(shaders[i]);

        GLint success;
        glGetShaderiv(shaders[i], GL_COMPILE_STATUS, &success);
        if (!success) {
            GLchar infoLog[1024];
            glGetShaderInfoLog(shaders[i], sizeof(infoLog), nullptr, infoLog);

            GLint shaderType;
            glGetShaderiv(shaders[i], GL_SHADER_TYPE, &shaderType);

            qDebug().noquote()
                << QString("Error compiling shader type %1: %2").arg(shaderType).arg(infoLog);
        }

        glAttachShader(shaderProgram, shaders[i]);
    }

    glLinkProgram(shaderProgram);
    GLint success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        GLchar infoLog[1024];
        glGetProgramInfoLog(shaderProgram, sizeof(infoLog), nullptr, infoLog);
        qDebug().noquote() << QString("Error linking shader program: %1").arg(infoLog);
    }

    glValidateProgram(shaderProgram);
    glUseProgram(shaderProgram);

    glEnableVertexAttribArray(defaultVertexAttribIndex);
    {
        glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);
        glVertexAttribPointer(defaultVertexAttribIndex,
                              numberOfDataComponents,
                              GL_FLOAT,
                              GL_FALSE,
                              dataSpacing,
                              dataOffset);
        glDrawArrays(GL_TRIANGLES, offsetOfFirstData, numberOfVertices);
    }
    glDisableVertexAttribArray(defaultVertexAttribIndex);

    glUseProgram(0);
    for (int i = 0; i < numberOfShaders; ++i) {
        glDetachShader(shaderProgram, shaders[i]);
        glDeleteShader(shaders[i]);
    }
    glDeleteProgram(shaderProgram);

    glDeleteBuffers(numberOfBufferObjects, VBOs);
}
