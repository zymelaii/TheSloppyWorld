#include "SingleTriangleSample.h"

#include <Eigen/Core>
#include <stddef.h>

using vec3f = Eigen::Vector3f;

SingleTriangleSample::SingleTriangleSample(QWidget* parent)
    : AbstractOpenGLSample(parent) {}

void SingleTriangleSample::paintGL() {
    constexpr size_t  numberOfVertices		   = 3;
    constexpr size_t  numberOfBufferObjects	   = 1;
    constexpr GLuint  defaultVertexAttribIndex = 0;
    constexpr size_t  numberOfDataComponents   = 3;
    constexpr size_t  dataSpacing			   = 0;
    constexpr GLvoid* dataOffset			   = static_cast<GLvoid*>(0);
    constexpr size_t  offsetOfFirstData		   = 0;

    vec3f vertices[numberOfVertices];
    vertices[0] = vec3f(.5, .3, .0);
    vertices[1] = vec3f(-.5, .3, .0);
    vertices[2] = vec3f(.0, -.5, .0);

    GLuint VBOs[numberOfBufferObjects];
    glGenBuffers(numberOfBufferObjects, VBOs);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);

    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

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

    glDeleteBuffers(numberOfBufferObjects, VBOs);
}
