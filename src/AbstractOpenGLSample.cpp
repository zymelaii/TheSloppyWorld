#include "AbstractOpenGLSample.h"

AbstractOpenGLSample::AbstractOpenGLSample(QWidget* parent)
    : QOpenGLWidget(parent) {}

void AbstractOpenGLSample::initializeGL() {
    initializeOpenGLFunctions();
}

void AbstractOpenGLSample::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}
