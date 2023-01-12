#ifndef TRIANGLEWITHSHADERANDRUNTIMEATTRIBSAMPLE_H
#define TRIANGLEWITHSHADERANDRUNTIMEATTRIBSAMPLE_H

#include "../AbstractOpenGLSample.h"

class TriangleWithShaderAndRuntimeAttribSample : public AbstractOpenGLSample {
public:
    explicit TriangleWithShaderAndRuntimeAttribSample(QWidget* parent = nullptr);

    virtual void paintGL() override;
};

#endif	 // TRIANGLEWITHSHADERANDRUNTIMEATTRIBSAMPLE_H
