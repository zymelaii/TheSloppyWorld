#ifndef TRIANGLEWITHSHADERSAMPLE_H
#define TRIANGLEWITHSHADERSAMPLE_H

#include "../AbstractOpenGLSample.h"

class TriangleWithShaderSample : public AbstractOpenGLSample {
public:
    explicit TriangleWithShaderSample(QWidget* parent = nullptr);

    virtual void paintGL() override;
};

#endif	 // TRIANGLEWITHSHADERSAMPLE_H
