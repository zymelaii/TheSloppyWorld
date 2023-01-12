#ifndef SINGLETRIANGLESAMPLE_H
#define SINGLETRIANGLESAMPLE_H

#include "../AbstractOpenGLSample.h"

class SingleTriangleSample : public AbstractOpenGLSample {
public:
    explicit SingleTriangleSample(QWidget* parent = nullptr);

    virtual void paintGL() override;
};

#endif	 // SINGLETRIANGLESAMPLE_H
