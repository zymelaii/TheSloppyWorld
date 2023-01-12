#ifndef SINGLEPOINTSAMPLE_H
#define SINGLEPOINTSAMPLE_H

#include "../AbstractOpenGLSample.h"

class SinglePointSample : public AbstractOpenGLSample {
public:
    explicit SinglePointSample(QWidget* parent = nullptr);

    virtual void paintGL() override;
};

#endif	 // SINGLEPOINTSAMPLE_H
