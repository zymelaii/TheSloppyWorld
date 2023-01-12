#ifndef OPENGLSAMPLEFACTORY_H
#define OPENGLSAMPLEFACTORY_H

#include "AbstractOpenGLSample.h"

class OpenGLSampleFactory {
public:
    static AbstractOpenGLSample* getSample(const QString& sampleName);
};

#endif	 // OPENGLSAMPLEFACTORY_H
