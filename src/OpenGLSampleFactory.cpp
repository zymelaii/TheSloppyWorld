#include "OpenGLSampleFactory.h"

#include "samples/SinglePointSample.h"
#include "samples/SingleTriangleSample.h"

AbstractOpenGLSample* OpenGLSampleFactory::getSample(const QString& sampleName) {
	AbstractOpenGLSample* sample = nullptr;
	if (sampleName == "SinglePoint") {
		sample = new SinglePointSample;
	} else if (sampleName == "SingleTriangle") {
        sample = new SingleTriangleSample;
    }
	return sample;
}
