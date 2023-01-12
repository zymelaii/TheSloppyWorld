#include "OpenGLSampleFactory.h"

#include "samples/SinglePointSample.h"

AbstractOpenGLSample* OpenGLSampleFactory::getSample(const QString& sampleName) {
	AbstractOpenGLSample* sample = nullptr;
	if (sampleName == "SinglePoint") {
		sample = new SinglePointSample;
	}
	return sample;
}
