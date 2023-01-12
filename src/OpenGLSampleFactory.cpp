#include "OpenGLSampleFactory.h"

#include "samples/SinglePointSample.h"
#include "samples/SingleTriangleSample.h"
#include "samples/TriangleWithShaderSample.h"

AbstractOpenGLSample* OpenGLSampleFactory::getSample(const QString& sampleName) {
	AbstractOpenGLSample* sample = nullptr;
	if (sampleName == "SinglePoint") {
		sample = new SinglePointSample;
	} else if (sampleName == "SingleTriangle") {
		sample = new SingleTriangleSample;
	} else if (sampleName == "TriangleWithShader") {
		sample = new TriangleWithShaderSample;
	}
	return sample;
}
