#include "OpenGLSampleFactory.h"

#include "samples/SinglePointSample.h"
#include "samples/SingleTriangleSample.h"
#include "samples/TriangleWithShaderSample.h"
#include "samples/TriangleWithShaderAndRuntimeAttribSample.h"
#include "samples/IndexDrawingSample.h"
#include "samples/ClumsyScene.h"
#include "samples/BetterScene.h"

AbstractOpenGLSample* OpenGLSampleFactory::getSample(const QString& sampleName) {
    AbstractOpenGLSample* sample = nullptr;
    if (sampleName == "SinglePoint") {
        sample = new SinglePointSample;
    } else if (sampleName == "SingleTriangle") {
        sample = new SingleTriangleSample;
    } else if (sampleName == "TriangleWithShader") {
        sample = new TriangleWithShaderSample;
    } else if (sampleName == "TriangleWithShaderAndRuntimeAttrib") {
        sample = new TriangleWithShaderAndRuntimeAttribSample;
    } else if (sampleName == "IndexDrawing") {
        sample = new IndexDrawingSample;
    } else if (sampleName == "ClumsyScene") {
        sample = new ClumsyScene;
    } else if (sampleName == "BetterScene") {
        sample = new BetterScene;
    }
    return sample;
}
