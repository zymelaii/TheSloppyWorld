#include "OpenGLSampleFactory.h"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QDebug>
#include <memory>

int main(int argc, char* argv[]) {
    QApplication	   app(argc, argv);
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.setApplicationDescription(R"(
Modern OpenGL samples gallery (built with Qt 6.4.1) powered by zymelaii.)");

    parser.addOption(
        QCommandLineOption({"t", "target"}, "target sample id", "sampleId", "BetterScene"));
    parser.process(app);

    AbstractOpenGLSample* w = OpenGLSampleFactory::getSample(parser.value("target"));
    if (w == nullptr) {
        exit(-1);
    } else {
        w->show();
    }

    QObject::connect(&app, &QCoreApplication::aboutToQuit, &app, [w] { w->deleteLater(); });

    return app.exec();
}
