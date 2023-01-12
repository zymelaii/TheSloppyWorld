#ifndef ABSTRACTOPENGLSAMPLE_H
#define ABSTRACTOPENGLSAMPLE_H

#include <QWidget>
#include <QtOpenGLWidgets/QtOpenGLWidgets>
#include <QOpenGLFunctions_4_5_Core>

class AbstractOpenGLSample : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core {
public:
    explicit AbstractOpenGLSample(QWidget* parent = nullptr);

    virtual void initializeGL() override;
    virtual void resizeGL(int w, int h) override;
};
#endif	 // ABSTRACTOPENGLSAMPLE_H
