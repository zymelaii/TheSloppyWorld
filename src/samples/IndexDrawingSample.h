#ifndef INDEXDRAWINGSAMPLE_H
#define INDEXDRAWINGSAMPLE_H

#include "../AbstractOpenGLSample.h"

class IndexDrawingSample : public AbstractOpenGLSample {
public:
    explicit IndexDrawingSample(QWidget* parent = nullptr);

    virtual void paintGL() override;
};

#endif	 // INDEXDRAWINGSAMPLE_H
