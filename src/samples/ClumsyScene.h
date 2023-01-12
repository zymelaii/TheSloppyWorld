#ifndef CLUMSYSCENE_H
#define CLUMSYSCENE_H

#include "../AbstractOpenGLSample.h"

#include <QTimer>
#include <QKeyEvent>
#include <Eigen/Core>
#include <vector>

struct Camera;

class ClumsyScene : public AbstractOpenGLSample {
	Q_OBJECT

public:
	explicit ClumsyScene(QWidget* parent = nullptr);
	~ClumsyScene();

	virtual void keyPressEvent(QKeyEvent* e) override;
	virtual void keyReleaseEvent(QKeyEvent* e) override;

	virtual void initializeGL() override;
	virtual void resizeGL(int w, int h) override;
	virtual void paintGL() override;

	Eigen::Matrix4f getTranslationTransform(const Eigen::Vector3f& translate);
	Eigen::Matrix4f getRotationTransform(float angle, const Eigen::Vector3f& axis);

public slots:
	void onIdle();

protected:
	void updateRoleState();

	void initBufferObject();
	void attachToProgram(GLuint program, GLenum shaderType, const char* source);
	void finalizeProgram(GLuint program);
	void enableShaderProgram(GLuint program);
	void addCubeObject(const Eigen::Vector3f& pos, const Eigen::Vector3f& scale, float angle);

public:
	constexpr static float velocity		   = 16.0;
	constexpr static float angularVelocity = 360.0;

private:
	QTimer idleTimer_;

	Eigen::Matrix4f perspectiveProj_;
	Eigen::Matrix4f orthoProj_;
	Eigen::Matrix4f WVP_;

	Eigen::Vector3f roleSize_;
	Eigen::Vector3f rolePosition_, roleStep_, roleStepAcc_;
	float			rolePitch_, rolePitchStep_, rolePitchAcc_;
	float			viewPitch_;
	float			faceToAngle_, faceToTarget_;

	std::vector<Eigen::Vector3f>		   vertices_;
	std::vector<GLuint>					   indices_;
	std::vector<std::pair<GLuint, GLuint>> shaders_;

	GLuint program_;
	GLuint VBO_, IBO_;
};

#endif	 // CLUMSYSCENE_H
