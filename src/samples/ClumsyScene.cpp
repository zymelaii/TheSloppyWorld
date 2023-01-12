#include "ClumsyScene.h"

#include <QWidget>
#include <QSurfaceFormat>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>

ClumsyScene::ClumsyScene(QWidget* parent)
	: AbstractOpenGLSample(parent)
	, idleTimer_(this)
	, roleStep_(0.0, 0.0, 0.0)
	, rolePitchStep_(0.0)
	, faceToTarget_(270.0) {
	QSurfaceFormat surfaceFormat;
	surfaceFormat.setSamples(4);
	setFormat(surfaceFormat);

	rolePosition_ = Eigen::Vector3f::Zero();
	roleSize_	  = Eigen::Vector3f(.4, .2, 1.6);
	viewPitch_	  = .0;

	faceToAngle_ = faceToTarget_;

	addCubeObject(rolePosition_, roleSize_, viewPitch_);			 //!< Role
	addCubeObject(rolePosition_, roleSize_, viewPitch_);			 //!< Relative
	addCubeObject({13, 25, 0}, {8, 8, 4}, qDegreesToRadians(30));	 //!< Cube 1
	addCubeObject({2.8, -5, 0}, {4, 4, 6}, qDegreesToRadians(75));	 //!< Cube 2
}

ClumsyScene::~ClumsyScene() {
	idleTimer_.stop();

	glUseProgram(0);
	glDeleteProgram(program_);

	glDeleteBuffers(1, &VBO_);
	glDeleteBuffers(1, &IBO_);
}

void ClumsyScene::keyPressEvent(QKeyEvent* e) {
	switch (e->key()) {
		case Qt::Key_W: {
			roleStep_.y() += velocity;
			e->accept();
		} break;
		case Qt::Key_A: {
			roleStep_.x() -= velocity;
			e->accept();
		} break;
		case Qt::Key_S: {
			roleStep_.y() -= velocity;
			e->accept();
		} break;
		case Qt::Key_D: {
			roleStep_.x() += velocity;
			e->accept();
		} break;
		case Qt::Key_Left: {
			rolePitchStep_ += angularVelocity;
			e->accept();
		} break;
		case Qt::Key_Right: {
			rolePitchStep_ -= angularVelocity;
			e->accept();
		} break;
		default: {
			e->ignore();
			return;
		}
	}
}

void ClumsyScene::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
		case Qt::Key_W: {
			roleStep_.y() -= velocity;
			e->accept();
		} break;
		case Qt::Key_A: {
			roleStep_.x() += velocity;
			e->accept();
		} break;
		case Qt::Key_S: {
			roleStep_.y() += velocity;
			e->accept();
		} break;
		case Qt::Key_D: {
			roleStep_.x() -= velocity;
			e->accept();
		} break;
		case Qt::Key_Left: {
			rolePitchStep_ -= angularVelocity;
			e->accept();
		} break;
		case Qt::Key_Right: {
			rolePitchStep_ += angularVelocity;
			e->accept();
		} break;
		default: {
			e->ignore();
			return;
		}
	}
}

void ClumsyScene::initializeGL() {
	AbstractOpenGLSample::initializeGL();

	initBufferObject();

	auto vertexShader = R"(#version 460
layout (location = 0) in vec3 Position;
uniform mat4 WVP;
void main() {
    gl_Position = WVP * vec4(Position, 1.0);
})";

	auto fragmentShader = R"(#version 460
in vec4 Color;
out vec4 FragColor;
void main() {
    vec4 lightColor = vec4(1.0, 1.0, 0.0, 1.0);
    float intensity = 0.8;
    FragColor = lightColor * intensity;
})";

	program_ = glCreateProgram();
	attachToProgram(program_, GL_VERTEX_SHADER, vertexShader);
	attachToProgram(program_, GL_FRAGMENT_SHADER, fragmentShader);
	finalizeProgram(program_);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	connect(&idleTimer_, &QTimer::timeout, this, &ClumsyScene::onIdle);
	idleTimer_.start(0);
}

void ClumsyScene::resizeGL(int w, int h) {
	AbstractOpenGLSample::resizeGL(w, h);

	const float aspectRatio = 1.0 * w / h;
	const float fieldOfView = 60.0;
	const float zNear		= 0.5;
	const float zFar		= 1024.0;

	const float tanHalfFOV = tan(qDegreesToRadians(fieldOfView) * 0.5);
	const float zRange	   = zFar - zNear;

	perspectiveProj_.setIdentity();
	perspectiveProj_(0, 0) = 1.0 / (tanHalfFOV * aspectRatio);
	perspectiveProj_(1, 1) = 1.0 / tanHalfFOV;
	perspectiveProj_(2, 2) = -(zNear + zFar) / zRange;
	perspectiveProj_(2, 3) = -2.0 * zNear * zFar / zRange;
	perspectiveProj_(3, 2) = -1.0;

	orthoProj_.setIdentity();
	orthoProj_(0, 0) = .5 / 8.;
	orthoProj_(1, 1) = .5 / 8. * aspectRatio;
	orthoProj_(2, 2) = -2. / zRange;
	orthoProj_(2, 3) = -(zNear + zFar) / zRange;
}

void ClumsyScene::paintGL() {
	GLint location = glGetUniformLocation(program_, "WVP");
	glUniformMatrix4fv(location, 1, GL_FALSE, WVP_.data());

	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_);
	glBufferSubData(GL_ARRAY_BUFFER, 0, 8 * sizeof(Eigen::Vector3f), vertices_.data());
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO_);

	enableShaderProgram(program_);
	glDrawElements(GL_QUADS, indices_.size(), GL_UNSIGNED_INT, 0);

	glDisableVertexAttribArray(0);
}

Eigen::Matrix4f ClumsyScene::getTranslationTransform(const Eigen::Vector3f& translate) {
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	transform(0, 3) = translate.x();
	transform(1, 3) = translate.y();
	transform(2, 3) = translate.z();
	return transform;
}

Eigen::Matrix4f ClumsyScene::getRotationTransform(float angle, const Eigen::Vector3f& axis) {
	Eigen::AngleAxisf angleAxis(qDegreesToRadians(angle), axis);
	Eigen::Matrix3f	  rotateMatrix = angleAxis.matrix();
	Eigen::Matrix4f	  transform	   = Eigen::Matrix4f::Identity();

	transform(0, 0) = rotateMatrix(0, 0);
	transform(0, 1) = rotateMatrix(0, 1);
	transform(0, 2) = rotateMatrix(0, 2);
	transform(1, 0) = rotateMatrix(1, 0);
	transform(1, 1) = rotateMatrix(1, 1);
	transform(1, 2) = rotateMatrix(1, 2);
	transform(2, 0) = rotateMatrix(2, 0);
	transform(2, 1) = rotateMatrix(2, 1);
	transform(2, 2) = rotateMatrix(2, 2);
	return transform;
}

void ClumsyScene::onIdle() {
	using std::chrono::duration_cast;
	static auto idleRec	  = std::chrono::high_resolution_clock::now();
	static auto updateRec = idleRec;

	auto now  = std::chrono::high_resolution_clock::now();
	auto diff = duration_cast<std::chrono::nanoseconds>(now - idleRec).count();
	idleRec	  = now;

	Eigen::Vector3f roleStep = Eigen::Vector3f::Zero();
	if (!roleStep_.isZero()) {
		roleStep = roleStep_.normalized() * velocity;
	}

	roleStepAcc_ += roleStep * diff * 1e-9;
	rolePitchAcc_ += rolePitchStep_ * diff * 1e-9;

	diff = duration_cast<std::chrono::milliseconds>(now - updateRec).count();
	if (diff > 10) {
		updateRoleState();
		update();
		updateRec = now;
	}
}

void ClumsyScene::updateRoleState() {
	auto oldFaceToAngle = faceToAngle_;
	auto oldRolePitch	= rolePitch_;

	if (!roleStep_.isZero()) {
		rolePitch_	  = viewPitch_;
		auto dot	  = Eigen::Vector3f::UnitX().dot(roleStep_.normalized());
		auto cross	  = Eigen::Vector3f::UnitX().cross(roleStep_);
		auto angle	  = qRadiansToDegrees(acos(dot));
		faceToTarget_ = cross.z() >= 0. ? angle : 360.0 - angle;
		if (faceToTarget_ - faceToAngle_ > 180.0) {
			faceToTarget_ -= 360.0;
		}
	}

	faceToAngle_ += (faceToTarget_ - faceToAngle_) * 0.2;

	auto finalPitch = qDegreesToRadians(rolePitch_);
	auto diffPitch	= qDegreesToRadians(rolePitch_ - oldRolePitch + faceToAngle_ - oldFaceToAngle);

	auto stepRotate = Eigen::AngleAxisf(finalPitch, Eigen::Vector3f::UnitZ()).matrix();
	auto selfRotate = Eigen::AngleAxisf(diffPitch, Eigen::Vector3f::UnitZ()).matrix();

	Eigen::Vector3f totalStep = stepRotate * roleStepAcc_;
	for (int i = 0; i < 8; ++i) {
		vertices_[i] -= rolePosition_;
		vertices_[i] = selfRotate * vertices_[i];
		vertices_[i] += rolePosition_;
		vertices_[i] += totalStep;
	}
	viewPitch_ += rolePitchAcc_;
	rolePosition_ += totalStep;

	roleStepAcc_.setZero();
	rolePitchAcc_ = 0.;

	constexpr float viewDistance = 24.0;

	Eigen::Matrix4f WorldToRole = getRotationTransform(-viewPitch_, Eigen::Vector3f::UnitZ()) *
								  getTranslationTransform(-rolePosition_);
	Eigen::Matrix4f RoleToCamera =
		getTranslationTransform(Eigen::Vector3f::UnitZ() * -viewDistance) *
		getRotationTransform(-45.0, Eigen::Vector3f::UnitX());

	WVP_ = perspectiveProj_ * RoleToCamera * WorldToRole;
}

void ClumsyScene::initBufferObject() {
	glGenBuffers(1, &VBO_);
	glGenBuffers(1, &IBO_);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_);
	glBufferData(GL_ARRAY_BUFFER,
				 vertices_.size() * sizeof(Eigen::Vector3f),
				 vertices_.data(),
				 GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO_);
	glBufferData(
		GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(GLuint), indices_.data(), GL_STATIC_DRAW);
}

void ClumsyScene::attachToProgram(GLuint program, GLenum shaderType, const char* source) {
	GLuint shader = glCreateShader(shaderType);
	GLint  length = strlen(source);

	glShaderSource(shader, 1, &source, &length);
	glCompileShader(shader);

	GLint success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[1024];
		glGetShaderInfoLog(shader, sizeof(infoLog), nullptr, infoLog);
		qDebug().noquote()
			<< QString("Error compiling shader type %1: %2").arg(shaderType).arg(infoLog);
	}

	glAttachShader(program, shader);
	shaders_.emplace_back(program, shader);
}

void ClumsyScene::finalizeProgram(GLuint program) {
	glLinkProgram(program);

	GLint success;
	glGetProgramiv(program, GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[1024];
		glGetProgramInfoLog(program, sizeof(infoLog), nullptr, infoLog);
		qDebug().noquote() << QString("Error linking shader program: %1").arg(infoLog);
	}

	glValidateProgram(program);

	auto it =
		std::remove_if(shaders_.begin(),
					   shaders_.end(),
					   [this, thisProgram = program](const decltype(shaders_)::value_type& e) {
						   const auto& [program, shader] = e;
						   const bool shouldRemove		 = program == thisProgram;
						   if (shouldRemove) {
							   glDetachShader(program, shader);
							   glDeleteShader(shader);
						   }
						   return shouldRemove;
					   });
	shaders_.erase(it, shaders_.end());
}

void ClumsyScene::enableShaderProgram(GLuint program) {
	glUseProgram(program);
}

void ClumsyScene::addCubeObject(const Eigen::Vector3f& pos, const Eigen::Vector3f& scale,
								float angle) {
	Eigen::Vector3f cubeVertices[8]{
		{.5, .5, -.5},
		{-.5, .5, -.5},
		{-.5, -.5, -.5},
		{.5, -.5, -.5},
		{.5, .5, .5},
		{-.5, .5, .5},
		{-.5, -.5, .5},
		{.5, -.5, .5},
	};

	Eigen::AngleAxisf rotateAxis(angle / 180. * M_PI, Eigen::Vector3f(0., 0., 1.));
	Eigen::Matrix3f	  rotateMatrix = rotateAxis.matrix();

	int offset = vertices_.size();

	for (auto& vertex : cubeVertices) {
		vertex.x() *= scale.x();
		vertex.y() *= scale.y();
		vertex.z() *= scale.z();
		vertex = rotateMatrix * vertex;
		vertex += pos;
		vertex.z() += scale.z() * .5;
		vertices_.emplace_back(vertex);
	}

	GLuint quadsindices[] = {0, 1, 2, 3, 4, 5, 6, 7, 1, 2, 6, 5,
							 0, 4, 7, 3, 0, 1, 5, 4, 2, 3, 7, 6};
	for (auto& e : quadsindices) {
		indices_.emplace_back(e + offset);
	}
}
