#include "BetterScene.h"

#include <Eigen/Dense>
#include <QFile>
#include <iterator>

Eigen::Matrix4f getTranslationTransform(const Eigen::Vector3f& translation) {
	Eigen::Matrix4f transform	= Eigen::Matrix4f::Identity();
	transform.block<3, 1>(0, 3) = translation.transpose();
	return transform;
}

Eigen::Matrix4f getAxisRotationTransform(float rotation, const Eigen::Vector3f& axis) {
	Eigen::Matrix4f transform	= Eigen::Matrix4f::Identity();
	const auto		angleAxis	= Eigen::AngleAxisf(qDegreesToRadians(rotation), axis);
	transform.block<3, 3>(0, 0) = angleAxis.matrix();
	return transform;
}

void createCubeObject(const Eigen::Vector3f& position, const Eigen::Vector3f& scale, float angle,
					  std::vector<Eigen::Vector3f>& outputVertices,
					  std::vector<GLuint>&			outputIndices) {
	Eigen::Vector3f cubeVertices[]{
		{.5, .5, -.5},
		{-.5, .5, -.5},
		{-.5, -.5, -.5},
		{.5, -.5, -.5},
		{.5, .5, .5},
		{-.5, .5, .5},
		{-.5, -.5, .5},
		{.5, -.5, .5},
	};

	GLuint cubeIndices[]{0, 3, 2, 1, 4, 5, 6, 7, 1, 2, 6, 5, 0, 4, 7, 3, 0, 1, 5, 4, 2, 3, 7, 6};

	const auto offset	= outputVertices.size();
	const auto rotation = Eigen::AngleAxisf(qDegreesToRadians(angle), Eigen::Vector3f::UnitZ());

	auto vit = std::back_inserter(outputVertices);
	for (const auto& cubeVertex : cubeVertices) {
		auto vertex = cubeVertex;
		vertex(0) *= scale(0);
		vertex(1) *= scale(1);
		vertex(2) *= scale(2);
		vertex = rotation * vertex + position;
		vertex.z() += scale.z() * .5;
		*vit++ = vertex;
	}

	auto iit = std::back_inserter(outputIndices);
	for (const auto& cubeIndex : cubeIndices) {
		*iit++ = cubeIndex + offset;
	}
}

void Role::moveForward(const Eigen::Vector3f& movement) {
	using namespace Eigen;
	auto rotation = AngleAxisf(qDegreesToRadians(currentDirection), Vector3f::UnitZ());
	position += rotation * movement;
}

void Role::update() {
	currentDirection += (targetDirection - currentDirection) * 0.2;
}

void Role::setTargetDirection(float direction) {
	targetDirection = direction;
	if (targetDirection - currentDirection > 180.0) {
		targetDirection -= 360.0;
	}
}

Eigen::Matrix4f Role::getWorld2RoleTransform() const {
	using namespace Eigen;
	Matrix4f translation = getTranslationTransform(-position);
	Matrix4f rotation	 = getAxisRotationTransform(-currentDirection, Vector3f::UnitZ());
	return rotation * translation;
}

void Camera::attach(Role* role, Eigen::Matrix4f transform) {
	if (attachedRole != nullptr) {
		detach();
	}
	attachedRole		  = role;
	roleToCameraTransform = transform;
}

void Camera::detach() {
	attachedRole		  = nullptr;
	roleToCameraTransform = Eigen::Matrix4f::Identity();
}

void Camera::setPose(Eigen::Vector3f lootAtDirection, Eigen::Vector3f upDirection) {
	lookAt = lootAtDirection.normalized();
	up	   = upDirection.normalized();
}

void Camera::installOrthoProjection(float width, float height, float zNear, float zFar) {
	const auto zRange = zFar - zNear;

	projectionTransform.setIdentity();
	projectionTransform(0, 0) = .5 / width;
	projectionTransform(1, 1) = .5 / height;
	projectionTransform(2, 2) = -2. / zRange;
	projectionTransform(2, 3) = -(zNear + zFar) / zRange;
}

void Camera::installPerspectiveProjection(float aspectRatio, float FOV, float zNear, float zFar) {
	const auto cotan  = 1. / tan(qDegreesToRadians(FOV) * .5);
	const auto zRange = zFar - zNear;

	projectionTransform.setIdentity();
	projectionTransform(0, 0) = cotan / aspectRatio;
	projectionTransform(1, 1) = cotan;
	projectionTransform(2, 2) = -(zNear + zFar) / zRange;
	projectionTransform(2, 3) = -2.0 * zNear * zFar / zRange;
	projectionTransform(3, 2) = -1.0;
}

Eigen::Matrix4f Camera::getWorld2CameraTransform() {
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	if (attachedRole != nullptr) {
		transform = roleToCameraTransform * attachedRole->getWorld2RoleTransform();
	} else {
		transform.block<3, 1>(0, 0) = lookAt.transpose();
		transform.block<3, 1>(0, 2) = up.transpose();
		transform.block<3, 1>(0, 3) = lookAt.cross(up).transpose();
	}
	return projectionTransform * transform;
}

BetterScene::BetterScene(QWidget* parent)
	: AbstractOpenGLSample(parent)
	, moveIndicate_(.0, .0, .0)
	, viewRotateIndicate_(.0, .0)
	, moveAccumulate_(.0, .0, .0)
	, viewRotateAccumulate_(.0, .0)
	, pitch_(.0)
	, gravity_(9.8)
	, zVelocity_(.0)
	, TPPCameraRotation_(.0)
	, viewJitter_(.0) {
	using namespace Eigen;
	role_.position		   = Eigen::Vector3f::Zero();
	role_.velocity		   = 16.0;
	role_.angularVelocity  = 360.0;
	role_.currentDirection = .0;
	role_.targetDirection  = role_.currentDirection;

	setViewMode(ViewMode::FPP);

	moveControlBindings_.clear();
	moveControlBindings_.emplace(Qt::Key_W, Eigen::Vector3f::UnitY());
	moveControlBindings_.emplace(Qt::Key_S, -Eigen::Vector3f::UnitY());
	moveControlBindings_.emplace(Qt::Key_A, -Eigen::Vector3f::UnitX());
	moveControlBindings_.emplace(Qt::Key_D, Eigen::Vector3f::UnitX());

	viewRotateControlBindings_.clear();
	viewRotateControlBindings_.emplace(Qt::Key_Left, Eigen::Vector2f::UnitX());
	viewRotateControlBindings_.emplace(Qt::Key_Right, -Eigen::Vector2f::UnitX());
	viewRotateControlBindings_.emplace(Qt::Key_Up, Eigen::Vector2f::UnitY());
	viewRotateControlBindings_.emplace(Qt::Key_Down, -Eigen::Vector2f::UnitY());

	QSurfaceFormat surfaceFormat;
	surfaceFormat.setSamples(4);
	setFormat(surfaceFormat);

	connect(&idleTimer_, &QTimer::timeout, this, &BetterScene::onIdle);
}

BetterScene::~BetterScene() {
	if (VBO_ != 0) {
		glDeleteBuffers(1, &VBO_);
		VBO_ = 0;
	}

	if (IBO_ != 0) {
		glDeleteBuffers(1, &IBO_);
		IBO_ = 0;
	}

	if (skyboxTexture_ != 0) {
		glDeleteTextures(1, &skyboxTexture_);
		skyboxTexture_ = 0;
	}

	if (skyboxVBO_ != 0) {
		glDeleteBuffers(1, &skyboxVBO_);
		skyboxVBO_ = 0;
	}

	camera_.detach();
}

void BetterScene::keyPressEvent(QKeyEvent* e) {
	const auto key = static_cast<Qt::Key>(e->key());

	if (auto it = moveControlBindings_.find(key); it != moveControlBindings_.end()) {
		moveIndicate_ += it->second;
		e->accept();
	}

	if (auto it = viewRotateControlBindings_.find(key); it != viewRotateControlBindings_.end()) {
		viewRotateIndicate_ += it->second;
		e->accept();
	}

	if (key == Qt::Key_Space) {
		if (role_.position.z() == 0.) {
			zVelocity_ += 4.0;
		}
		e->accept();
	}

	if (key == Qt::Key_Tab) {
		using namespace Eigen;
		if (viewMode_ == ViewMode::FPP) {
			setViewMode(ViewMode::TPP);
		} else if (viewMode_ == ViewMode::TPP) {
			setViewMode(ViewMode::FPP);
		}
		e->accept();
	}

	if (key == Qt::Key_Shift) {
		lockPitch_ = true;
		e->accept();
	}

	if (!e->isAccepted()) {
		e->ignore();
	}
}

void BetterScene::keyReleaseEvent(QKeyEvent* e) {
	const auto key = static_cast<Qt::Key>(e->key());

	if (auto it = moveControlBindings_.find(key); it != moveControlBindings_.end()) {
		moveIndicate_ -= it->second;
		e->accept();
	}

	if (auto it = viewRotateControlBindings_.find(key); it != viewRotateControlBindings_.end()) {
		viewRotateIndicate_ -= it->second;
		e->accept();
	}

	if (key == Qt::Key_Shift) {
		lockPitch_ = false;
		e->accept();
	}

	if (!e->isAccepted()) {
		e->ignore();
	}
}

void BetterScene::initializeGL() {
	AbstractOpenGLSample::initializeGL();

	setupScene();

	worldShader_.init();
	worldShader_.add(GL_VERTEX_SHADER, ":/assets/world.vs");
	worldShader_.add(GL_FRAGMENT_SHADER, ":/assets/world.fs");
	worldShader_.finalize();

	skyboxShader_.init();
	skyboxShader_.add(GL_VERTEX_SHADER, ":/assets/skybox.vs");
	skyboxShader_.add(GL_FRAGMENT_SHADER, ":/assets/skybox.fs");
	skyboxShader_.finalize();

	floorShader_.init();
	floorShader_.add(GL_VERTEX_SHADER, ":/assets/floor.vs");
	floorShader_.add(GL_FRAGMENT_SHADER, ":/assets/floor.fs");
	floorShader_.finalize();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	idleTimer_.start(0);
}

void BetterScene::resizeGL(int w, int h) {
	AbstractOpenGLSample::resizeGL(w, h);
	camera_.installPerspectiveProjection(1.0 * w / h, 60.0, .5, 1024.0);
}

void BetterScene::paintGL() {
	const size_t N = roleVertices_.size() * 3;

	if (viewMode_ == ViewMode::FPP) {
		renderSkybox();
	}

	renderFloor();

	GLint cullFaceMode;
	glGetIntegerv(GL_CULL_FACE_MODE, &cullFaceMode);
	glCullFace(GL_BACK);
	worldShader_.bind();
	glBindBuffer(GL_ARRAY_BUFFER, VBO_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO_);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	if (viewMode_ == ViewMode::TPP) {
		glDrawElements(GL_QUADS, indices_.size(), GL_UNSIGNED_INT, 0);
	} else {
		glDrawElements(GL_QUADS, indices_.size() - N, GL_UNSIGNED_INT, (void*)(N * sizeof(GLuint)));
	}
	glDisableVertexAttribArray(0);
	worldShader_.release();
	glCullFace(cullFaceMode);
}

void BetterScene::initRoleModel() {
	const auto headSize = Eigen::Vector3f(.4, .4, .38);
	const auto bodySize = Eigen::Vector3f(.4, .3, .6);
	const auto armSize	= Eigen::Vector3f(.09, .09, .72);
	const auto legSize	= Eigen::Vector3f(.18, .2, .8);

	//! head
	createCubeObject(Eigen::Vector3f(.0, .0, legSize.z() + bodySize.z()),
					 headSize,
					 role_.currentDirection,
					 vertices_,
					 indices_);
	
	//! body
	createCubeObject(Eigen::Vector3f(.0, .0, legSize.z()),
					 bodySize,
					 role_.currentDirection,
					 vertices_,
					 indices_);

	//! left arm
	createCubeObject(
		Eigen::Vector3f(
			-(bodySize.x() + armSize.x()) * .5, .0, legSize.z() + bodySize.z() - armSize.z()),
		armSize,
		role_.currentDirection,
		vertices_,
		indices_);
	
	//! right arm
	createCubeObject(
		Eigen::Vector3f(
			+(bodySize.x() + armSize.x()) * .5, .0, legSize.z() + bodySize.z() - armSize.z()),
		armSize,
		role_.currentDirection,
		vertices_,
		indices_);

	//! left leg
	createCubeObject(Eigen::Vector3f(-(bodySize.x() - legSize.x()) * .5, .0, .0),
					 legSize,
					 role_.currentDirection,
					 vertices_,
					 indices_);
	//! right leg
	createCubeObject(Eigen::Vector3f(+(bodySize.x() - legSize.x()) * .5, .0, .0),
					 legSize,
					 role_.currentDirection,
					 vertices_,
					 indices_);

	roleVertices_.assign(vertices_.begin(), vertices_.end());
}

void BetterScene::setupScene() {
	initRoleModel();

	createCubeObject({13, 25, 0}, {8, 8, 4}, 30.0, vertices_, indices_);	//!< Cube 0
	createCubeObject({2.8, -5, 0}, {4, 4, 6}, 75.0, vertices_, indices_);	//!< Cube 1

	glGenBuffers(1, &VBO_);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_);
	glBufferData(GL_ARRAY_BUFFER,
				 vertices_.size() * sizeof(Eigen::Vector3f),
				 vertices_.data(),
				 GL_DYNAMIC_DRAW);

	glGenBuffers(1, &IBO_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO_);
	glBufferData(
		GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(GLuint), indices_.data(), GL_STATIC_DRAW);


	initSkybox(":/assets/skybox.jpg");
}

void BetterScene::initSkybox(const QString& texturePath) {
	QImage		 texture(texturePath);
	const size_t size = texture.width() / 4;

	auto up		= texture.copy(size, 0, size, size);
	auto bottom = texture.copy(size, size * 2, size, size);
	auto front	= texture.copy(size, size, size, size);
	auto back	= texture.copy(size * 3, size, size, size);
	auto left	= texture.copy(0, size, size, size);
	auto right	= texture.copy(size * 2, size, size, size);

	QImage* textures[6]{&right, &left, &up, &bottom, &front, &back};

	glGenTextures(1, &skyboxTexture_);
	glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTexture_);

	for (int i = 0; i < 6; ++i) {
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
					 0,
					 GL_RGB,
					 size,
					 size,
					 0,
					 GL_BGRA,
					 GL_UNSIGNED_BYTE,
					 textures[i]->bits());

		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	}

	Eigen::Vector3f data[]{
		{+1.0, +1.0, -1.0}, {+1.0, +1.0, +1.0}, {+1.0, -1.0, +1.0}, {+1.0, -1.0, -1.0},
		{-1.0, +1.0, -1.0}, {-1.0, -1.0, -1.0}, {-1.0, -1.0, +1.0}, {-1.0, +1.0, +1.0},
		{+1.0, +1.0, -1.0}, {-1.0, +1.0, -1.0}, {-1.0, +1.0, +1.0}, {+1.0, +1.0, +1.0},
		{+1.0, -1.0, -1.0}, {+1.0, -1.0, +1.0}, {-1.0, -1.0, +1.0}, {-1.0, -1.0, -1.0},
		{+1.0, -1.0, +1.0}, {+1.0, +1.0, +1.0}, {-1.0, +1.0, +1.0}, {-1.0, -1.0, +1.0},
		{+1.0, -1.0, -1.0}, {-1.0, -1.0, -1.0}, {-1.0, +1.0, -1.0}, {+1.0, +1.0, -1.0},
	};

	glGenBuffers(1, &skyboxVBO_);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);
}

void BetterScene::renderSkybox() {
	GLint cullFaceMode;
	GLint depthFuncMode;
	glGetIntegerv(GL_CULL_FACE_MODE, &cullFaceMode);
	glGetIntegerv(GL_DEPTH_FUNC, &depthFuncMode);
	glCullFace(GL_FRONT);
	glDepthFunc(GL_LEQUAL);

	skyboxShader_.bind();
	skyboxShader_.setUniform("sampleCubeTexture", 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTexture_);

	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO_);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glDrawArrays(GL_QUADS, 0, 24);

	glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
	glDisableVertexAttribArray(0);
	skyboxShader_.release();

	glCullFace(cullFaceMode);
	glDepthFunc(depthFuncMode);
}

void BetterScene::renderFloor() {
	Eigen::Vector2f metaData[4]{
		{1.0, 0.0},
		{1.0, 1.0},
		{0.0, 1.0},
		{0.0, 0.0},
	};

	constexpr auto	Span = 15;
	constexpr auto	N	 = Span * Span * 16;
	Eigen::Vector3f data[N];
	for (int i = 0; i < Span * 2; ++i) {
		for (int j = 0; j < Span * 2; ++j) {
			const auto index  = (i * Span * 2 + j) * 4;
			const auto offset = Eigen::Vector2f(i - Span, j - Span);
			for (int k = 0; k < 4; ++k) {
				data[index + k].block<2, 1>(0, 0) = metaData[k] + offset;
				data[index + k](2)				  = (i & 1) ^ (j & 1) ? 1.0 : 0.0;
			}
		}
	}

	GLuint VBO;
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);

	floorShader_.bind();
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), 0);
	glVertexAttribPointer(
		1, 1, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void*)sizeof(Eigen::Vector2f));
	glDrawArrays(GL_QUADS, 0, N * 3);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	floorShader_.release();

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, &VBO);
}

void BetterScene::setViewMode(ViewMode mode) {
	using namespace Eigen;

	switch (mode) {
		case ViewMode::FPP: {
			const Matrix4f r2cRotation	  = getAxisRotationTransform(-90.0, Vector3f::UnitX());
			const Matrix4f r2cTranslation = getTranslationTransform(-1.6 * Vector3f::UnitY());
			camera_.attach(&role_, r2cTranslation * r2cRotation);
			viewJitter_ = .0;
		} break;
		case ViewMode::TPP: {
			constexpr auto viewDistance = 24.0;
			const Matrix4f r2cRotation	= getAxisRotationTransform(-45.0, Vector3f::UnitX());
			const Matrix4f r2cTranslation =
				getTranslationTransform(-viewDistance * Vector3f::UnitZ());
			camera_.attach(&role_, r2cTranslation * r2cRotation);
			viewJitter_ = .0;
		} break;
	}

	viewMode_ = mode;
}

void BetterScene::computeNextFrame() {
	switch (viewMode_) {
		case ViewMode::FPP: {
			updateFPPTransform();
		} break;
		case ViewMode::TPP: {
			updateTPPTransform();
		} break;
	}
}

void BetterScene::updateFPPTransform() {
	using namespace Eigen;
	role_.moveForward(moveAccumulate_);
	role_.currentDirection += viewRotateAccumulate_(0);
	role_.targetDirection = role_.currentDirection;
	moveAccumulate_.setZero();

	Matrix4f transform = Matrix4f::Identity();

	if (viewRotateAccumulate_(1) == .0 && !lockPitch_) {
		pitch_ *= 0.8;
	}

	pitch_ = std::clamp<float>(pitch_ + viewRotateAccumulate_(1), -30.0, 90.0);
	viewRotateAccumulate_.setZero();

	auto viewJitter = .0;
	if (!moveIndicate_.isZero()) {
		viewJitter_ = fmod(viewJitter_ + M_PI_2 * .1, M_PI * 2.0);
		viewJitter	= sin(viewJitter_) * 1.0;
	} else {
		viewJitter_ = .0;
	}

	auto roleToCameraTransform	  = camera_.roleToCameraTransform;
	camera_.roleToCameraTransform = getAxisRotationTransform(viewJitter, Eigen::Vector3f::UnitZ()) *
									getAxisRotationTransform(-pitch_, Eigen::Vector3f::UnitX()) *
									camera_.roleToCameraTransform;

	transform					  = camera_.getWorld2CameraTransform();
	camera_.roleToCameraTransform = roleToCameraTransform;

	skyboxShader_.setUniform<4, 4>("WVP", transform.data());

	floorShader_.setUniform<4, 4>("WVP", transform.data());
	floorShader_.setUniform(
		"rolePosition", role_.position.x(), role_.position.y(), role_.position.z());

	worldShader_.setUniform<4, 4>("WVP", transform.data());
}

void BetterScene::updateTPPTransform() {
	using namespace Eigen;

	Matrix4f transform = Matrix4f::Identity();

	constexpr auto TPPCameraRotationBase = 90.0;
	TPPCameraRotation_ += viewRotateAccumulate_(0);
	viewRotateAccumulate_.setZero();

	if (!moveIndicate_.isZero()) {
		auto dirNorm = moveIndicate_.normalized();
		auto u		 = Vector3f::UnitX().dot(dirNorm);
		auto v		 = Vector3f::UnitX().cross(dirNorm);
		auto angle	 = qRadiansToDegrees(acos(u));
		auto azimuth = v.z() >= 0. ? angle : 360.0 - angle;
		role_.setTargetDirection(TPPCameraRotation_ + azimuth - TPPCameraRotationBase);
	}

	if (!moveIndicate_.isZero()) {
		viewJitter_ = fmod(viewJitter_ + M_PI_2 * .1, M_PI * 2.0);
	} else {
		viewJitter_ = .0;
	}

	role_.update();

	auto direction		   = role_.currentDirection;
	auto height			   = role_.position.z();
	role_.currentDirection = TPPCameraRotation_;
	role_.position.z()	   = .0;
	role_.moveForward(moveAccumulate_);
	moveAccumulate_.setZero();
	transform			   = camera_.getWorld2CameraTransform();
	role_.currentDirection = direction;
	role_.position.z()	   = height;

	skyboxShader_.setUniform<4, 4>("WVP", transform.data());

	floorShader_.setUniform<4, 4>("WVP", transform.data());
	floorShader_.setUniform(
		"rolePosition", role_.position.x(), role_.position.y(), role_.position.z());

	worldShader_.setUniform<4, 4>("WVP", transform.data());

	updateRoleAnimation();
}

void BetterScene::updateRoleAnimation() {
	using namespace Eigen;

	Matrix4f transform = Matrix4f::Identity();

	std::copy(roleVertices_.begin(), roleVertices_.end(), vertices_.begin());

	const auto headSize			 = Vector3f(.4, .4, .38);
	const auto bodySize			 = Vector3f(.4, .3, .6);
	const auto armSize			 = Vector3f(.09, .09, .72);
	const auto legSize			 = Vector3f(.18, .2, .8);
	const auto armCentroidHeight = legSize.z() + bodySize.z() - .045;

	//! head
	//! body
	//! left arm
	transform = getTranslationTransform(armCentroidHeight * Vector3f::UnitZ()) *
				getAxisRotationTransform(sin(viewJitter_) * 30.0, Vector3f::UnitX()) *
				getTranslationTransform(-armCentroidHeight * Vector3f::UnitZ());
	for (int i = 2 * 8; i < 3 * 8; ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}
	//! right arm
	transform = getTranslationTransform(armCentroidHeight * Vector3f::UnitZ()) *
				getAxisRotationTransform(-sin(viewJitter_) * 30.0, Vector3f::UnitX()) *
				getTranslationTransform(-armCentroidHeight * Vector3f::UnitZ());
	for (int i = 3 * 8; i < 4 * 8; ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}
	//! left leg
	transform = getTranslationTransform(legSize.z() * Vector3f::UnitZ()) *
				getAxisRotationTransform(-sin(viewJitter_) * 40.0, Vector3f::UnitX()) *
				getTranslationTransform(-legSize.z() * Vector3f::UnitZ());
	for (int i = 4 * 8; i < 5 * 8; ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}
	//! right leg
	transform = getTranslationTransform(legSize.z() * Vector3f::UnitZ()) *
				getAxisRotationTransform(sin(viewJitter_) * 40.0, Vector3f::UnitX()) *
				getTranslationTransform(-legSize.z() * Vector3f::UnitZ());
	for (int i = 5 * 8; i < 6 * 8; ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}

	transform = getTranslationTransform(role_.position) *
				getAxisRotationTransform(role_.currentDirection, Vector3f::UnitZ());
	for (int i = 0; i < roleVertices_.size(); ++i) {
		auto vertex				 = Vector4f(.0, .0, .0, 1.0);
		vertex.block<3, 1>(0, 0) = vertices_[i];
		vertices_[i]			 = (transform * vertex).block<3, 1>(0, 0);
	}

	glBindBuffer(GL_ARRAY_BUFFER, VBO_);
	glBufferSubData(GL_ARRAY_BUFFER, 0, roleVertices_.size() * sizeof(Vector3f), vertices_.data());
}

void BetterScene::onIdle() {
	using std::chrono::duration_cast;
	using namespace Eigen;
	static auto idleRec	  = std::chrono::high_resolution_clock::now();
	static auto updateRec = idleRec;

	auto now  = std::chrono::high_resolution_clock::now();
	auto diff = duration_cast<std::chrono::nanoseconds>(now - idleRec).count();
	idleRec	  = now;

	Vector3f moveIndicate = Vector3f::Zero();
	if (!moveIndicate_.isZero()) {
		moveIndicate = moveIndicate_.normalized() * role_.velocity;
	}

	Vector2f viewRotateIndicate = viewRotateIndicate_ * role_.angularVelocity;

	const auto dt = diff * 1e-9;
	moveAccumulate_ += moveIndicate * dt;
	viewRotateAccumulate_ += viewRotateIndicate * dt;

	role_.position.z() = std::max(.0, role_.position.z() + zVelocity_ * dt);
	zVelocity_		   = role_.position.z() == 0. ? 0. : zVelocity_ - gravity_ * dt;

	diff = duration_cast<std::chrono::milliseconds>(now - updateRec).count();
	if (diff > 10) {
		updateRec = now;
		computeNextFrame();
		update();
	}
}
