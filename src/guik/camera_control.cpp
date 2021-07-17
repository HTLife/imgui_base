#include <guik/camera_control.hpp>
#include <math.h>

#include <memory>
#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>

#include <imgui.h>
#include <imgui_internal.h>
namespace guik {

ArcCameraControl::ArcCameraControl() :
		center(0.0f, 0.0f, 0.0f), distance(10.0f), left_button_down(false), theta(
				0.0f), phi(-89.0f * M_PI / 180.0f), xVec(0.0f, 0.0f), yVec(0.0f,
				0.0f) {
	left_button_down = false;
	middle_button_down = false;
}

ArcCameraControl::~ArcCameraControl() {
}

void ArcCameraControl::mouse(const Eigen::Vector2i &p, int button, bool down) {
	if (button == 0) {
		left_button_down = down;
	}
	if (button == 2) {
		middle_button_down = down;
	}
	drag_last_pos = p;
}

void ArcCameraControl::drag(const Eigen::Vector2i &p, int button, bool focus) {
	Eigen::Vector2i rel = p - drag_last_pos;

	if (left_button_down) {
		theta -= rel[0] * 0.005f;
		phi -= rel[1] * 0.005f;

		phi = (phi < -0.01) ?
				((phi >= -M_PI_2 + 0.01) ? phi : -M_PI_2 + 0.01) : -0.01;
		theta = (theta > M_PI - 0.01) ? theta - 2 * M_PI :
				(theta < -M_PI + 0.01) ? theta + 2 * M_PI : theta;

		/*
		 double newX = (1/cos(theta))/cos((M_PI_2 - theta)*(M_PI_2 + phi)/M_PI_2);
		 double newY = (1/cos(theta))/cos(theta*(M_PI_2 + phi)/M_PI_2);
		 double angleX = (M_PI_2 - theta)*phi/M_PI_2;
		 double angleY = theta*phi/M_PI_2;
		 xVec = Eigen::Vector2f(1, cos(angleX)/sin(angleX));
		 yVec = Eigen::Vector2f(-sin(angleY)/cos(angleY), 1);*/
	}

	if (middle_button_down && focus == false) {
		center += Eigen::AngleAxisf(theta + M_PI_2, Eigen::Vector3f::UnitZ())
				* Eigen::Vector3f(-rel[0], rel[1], 0.0f) * distance * 0.001f;
	}

	drag_last_pos = p;
}

void ArcCameraControl::scroll(const Eigen::Vector2f &rel) {
	if (rel[0] > 0) {
		distance = distance * 0.9f;
	} else if (rel[0] < 0) {
		distance = distance * 1.1f;
	}

	distance = std::max(0.1, distance);
}

void ArcCameraControl::set_center(const Eigen::Vector3f pos){
	center = pos;
	distance = 10.0f;
}

Eigen::Quaternionf ArcCameraControl::rotation() const {
	return Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY());
}

Eigen::Matrix4f ArcCameraControl::view_matrix() const {
	Eigen::Vector3f offset = rotation() * Eigen::Vector3f(distance, 0.0f, 0.0f);
	Eigen::Vector3f eye = center + offset;

	glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]),
			glm::vec3(center[0], center[1], center[2]),
			glm::vec3(0.0f, 0.0f, 1.0f));
	return Eigen::Map < Eigen::Matrix4f > (glm::value_ptr(mat));
}

double* ArcCameraControl::get_rotation() {
	double *angle_ = new double[2];
	angle_[0] = theta;
	angle_[1] = phi;
	return angle_;
}
Eigen::Vector2f* ArcCameraControl::get_mouse_position() {
	Eigen::Vector2f *corr = new Eigen::Vector2f[2];
	corr[0] = xVec;
	corr[1] = yVec;
	return corr;
}
}  // namespace guik
