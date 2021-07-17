#ifndef GLK_CAMERA_CONTROL_HPP
#define GLK_CAMERA_CONTROL_HPP

#include <memory>
#include <GL/gl3w.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace guik {

/**
 * @brief A class to contol camera position with mouse
 *
 */
class CameraControl {
public:
  virtual ~CameraControl() {}

  /** @brief mouse button callback */
  virtual void mouse(const Eigen::Vector2i& p, int button, bool down) = 0;

  /** @brief mouse dragging callback */
  virtual void drag(const Eigen::Vector2i& p, int button, bool focus) = 0;

  /** @brief mouse scroll callback */
  virtual void scroll(const Eigen::Vector2f& rel) = 0;
  virtual void set_center(const Eigen::Vector3f pos) = 0;

  virtual double* get_rotation() = 0;
  virtual Eigen::Vector2f* get_mouse_position() = 0;
  /** @brief camera view matrix */
  virtual Eigen::Matrix4f view_matrix() const = 0;
};

/**
 * @brief A simple arctic camera control implementation
 *
 */
class ArcCameraControl : public CameraControl {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ArcCameraControl();

  virtual ~ArcCameraControl() override;

  void mouse(const Eigen::Vector2i& p, int button, bool down) override;
  void drag(const Eigen::Vector2i& p, int button, bool focus) override;
  void scroll(const Eigen::Vector2f& rel) override;
  void set_center(const Eigen::Vector3f pos) override;

  double* get_rotation();
  Eigen::Vector2f* get_mouse_position();
  Eigen::Quaternionf rotation() const;
  Eigen::Matrix4f view_matrix() const override;

private:
  Eigen::Vector3f center;
  double distance;

  Eigen::Vector2i drag_last_pos;

  bool left_button_down;
  double theta;
  double phi;
  Eigen::Vector2f xVec;
  Eigen::Vector2f yVec;
  bool middle_button_down;
};

}  // namespace guik

#endif
