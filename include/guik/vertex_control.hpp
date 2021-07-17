//unused
#ifndef GUIK_VERTEX_CONTROL_HPP
#define GUIK_VERTEX_CONTROL_HPP

#include <memory>
#include <GL/gl3w.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <guik/camera_control.hpp>
namespace guik {

/**
 * @brief A simple arctic camera control implementation
 *
 */
class VertexControl{
public:

  VertexControl(Eigen::Vector3f pos);

  virtual ~VertexControl();
  void control(Eigen::Vector3f vertex);
  void drag2d(float dragx, float dragy);
  void drag1d(float dragz);
  Eigen::Vector3f generate_3d_location(Eigen::Vector3f old_pos, Eigen::Vector2f new_pos);
private:
  Eigen::Vector3f position;
};

}  // namespace guik

#endif
