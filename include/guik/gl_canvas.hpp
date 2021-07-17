#ifndef GLK_GL_CANVAS_CANVAS_HPP
#define GLK_GL_CANVAS_CANVAS_HPP

#include <imgui.h>

#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>

#include <guik/camera_control.hpp>
#include <guik/projection_control.hpp>

#include <hdl_graph_slam/graph_slam.hpp>

namespace guik {

/**
 * @brief OpenGL canvas for imgui
 *
 */
class GLCanvas {
public:
  GLCanvas(const std::string& data_directory, const Eigen::Vector2i& size);

  bool ready() const;

  void reset_camera();
  void auto_camera(hdl_graph_slam::Box3D &box);

  void set_size(const Eigen::Vector2i& size);
  std::string get_angle();
  void mouse_control(bool focus);

  void bind(bool clear_buffers = true);
  void unbind();

  void render_to_screen(int color_buffer_id = 0);

  Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window = 2) const;
  float pick_depth(const Eigen::Vector2i& p, int window = 2) const;
  Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const;
  int pick_axis(Eigen::Vector2i dragAmount);
  double acuteAngle(Eigen::Vector2i v1, Eigen::Vector2f v2);
  void draw_ui();
  void show_projection_setting();
  int get_export_skip_frames();
  float *get_voxel_leaf_sizes();
  bool is_apply_voxelization();
  float get_vertex_size();

public:
  Eigen::Vector2i size;
  std::unique_ptr<glk::GLSLShader> shader;
  std::unique_ptr<glk::FrameBuffer> frame_buffer;
  std::unique_ptr<glk::TextureRenderer> texture_renderer;

  std::unique_ptr<guik::CameraControl> camera_control;
  std::unique_ptr<guik::ProjectionControl> projection_control;

private:
  float point_size;
  float min_z;
  float max_z;
  bool z_clipping;
  int export_skip_frames=1;
  float voxel_leaf_sizes[3] = {0.01f, 0.01f, 0.01f};
  bool apply_voxelization = false;
  float vertex_size = 1;
};

}  // namespace guik

#endif
