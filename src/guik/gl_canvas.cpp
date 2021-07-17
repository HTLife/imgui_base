#include <guik/gl_canvas.hpp>

#include <iostream>
#include <boost/format.hpp>
#include <imgui.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>

#include <guik/camera_control.hpp>

#include <GL/gl3w.h>
#include <GL/gl.h>

namespace guik {

/**
 * @brief Construct a new GLCanvas object
 *
 * @param data_directory
 * @param size
 */
GLCanvas::GLCanvas(const std::string& data_directory, const Eigen::Vector2i& size) : size(size), point_size(50.0f), min_z(-1.5f), max_z(5.0f), z_clipping(true) {
  frame_buffer.reset(new glk::FrameBuffer(size));
  frame_buffer->add_color_buffer(GL_RGBA32I, GL_RGBA_INTEGER, GL_INT);
  shader.reset(new glk::GLSLShader());
  if (!shader->init(data_directory + "/shader/rainbow")) {
    shader.reset();
    return;
  }

  shader->use();

  camera_control.reset(new guik::ArcCameraControl());
  projection_control.reset(new guik::ProjectionControl(size));
  texture_renderer.reset(new glk::TextureRenderer(data_directory));
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool GLCanvas::ready() const { return frame_buffer && shader && camera_control && texture_renderer; }

void GLCanvas::reset_camera() {
  camera_control.reset(new guik::ArcCameraControl());
  projection_control.reset(new guik::ProjectionControl(size));
}

void GLCanvas::auto_camera(hdl_graph_slam::Box3D &box) {
  double center_x = (box.max.x - box.min.x) / 2 + box.min.x;
  double center_y = (box.max.y - box.min.y) / 2 + box.min.y;
  double param = 15;
  double radian = param * 3.14159265 / 180.0;
  double width = std::max(box.max.x - box.min.x, box.max.y - box.min.y);
  double z_height = ((width) / 2) / tan(radian) + box.max.z;

  auto cameraControl = new guik::ArcCameraControl();
  Eigen::Vector3f v(3);
  v << center_x, center_y, 0;
  cameraControl->set_center(v);
  camera_control.reset(cameraControl);
  projection_control.reset(new guik::ProjectionControl(size));
}

/**
 * @brief Set the Size object
 *
 * @param size
 */
void GLCanvas::set_size(const Eigen::Vector2i& size) {
  this->size = size;
  projection_control->set_size(size);
  frame_buffer.reset(new glk::FrameBuffer(size));
  frame_buffer->add_color_buffer(GL_RGBA32I, GL_RGBA_INTEGER, GL_INT);
}

std::string GLCanvas::get_angle()
{
	std::stringstream sst;
	double* angle_ = camera_control->get_rotation();
	//Eigen::Vector2f* corr_ = camera_control->get_mouse_position();

	sst << "Angle spec: \n";
	sst << boost::format("# theta: %f") % angle_[0] << "\n";
	sst << boost::format("# phi: %f") % angle_[1] << "\n";
	//sst << boost::format("xVec: %f") % corr_[0] << "\n";
	//sst << boost::format("yVec: %f") % corr_[1] << "\n";
	//sst << boost::format("z: %f") % corr_[2] << "\n";
	std::string stats = sst.str();
	delete angle_;
	//delete corr_;
	return stats;
}
/**
 * @brief
 *
 */
void GLCanvas::bind(bool clear_buffers) {
  frame_buffer->bind();
  glDisable(GL_SCISSOR_TEST);
  if(clear_buffers) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  GLint clear_color[] = {0, -1, 0, 0};
  glClearTexImage(frame_buffer->color(1).id(), 0, GL_RGBA_INTEGER, GL_INT, clear_color);

  shader->use();

  Eigen::Matrix4f view_matrix = camera_control->view_matrix();
  Eigen::Matrix4f projection_matrix = projection_control->projection_matrix();

  shader->set_uniform("view_matrix", view_matrix);
  shader->set_uniform("projection_matrix", projection_matrix);
  shader->set_uniform("z_clipping", z_clipping ? 1 : 0);
  shader->set_uniform("z_range", Eigen::Vector2f(min_z, max_z));

  shader->set_uniform("color_mode", 1);
  shader->set_uniform("point_scale", 1.0f);
  shader->set_uniform("point_size", point_size);

  glEnable(GL_DEPTH_TEST);
}

/**
 * @brief
 *
 */
void GLCanvas::unbind() {
  glDisable(GL_DEPTH_TEST);
  glFlush();

  frame_buffer->unbind();
}

/**
 * @brief
 *
 */
void GLCanvas::render_to_screen(int color_buffer_id) { texture_renderer->draw(frame_buffer->color(color_buffer_id).id()); }

/**
 * @brief mouse control
 * handling left, middle, dragging, scrolling
 *
 */
void GLCanvas::mouse_control(bool focus) {
  ImGuiIO& io = ImGui::GetIO();
  auto mouse_pos = ImGui::GetMousePos();
  auto drag_delta = ImGui::GetMouseDragDelta();

  Eigen::Vector2i p(mouse_pos.x, mouse_pos.y);

  for (int i = 0; i < 3; i++) {
    if (ImGui::IsMouseClicked(i)) {
    	camera_control->mouse(p, i, true);
    }
    if (ImGui::IsMouseReleased(i)) {
    	camera_control->mouse(p, i, false);
    }
    if (ImGui::IsMouseDragging(i)) {
    	camera_control->drag(p, i, focus);
    }
    camera_control->scroll(Eigen::Vector2f(io.MouseWheel, io.MouseWheelH));
  }
}

/**
 * @brief
 *
 * @param p
 * @param window
 * @return Eigen::Vector4i
 */
Eigen::Vector4i GLCanvas::pick_info(const Eigen::Vector2i& p, int window) const {
  if (p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
    return Eigen::Vector4i(-1, -1, -1, -1);
  }

  std::vector<int> pixels = frame_buffer->color(1).read_pixels<int>(GL_RGBA_INTEGER, GL_INT);

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

  for (int i = -window; i <= window; i++) {
    for (int j = -window; j <= window; j++) {
      ps.push_back(Eigen::Vector2i(i, j));
    }
  }

  std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
  for (int i = 0; i < ps.size(); i++) {
    Eigen::Vector2i p_ = p + ps[i];
    int index = ((size[1] - p[1]) * size[0] + p_[0]) * 4;
    Eigen::Vector4i info = Eigen::Map<Eigen::Vector4i>(&pixels[index]);

    if (info[3] >= 0) {
      return info;
    }
  }

  return Eigen::Vector4i(-1, -1, -1, -1);
}

/**
 * @brief
 *
 * @param p
 * @param window
 * @return float
 */
float GLCanvas::pick_depth(const Eigen::Vector2i& p, int window) const {
  if (p[0] < 5 || p[1] < 5 || p[0] > size[0] - 5 || p[1] > size[1] - 5) {
    return -1.0f;
  }

  std::vector<float> pixels = frame_buffer->depth().read_pixels<float>(GL_DEPTH_COMPONENT, GL_FLOAT);

  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

  for (int i = -window; i <= window; i++) {
    for (int j = -window; j <= window; j++) {
      ps.push_back(Eigen::Vector2i(i, j));
    }
  }

  std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
  for (int i = 0; i < ps.size(); i++) {
    Eigen::Vector2i p_ = p + ps[i];
    int index = ((size[1] - p[1]) * size[0] + p_[0]);
    float depth = pixels[index];

    if (depth < 1.0f) {
      return depth;
    }
  }

  return 1.0f;
}

#include <math.h>
int GLCanvas::pick_axis(Eigen::Vector2i dragAmount){
	double* angle_ = camera_control->get_rotation();
	Eigen::Vector2f* corr_ = camera_control->get_mouse_position();
	double angleX = acuteAngle(dragAmount, corr_[0]);
	double angleY = acuteAngle(dragAmount, corr_[1]);
	std::cout << angleX << " " << angleY << std::endl;
	if (angleX < angleY) return 0; else return 1;
}
double GLCanvas::acuteAngle(Eigen::Vector2i v1, Eigen::Vector2f v2){
	double angle = atan2(v1[0]*v2[1]-v1[1]*v2[0],v1[0]*v2[0]+v1[1]*v2[1]);
	std::cout << angle << std::endl;
	std::cout << abs(angle) << " " << M_PI - abs(angle) << std::endl;
	if (abs(angle) >= M_PI_2) return M_PI - abs(angle); else return abs(angle);
}
/**
 * @brief
 *
 * @param p
 * @param depth
 * @return Eigen::Vector3f
 */
Eigen::Vector3f GLCanvas::unproject(const Eigen::Vector2i& p, float depth) const {
  Eigen::Matrix4f view_matrix = camera_control->view_matrix();
  glm::mat4 view = glm::make_mat4(view_matrix.data());

  Eigen::Matrix4f projection_matrix = projection_control->projection_matrix();
  glm::mat4 projection = glm::make_mat4(projection_matrix.data());

  glm::vec4 viewport = glm::vec4(0, 0, size[0], size[1]);
  glm::vec3 wincoord = glm::vec3(p[0], size[1] - p[1], depth);
  glm::vec3 objcoord = glm::unProject(wincoord, view, projection, viewport);

  return Eigen::Vector3f(objcoord.x, objcoord.y, objcoord.z);
}

void GLCanvas::draw_ui() {
  ImGui::Begin("shader setting", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::DragFloat("point_size", &point_size, 10.0f);
  ImGui::DragFloat("min_z", &min_z, 0.1f);
  ImGui::DragFloat("max_z", &max_z, 0.1f);
  ImGui::DragFloat("vertex_size", &vertex_size, 0.1f, 0.01f, 10.0f);
  ImGui::Checkbox("z_clipping", &z_clipping);

  ImGui::Separator();
  ImGui::Text("Export PCD Settings");
  ImGui::DragInt("skip frames", &export_skip_frames, 1, 1, 100);
  ImGui::DragFloat("voxel leaf size (x): ", &voxel_leaf_sizes[0], 0.001f, 0.001f, 10.0f);
  ImGui::DragFloat("voxel leaf size (y): ", &voxel_leaf_sizes[1], 0.001f, 0.001f, 10.0f );
  ImGui::DragFloat("voxel leaf size (z): ", &voxel_leaf_sizes[2], 0.001f, 0.001f, 10.0f);
  ImGui::Checkbox("apply to current view", &apply_voxelization);
  ImGui::End();

  projection_control->draw_ui();
}

int GLCanvas::get_export_skip_frames() {
    return export_skip_frames;
}

float *GLCanvas::get_voxel_leaf_sizes() {
    return voxel_leaf_sizes;
}

bool GLCanvas::is_apply_voxelization() {
    return apply_voxelization;
}

void GLCanvas::show_projection_setting() { projection_control->show(); }

float GLCanvas::get_vertex_size() {
    return vertex_size;
}

}  // namespace guik


