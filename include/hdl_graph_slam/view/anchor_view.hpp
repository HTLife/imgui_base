#ifndef HDL_GRAPH_SLAM_VIEW_ANCHOR_VIEW_HPP
#define HDL_GRAPH_SLAM_VIEW_ANCHOR_VIEW_HPP

#include <Eigen/Core>
#include <g2o/core/hyper_graph.h>

#include <glk/glsl_shader.hpp>
#include <hdl_graph_slam/view/drawable_object.hpp>
#include <glk/primitives/primitives.hpp>
#include <pcl/common/eigen.h>


namespace hdl_graph_slam {

struct Anchor {
public:
    float x;
    float y;
    float z;

    Anchor(float x_, float y_, float z_) {
        x = x_;
        y = y_;
        z = z_;
    }
};

class AnchorView : public DrawableObject {
public:
  using Ptr = std::shared_ptr<AnchorView>;

  AnchorView(std::vector<Anchor> _anchors) {
      anchors = _anchors;
  };
  virtual ~AnchorView() {};

  virtual void draw(const DrawFlags &flags, glk::GLSLShader &shader)
    override {
  int count = 0;
    for (auto const &anchor : anchors) {
        shader.set_uniform("color_mode", 1);
        shader.set_uniform("material_color",
                           Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f));

        shader.set_uniform("info_values",
                           Eigen::Vector4i(VERTEX | KEYFRAME, count, 0, 0));
        Eigen::Matrix4f model_matrix = pcl::getTransformation(anchor.x, anchor.y, anchor.z, 0.0f, 0.0f, 0.0f).matrix();
        model_matrix.block<3, 3>(0, 0) *= 0.65;

        shader.set_uniform("model_matrix", model_matrix);

        const auto &sphere = glk::Primitives::instance()->primitive(
                glk::Primitives::SPHERE);
        sphere.draw(shader);
        count++;
    }
  }
  void update(std::vector<Anchor> &anchors_) {
      anchors = std::move(anchors_);
  }

private:
    AnchorView();

private:
    std::vector<Anchor> anchors;

};

}  // namespace hdl_graph_slam

#endif
