#ifndef HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_VIEW_HPP
#define HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_VIEW_HPP

#include <mutex>
#include <unordered_map>
#include <glk/glsl_shader.hpp>

#include <hdl_graph_slam/interactive_graph.hpp>

#include <g2o/core/factory.h>

#include <hdl_graph_slam/view/edge_view.hpp>
#include <hdl_graph_slam/view/vertex_view.hpp>
#include <hdl_graph_slam/view/keyframe_view.hpp>
#include <hdl_graph_slam/view/line_buffer.hpp>
#include <hdl_graph_slam/view/drawable_object.hpp>

namespace hdl_graph_slam {

class InteractiveGraphView : public InteractiveGraph {
public:
  InteractiveGraphView() { }
  virtual ~InteractiveGraphView() override {}

  void init_gl() { line_buffer.reset(new LineBuffer()); }

  void update_view_pointcloud(float *voxel_leaf_sizes) {
      for (auto& drawable : drawables) {
          if (drawable->available()) {
              drawable->voxelize(voxel_leaf_sizes);
          }
      }
  }

  void set_vertex_size(float vertex_size) {
      for (auto& drawable : drawables) {
          if (drawable->available()) {
              drawable->set_vertex_size(vertex_size);
          }
      }
  }

  void update_view() {
    bool keyframe_inserted = false;
    for (const auto& key_item : keyframes) {
      auto& keyframe = key_item.second;
      auto found = keyframes_view_map.find(keyframe);
      if (found == keyframes_view_map.end()) {
        keyframe_inserted = true;
        keyframes_view.push_back(std::make_shared<KeyFrameView>(keyframe));
        keyframes_view_map[keyframe] = keyframes_view.back();

        vertices_view.push_back(keyframes_view.back());
        vertices_view_map[keyframe->id()] = keyframes_view.back();

        drawables.push_back(keyframes_view.back());
      }
    }

    if (keyframe_inserted) {
      std::sort(keyframes_view.begin(), keyframes_view.end(), [=](const KeyFrameView::Ptr& lhs, const KeyFrameView::Ptr& rhs) { return lhs->lock()->id() < rhs->lock()->id(); });
    }

    for (const auto& vertex : graph->vertices()) {
      auto found = vertices_view_map.find(vertex.second->id());
      if (found != vertices_view_map.end()) {
        continue;
      }

      auto vertex_view = VertexView::create(vertex.second);
      if (vertex_view) {
        vertices_view.push_back(vertex_view);
        vertices_view_map[vertex.second->id()] = vertex_view;

        drawables.push_back(vertex_view);
      }
    }

    for (const auto& edge : graph->edges()) {
      auto found = edges_view_map.find(edge);
      if (found != edges_view_map.end()) {
        continue;
      }

      auto edge_view = EdgeView::create(edge, *line_buffer);
      if (edge_view) {
        edges_view.push_back(edge_view);
        edges_view_map[edge] = edge_view;

        drawables.push_back(edge_view);
      }
    }
  }

  void draw(const DrawFlags& flags, glk::GLSLShader& shader) {
    update_view();
    line_buffer->clear();
    for (auto& drawable : drawables) {
      if (drawable->available()) {
        drawable->draw(flags, shader);
      }
    }

    line_buffer->draw(shader);
  }

  void draw(const DrawFlags& flags, glk::GLSLShader& shader, int id, Eigen::Vector4f color, float object_width){
	  update_view();
	  line_buffer->clear();
	  for (auto& drawable : drawables){
		  if (drawable->available()){
		      drawable->set_vertex_size(object_width);
			  drawable->draw(flags, shader, id, color);
		  }
	  }

	  line_buffer->draw(shader, object_width);
  }

  void delete_edge(EdgeView::Ptr edge) {
    std::cout << "delete edge " << edge->id() << std::endl;

    auto found = std::find(edges_view.begin(), edges_view.end(), edge);
    while(found != edges_view.end()) {
        edges_view.erase(found);
        found = std::find(edges_view.begin(), edges_view.end(), edge);
    }

    auto found2 = std::find(drawables.begin(), drawables.end(), edge);
    while(found2 != drawables.end()) {
      drawables.erase(found2);
      found2 = std::find(drawables.begin(), drawables.end(), edge);
    }

    for(auto itr = edges_view_map.begin(); itr != edges_view_map.end(); itr++) {
      if(itr->second == edge) {
        edges_view_map.erase(itr);
        break;
      }
    }
    graph->removeEdge(edge->edge);
  }

    void delete_vertex(VertexView::Ptr vertex) {
        std::cout << "delete vertex " << vertex->id() << std::endl;
        g2o::Factory *factory = g2o::Factory::instance();
        std::string type_vertex = factory->tag(graph->vertex(vertex->id())).c_str();

        auto found2 = std::find(drawables.begin(), drawables.end(), vertex);
        while(found2 != drawables.end()) {
            drawables.erase(found2);
            found2 = std::find(drawables.begin(), drawables.end(), vertex);

        }
        auto found3 = std::find(keyframes_view.begin(), keyframes_view.end(), vertex);
        while(found3 != keyframes_view.end()) {
            keyframes_view.erase(found3);
            found3 = std::find(keyframes_view.begin(), keyframes_view.end(), vertex);

        }

        if (type_vertex != "VERTEX_PLANE"){
            for(auto itr = vertices_view_map.begin(); itr != vertices_view_map.end(); itr++) {
                if(itr->second == vertex) {
                    auto check2 = itr;
                    vertices_view_map.erase(itr);
                    break;
                }
            }

            for(auto itr = keyframes.begin(); itr != keyframes.end(); itr++) {
                if(itr->first == vertex->id()) {
                    keyframes.erase(itr);
                    break;
                }
            }

            graph->removeVertex(vertex->get_vertex());
        }
    }
public:
  std::unique_ptr<LineBuffer> line_buffer;

  std::vector<KeyFrameView::Ptr> keyframes_view;
  std::unordered_map<InteractiveKeyFrame::Ptr, KeyFrameView::Ptr> keyframes_view_map;

  std::vector<VertexView::Ptr> vertices_view;
  std::unordered_map<long, VertexView::Ptr> vertices_view_map;

  std::vector<EdgeView::Ptr> edges_view;
  std::unordered_map<g2o::HyperGraph::Edge*, EdgeView::Ptr> edges_view_map;

  std::vector<DrawableObject::Ptr> drawables;

};

}  // namespace hdl_graph_slam

#endif
