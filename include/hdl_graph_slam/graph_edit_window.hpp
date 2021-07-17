#ifndef HDL_GRAPH_SLAM_GRAPH_EDIT_WINDOW_HPP
#define HDL_GRAPH_SLAM_GRAPH_EDIT_WINDOW_HPP

#include <memory>

#include <imgui.h>
#include <guik/gl_canvas.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>

namespace hdl_graph_slam {
enum edit_mode {
    MOUSE_TRANSLATION=0,
    KEYBOARD_TRANSLATION=1,
    KEYBOARD_ROTATION=2,
    NO_EDITABLE=-1
};

struct GraphEditParam {
public:
	int selected_vertex;
	int edit_dimension;
	float speed;
    int mode_map;
	int edit_mode;
	ImVec4 color;
};

struct GraphEditFlags {
public:
	bool xylock;
	bool vset;
	bool cset;
	bool cfocus;
	bool show_window;
	bool color_change;
};
class GraphEditWindow {
public:
	explicit GraphEditWindow(std::shared_ptr<InteractiveGraphView> &graph);
	~GraphEditWindow();

	void set_edit_mode(int new_edit_mode) {
	    param.edit_mode = new_edit_mode;
	}

	void show() {
		flags.show_window = true;
	}
	bool is_active() const {
		return flags.show_window;
	}
	bool is_center_changed() const {
		return flags.cset;
	}
	bool is_locked() const{
		return flags.xylock;
	}
	void center_changed() {
		flags.cset = false;
	}
	bool is_center_focus() const {
		return flags.cfocus;
	}
	bool is_color_changed() const{
		return flags.color_change;
	}
	GraphEditParam get_edit_parameter() {
		return param;
	}
	void set_vertex(int vertex) {
		flags.vset = true;
		param.selected_vertex = vertex;
	}
	void set_fixed_vertex(int vertex, int condition) {
		if (condition == 0) //fixed to non
			fixed_vertex.erase(
					std::remove(fixed_vertex.begin(), fixed_vertex.end(),
							vertex), fixed_vertex.end());
		else //non-fixed to fixed
		{
			std::vector<int>::iterator it = std::lower_bound(
					fixed_vertex.begin(), fixed_vertex.end(), vertex,
					std::greater<int>());
			fixed_vertex.insert(it, vertex);
		}
	}
	bool search(int vertex) { //actually a binary search for fixed vertex
		size_t mid, left = 0;
		size_t right = fixed_vertex.size();
		while (left < right) {
			mid = left + (right - left) / 2;
			if (vertex < fixed_vertex[mid]) {
				left = mid + 1;
			} else if (vertex > fixed_vertex[mid]) {
				right = mid;
			} else {
				return true;
			}
		}
		return false;
	}
	void draw_ui();

private:
	GraphEditParam param;
	GraphEditFlags flags;
	const char *dragging_start = "Dragging start";
	const char *dragging_end = "Dragging end";
	std::vector<int> fixed_vertex;
	std::shared_ptr<InteractiveGraphView> &graph;
};
}  // namespace hdl_graph_slam

#endif
