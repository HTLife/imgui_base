#include <hdl_graph_slam/graph_edit_window.hpp>

#include <sstream>
#include <g2o/core/optimizable_graph.h>
#include <string>

namespace hdl_graph_slam {
static void help(const char *desc) {
	ImGui::TextDisabled("(?)");
	if (ImGui::IsItemHovered()) {
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
		ImGui::TextUnformatted(desc);
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}
GraphEditWindow::GraphEditWindow(std::shared_ptr<InteractiveGraphView> &graph) :
		graph(graph) {
	param = { -1, //selected vertex
			-1, //dimension
			0.1f, //speed
			0,//select mode map
			hdl_graph_slam::NO_EDITABLE, //edit mode
			ImVec4(0.0f, 1.0f, 1.0f, 0.0f) };
	flags = { false, false, false, false, false, true };
}
GraphEditWindow::~GraphEditWindow() {
}

void GraphEditWindow::draw_ui() {
	if (!flags.show_window) {
		param.edit_mode = hdl_graph_slam::NO_EDITABLE;
		return;
	}

	ImGui::Begin("graph edit", &flags.show_window,
			ImGuiWindowFlags_AlwaysAutoResize);

	ImGuiTabBarFlags flag = ImGuiTabBarFlags_None;
	if (ImGui::BeginTabBar("tabbar", flag)) {
		if (ImGui::BeginTabItem("Fixed vertices")) {
			fixed_vertex.clear();
			g2o::VertexSE3 *anchor_node =
					dynamic_cast<g2o::VertexSE3*>(graph->graph->vertex(
							graph->anchor_node_id()));
			if (anchor_node) {
				bool fixed = anchor_node->fixed();
				if (ImGui::Checkbox("Anchor Fixed", &fixed)) {
					anchor_node->setFixed(fixed);
				}
			} else {
				ImGui::Text("warning: No anchor node created!!");
			}
			bool fixed_vertex_exists = false;
			for (const auto &vertex : graph->graph->vertices()) {
				auto v =
						dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex.second);
				assert(v != nullptr);

				if (!v->fixed() || v->id() == graph->anchor_node_id()) {
					continue;
				}

				fixed_vertex.push_back(v->id());
				std::stringstream sst;
				sst << "Vertex " << v->id();

				if (ImGui::Button(
						("Unfix##" + std::to_string(v->id())).c_str())) {
					v->setFixed(false);
				}
				ImGui::SameLine();
				ImGui::Text("Vertex %d", v->id());
				fixed_vertex_exists = true;
			}

			if (!fixed_vertex_exists) {
				ImGui::Text("No fixed vertices!!");
			}

			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Edit vertices")) {
			//init
            param.mode_map = 0;

			int num_vertices = graph->num_vertices();
			const char *vertices[num_vertices];
			std::string str[num_vertices];
			for (int i = 0; i < num_vertices; i++) {
				str[i] = std::to_string(i);
				vertices[i] = str[i].c_str();
			}

			static int current = 0;
			static int current_dim = 0;
            ImGui::Text("Edit Mode %d", param.edit_mode);
			ImGui::Text("Select vertex");
			ImGui::Combo("##id", &current, vertices, IM_ARRAYSIZE(vertices));
			ImGui::Checkbox("Change color", &flags.color_change);
			if (flags.color_change)
				ImGui::SameLine();
			ImGui::ColorEdit4("##Color", (float*) &param.color,
					ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
			//handle tab -> select adjacent vertex
			if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Tab), false)) {
				if (current == num_vertices - 1)
					current = 0;
				else
					current++;
			}
			//if set vertex action is activated, current selected vertex will be altered by the chosen id
			if (flags.vset) {
				current = param.selected_vertex;
				flags.vset = false;
			} else
				param.selected_vertex = current;
			//if vertex fixed ->show fixed popup
			if (search(current)) {
				g2o::VertexSE3 *node =
						dynamic_cast<g2o::VertexSE3*>(graph->graph->vertex(
								current));
				if (node) {
					ImGui::TextWrapped(
							"Vertex is fixed. Select another vertex or perform actions: Find to focus on the fixed vertex. Unfix to start moving");
					//set center to chosen id
					if (ImGui::Button("Find")) {
						flags.cset = true;
					}
					//unfix chosen id
					ImGui::SameLine();
					if (ImGui::Button("Unfix")) {
						node->setFixed(false);
						set_fixed_vertex(current, 0);
					}
					ImGui::SameLine();
					ImGui::Checkbox("Focus", &flags.cfocus);
					param.selected_vertex = current;
				}

				else {
					ImGui::TextWrapped(
							"This is vertex plane. Select another vertex to proceed");
				}

			} else { //vertex not fixed -> show normal popup
				static float f0 = 0.1f;
				ImGui::Text("Adjust speed");
				ImGui::InputFloat("##Speed", &f0, 0.01f, 1.0f, "%.3f");
				ImGui::Text("Dragging edit.");
				ImGui::SameLine();
				help(
						"Hold Z, drag left-right to adjust X axis. \nHold X, drag left-right to adjust Y axis. \nHold C, drag up-down to adjust Z axis");
				if (f0 < 0.01)
					f0 = 0.01;
				param.speed = f0;
				//handle zxc (dragging) control
				static const char *dragging_content = dragging_start;

				if (ImGui::Button(dragging_content)) {
					if (dragging_content == dragging_start) //start
					    {
						dragging_content = dragging_end;
						current_dim = 0;
						param.edit_mode = hdl_graph_slam::MOUSE_TRANSLATION;
					} else //end
					{
						dragging_content = dragging_start;
						param.edit_mode = hdl_graph_slam::NO_EDITABLE;
					}
				}
				//handle arrow control
				ImGui::Text("Arrow edit.");
				ImGui::SameLine();
				help("Select dimension and press arrow keys or on-screen arrows.");
				if (param.edit_mode == hdl_graph_slam::KEYBOARD_ROTATION) {
                    const char *dimension[] = { "None", "X", "Y", "Z" };
                    ImGui::Combo("##Axes", &current_dim, dimension,
                                 IM_ARRAYSIZE(dimension));
				}
				else {
                    const char *dimension[] = { "None", "XY", "Z" };
                    ImGui::Combo("##Axes", &current_dim, dimension,
                                 IM_ARRAYSIZE(dimension));
				}

				if (current_dim == 0) {
					param.edit_dimension = -1;
				} else {
					if (current_dim == 1) {
						ImGui::Checkbox("Lock XY axis", &flags.xylock);
					}
					if (param.edit_mode == hdl_graph_slam::MOUSE_TRANSLATION)
						dragging_content = dragging_start;
					else {
                        param.edit_dimension = current_dim - 1;
					}
				}
				//find/focus on vertex
				if (ImGui::Button("Find")) {
					flags.cset = true;
				}
				ImGui::SameLine();
				if (ImGui::Button("Fix")) {
					g2o::VertexSE3 *node =
							dynamic_cast<g2o::VertexSE3*>(graph->graph->vertex(
									current));
					node->setFixed(true);
					set_fixed_vertex(current, 1);
				}
				ImGui::SameLine();
				ImGui::Checkbox("Focus", &flags.cfocus);
				ImGui::TextWrapped(
						"Right click on the vertex to fix and select a new one.");
			}
			ImGui::EndTabItem();
		}
        if (ImGui::BeginTabItem("Edit map")) {
            //init
            param.mode_map = 1;

            int num_vertices = graph->num_vertices();
            const char *vertices[num_vertices];
            std::string str[num_vertices];
            for (int i = 0; i < num_vertices; i++) {
                str[i] = std::to_string(i);
                vertices[i] = str[i].c_str();
            }

            static int current = 0;
            static int current_dim = 0;
            ImGui::Text("Edit Mode %d", param.edit_mode);
            ImGui::Text("Select vertex");
            ImGui::Combo("##id", &current, vertices, IM_ARRAYSIZE(vertices));
            ImGui::Checkbox("Change color", &flags.color_change);
            if (flags.color_change)
                ImGui::SameLine();
            ImGui::ColorEdit4("##Color", (float*) &param.color,
                              ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
            //handle tab -> select adjacent vertex
            if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Tab), false)) {
                if (current == num_vertices - 1)
                    current = 0;
                else
                    current++;
            }
            //if set vertex action is activated, current selected vertex will be altered by the chosen id
            if (flags.vset) {
                current = param.selected_vertex;
                flags.vset = false;
            } else
                param.selected_vertex = current;
            //if vertex fixed ->show fixed popup
            if (search(current)) {
                g2o::VertexSE3 *node =
                        dynamic_cast<g2o::VertexSE3*>(graph->graph->vertex(
                                current));
                if (node) {
                    ImGui::TextWrapped(
                            "Vertex is fixed. Select another vertex or perform actions: Find to focus on the fixed vertex. Unfix to start moving");
                    //set center to chosen id
                    if (ImGui::Button("Find")) {
                        flags.cset = true;
                    }
                    //unfix chosen id
                    ImGui::SameLine();
                    if (ImGui::Button("Unfix")) {
                        node->setFixed(false);
                        set_fixed_vertex(current, 0);
                    }
                    ImGui::SameLine();
                    ImGui::Checkbox("Focus", &flags.cfocus);
                    param.selected_vertex = current;
                }

                else {
                    ImGui::TextWrapped(
                            "This is vertex plane. Select another vertex to proceed");
                }

            } else { //vertex not fixed -> show normal popup
                static float f0 = 0.1f;
                ImGui::Text("Adjust speed");
                ImGui::InputFloat("##Speed", &f0, 0.01f, 1.0f, "%.3f");
                ImGui::Text("Dragging edit.");
                ImGui::SameLine();
                help(
                        "Hold Z, drag left-right to adjust X axis. \nHold X, drag left-right to adjust Y axis. \nHold C, drag up-down to adjust Z axis");
                if (f0 < 0.01)
                    f0 = 0.01;
                param.speed = f0;
                //handle zxc (dragging) control
                static const char *dragging_content = dragging_start;

                if (ImGui::Button(dragging_content)) {
                    if (dragging_content == dragging_start) //start
                    {
                        dragging_content = dragging_end;
                        current_dim = 0;
                        param.edit_mode = hdl_graph_slam::MOUSE_TRANSLATION;
                    } else //end
                    {
                        dragging_content = dragging_start;
                        param.edit_mode = hdl_graph_slam::NO_EDITABLE;
                    }
                }
                //handle arrow control
                ImGui::Text("Arrow edit.");
                ImGui::SameLine();
                help("Select dimension and press arrow keys or on-screen arrows.");
                if (param.edit_mode == hdl_graph_slam::KEYBOARD_ROTATION) {
                    const char *dimension[] = { "None", "X", "Y", "Z" };
                    ImGui::Combo("##Axes", &current_dim, dimension,
                                 IM_ARRAYSIZE(dimension));
                }
                else {
                    const char *dimension[] = { "None", "XY", "Z" };
                    ImGui::Combo("##Axes", &current_dim, dimension,
                                 IM_ARRAYSIZE(dimension));
                }

                if (current_dim == 0) {
                    param.edit_dimension = -1;
                } else {
                    if (current_dim == 1) {
                        ImGui::Checkbox("Lock XY axis", &flags.xylock);
                    }
                    if (param.edit_mode == hdl_graph_slam::MOUSE_TRANSLATION)
                        dragging_content = dragging_start;
                    else {
                        param.edit_dimension = current_dim - 1;
                    }
                }
                //find/focus on vertex
                if (ImGui::Button("Find")) {
                    flags.cset = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("Fix")) {
                    g2o::VertexSE3 *node =
                            dynamic_cast<g2o::VertexSE3*>(graph->graph->vertex(
                                    current));
                    node->setFixed(true);
                    set_fixed_vertex(current, 1);
                }
                ImGui::SameLine();
                ImGui::Checkbox("Focus", &flags.cfocus);
                ImGui::TextWrapped(
                        "Right click on the vertex to fix and select a new one.");
            }
            ImGui::EndTabItem();
        }
		ImGui::EndTabBar();
	}

	ImGui::End();
}

}  // namespace hdl_graph_slam
