#include <hdl_graph_slam/interactive_graph.hpp>

#include <chrono>
#include <boost/filesystem.hpp>

#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane_parallel.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>


namespace hdl_graph_slam {

InteractiveGraph::InteractiveGraph() :
		GraphSLAM("lm_var_cholmod"), iterations(0), chi2_before(0.0), chi2_after(
				0.0), elapsed_time_msec(0.0) {
	inf_calclator.reset(new InformationMatrixCalculator());
	inf_calclator->load(params);
	edge_id_gen = 0;

	anchor_node = nullptr;
	anchor_edge = nullptr;
	floor_node = nullptr;
}

InteractiveGraph::~InteractiveGraph() {
	if (optimization_thread.joinable()) {
		optimization_thread.join();
	}
}

bool InteractiveGraph::load_map_data(const std::string &directory,
		guik::ProgressInterface &progress, 
		Box3D &box) {
	// load graph file
	progress.set_title("Opening " + directory);
	progress.set_text("loading graph");

	if (!load(directory + "/graph.g2o", box) ) {
		return false;
	}

	// re-assign edge ids
	// note: newly created edges may have inconsistent IDs (who cares!)
	edge_id_gen = 0;
	for (auto &edge : graph->edges()) {
		edge->setId(edge_id_gen++);
	}

    for (auto &vertex : graph->vertices()) {
        if (max_vertex_id < vertex.second->id())
            max_vertex_id = vertex.second->id();
    }

	// load keyframes
	progress.increment();
	progress.set_text("loading keyframes");

	if (!load_keyframes(directory, progress)) {
		return false;
	}
	// load anchor and floor nodes
	if (!load_special_nodes(directory, progress)) {
		return false;
	}
    boost::filesystem::path p(directory);
    map_names.push_back(p.filename().string());

	return true;
}

std::vector<InteractiveGraph*> InteractiveGraph::split_map_data() {

    std::vector<InteractiveGraph*> interactive_graphs;

    // in case there is a single map, return its pointer
    if (map_names.size() < 2) {
        interactive_graphs.push_back(this);
        return interactive_graphs;
    }

    for(int mapIdx=0; mapIdx < map_names.size(); ++mapIdx) {
        InteractiveGraph *graph_ = new InteractiveGraph();

        // remove the anchor node in the graph to be merged
        if (graph_->anchor_node) {
            graph_->graph->removeEdge(graph_->anchor_edge);
            graph_->graph->detachVertex(graph_->anchor_node);
        }

        std::string map_name = map_names[mapIdx];

        graph_->map_names.push_back(map_name);

        g2o::Factory *factory = g2o::Factory::instance();
        std::unordered_map<long, g2o::HyperGraph::Vertex*> new_vertices_map; // old vertex Id -> new vertex instance map

        std::vector<long> vertex_ids;
        long tmp_max_vertex_id = max_vertex_id;
        if (mapIdx < vector_max_id_maps.size() - 1) {
            tmp_max_vertex_id = vector_max_id_maps[mapIdx+1];
        }
        // clone vertices
        for (int vertex_idx=vector_max_id_maps[mapIdx]; vertex_idx < tmp_max_vertex_id; ++vertex_idx) {
            const auto vertex = graph->vertices()[vertex_idx];
            const auto tmp_v_id = vertex->id();
            auto v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex);

            // copy params via g2o::Factory
            std::stringstream sst;
            if (!v->write(sst)) {
                std::cerr << "error: failed to write vertex data" << std::endl;
            }

            auto new_v =
                    dynamic_cast<g2o::OptimizableGraph::Vertex*>(factory->construct(
                            factory->tag(v)));
            if (!new_v->read(sst)) {
                std::cerr << "error: failed to read vertex data" << std::endl;
            }
            new_v->setFixed(v->fixed());
            new_v->setId(v->id() - vector_max_id_maps[mapIdx]);
            graph_->graph->addVertex(new_v);

            if (new_v->id() == 0) {
                std::cout << "HERE";
            }

            vertex_ids.push_back(v->id());

            // for remapping
            new_vertices_map[v->id()] = new_v;
        }

        // clone edges
        long new_edge_id = 0;
        for (const auto &edge : graph->edges()) {
            auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);

            // copy params via g2o::Factory
            std::stringstream sst;
            if (!e->write(sst)) {
                std::cerr << "error: failed to write edge data" << std::endl;
            }

            auto new_edge =
                    dynamic_cast<g2o::OptimizableGraph::Edge*>(factory->construct(
                            factory->tag(e)));
            if (!new_edge->read(sst)) {
                std::cerr << "error: failed to read edge data" << std::endl;
            }
            new_edge->setId(edge->id());

            // remap vertices with new ones
            bool valid_edge = true;
            g2o::HyperGraph::VertexContainer new_vertices;
            for (int vertex_idx = 0; vertex_idx < new_edge->vertices().size(); vertex_idx++) {
                const long currVertexId = e->vertices()[vertex_idx]->id();
                if (std::find(vertex_ids.begin(), vertex_ids.end(), currVertexId) != vertex_ids.end()) {
                    new_vertices.push_back(new_vertices_map[currVertexId]);
                } else {
                    valid_edge = false;
                    break;
                }

            }
            // if all vertices of edge is in current map, add edge to graph
            if (valid_edge) {
                new_edge->vertices() = new_vertices;

                // copy robust kernel
                if (e->robustKernel()) {
                    g2o::RobustKernel *kernel = nullptr;

                    if (dynamic_cast<g2o::RobustKernelHuber*>(e->robustKernel())) {
                        kernel = new g2o::RobustKernelHuber();
                    }

                    if (kernel == nullptr) {
                        std::cerr << "warning: unknown kernel type!!" << std::endl;
                    } else {
                        kernel->setDelta(e->robustKernel()->delta());
                        new_edge->setRobustKernel(kernel);
                    }
                }

                new_edge->setId(new_edge_id++);
                graph_->graph->addEdge(new_edge);
            }

        }

        // copy keyframes
        for (const auto& keyframe : keyframes) {
            if (new_vertices_map[keyframe.second->id()] != nullptr) {
                const auto new_keyframe = std::make_shared<InteractiveKeyFrame>(*keyframe.second);
                new_keyframe->node =
                        dynamic_cast<g2o::VertexSE3*>(new_vertices_map[new_keyframe->id()]);
                std::cout << "keyframe_id:" << new_keyframe->node->id() << std::endl;
                assert(new_keyframe->node);
                graph_->keyframes[new_keyframe->node->id()] = new_keyframe;
            }
        }
        interactive_graphs.push_back(graph_);
    }

    return interactive_graphs;
}

bool InteractiveGraph::merge_map_data(InteractiveGraph &graph_,
		const InteractiveKeyFrame::Ptr &key1,
		const InteractiveKeyFrame::Ptr &key2,
		const Eigen::Isometry3d &relative_pose) {


	long max_edge_id = 0;
    vector_max_id_maps.push_back(max_vertex_id+1);
    map_names.push_back(graph_.map_names[0]);
	for (const auto &edge : graph->edges()) {
		max_edge_id = std::max<long>(max_edge_id, edge->id());
	}

	g2o::Factory *factory = g2o::Factory::instance();
	std::unordered_map<long, g2o::HyperGraph::Vertex*> new_vertices_map; // old vertex Id -> new vertex instance map

	// remove the anchor node in the graph to be merged
	if (graph_.anchor_node) {
		graph_.graph->removeEdge(graph_.anchor_edge);
		graph_.graph->detachVertex(graph_.anchor_node);
	}

	// clone vertices
	// loop over vetex id for ensure the order
	for (int vertex_id=0; vertex_id < graph_.graph->vertices().size(); ++vertex_id) {
        const auto vertex = graph_.graph->vertices()[vertex_id];
		long new_vertex_id = ++max_vertex_id;
		auto v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex);

		// copy params via g2o::Factory
		std::stringstream sst;
		if (!v->write(sst)) {
			std::cerr << "error: failed to write vertex data" << std::endl;
			return false;
		}

		auto new_v =
				dynamic_cast<g2o::OptimizableGraph::Vertex*>(factory->construct(
						factory->tag(v)));
		if (!new_v->read(sst)) {
			std::cerr << "error: failed to read vertex data" << std::endl;
			return false;
		}
		new_v->setFixed(v->fixed());
		new_v->setId(new_vertex_id);
		graph->addVertex(new_v);

		// for remapping
		new_vertices_map[v->id()] = new_v;
	}

	// clone edges
	for (const auto &edge : graph_.graph->edges()) {
		long new_edge_id = ++max_edge_id;
		auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);

		// copy params via g2o::Factory
		std::stringstream sst;
		if (!e->write(sst)) {
			std::cerr << "error: failed to write edge data" << std::endl;
			return false;
		}

		auto new_e =
				dynamic_cast<g2o::OptimizableGraph::Edge*>(factory->construct(
						factory->tag(e)));
		if (!new_e->read(sst)) {
			std::cerr << "error: failed to read edge data" << std::endl;
			return false;
		}
		new_e->setId(new_edge_id);

		// remap vertices with new ones
		for (int i = 0; i < new_e->vertices().size(); i++) {
			new_e->vertices()[i] = new_vertices_map[e->vertices()[i]->id()];
		}

		// copy robust kernel
		if (e->robustKernel()) {
			g2o::RobustKernel *kernel = nullptr;

			if (dynamic_cast<g2o::RobustKernelHuber*>(e->robustKernel())) {
				kernel = new g2o::RobustKernelHuber();
			}

			if (kernel == nullptr) {
				std::cerr << "warning: unknown kernel type!!" << std::endl;
			} else {
				kernel->setDelta(e->robustKernel()->delta());
				new_e->setRobustKernel(kernel);
			}
		}

		edge->setId(new_edge_id);
		graph->addEdge(new_e);
	}

	// copy keyframes
	for (const auto &keyframe : graph_.keyframes) {
		keyframe.second->node =
				dynamic_cast<g2o::VertexSE3*>(new_vertices_map[keyframe.second->id()]);
		std::cout << "keyframe_id:" << keyframe.second->node->id() << std::endl;
		assert(keyframe.second->node);
		keyframes[keyframe.second->id()] = keyframe.second;
	}

	return true;
}

long InteractiveGraph::max_id_map() {
    long max_vertex_id = 0;
    for (const auto &vertex : graph->vertices()) {
        max_vertex_id = std::max<long>(max_vertex_id, vertex.first);
    }
    return max_vertex_id;

}

bool InteractiveGraph::load_special_nodes(const std::string &directory,
		guik::ProgressInterface &progress) {
	// load special nodes from file
	std::ifstream ifs(directory + "/special_nodes.csv");
	if (ifs) {
		while (!ifs.eof()) {
			std::string line;
			std::getline(ifs, line);

			if (line.empty()) {
				continue;
			}

			std::stringstream sst(line);
			std::string tag;
			sst >> tag;

			// load anchor node
			if (tag == "anchor_node") {
				long anchor_node_id = -1;
				sst >> anchor_node_id;

				if (anchor_node_id < 0) {
					continue;
				}

				anchor_node = dynamic_cast<g2o::VertexSE3*>(graph->vertex(
						anchor_node_id));
				if (anchor_node == nullptr) {
					std::cerr << "failed to cast anchor node to VertexSE3!!"
							<< std::endl;
					return false;
				}
				if (anchor_node->edges().empty()) {
					std::cerr << "anchor node is not connected with any edges!!"
							<< std::endl;
					return false;
				}

				anchor_edge =
						dynamic_cast<g2o::EdgeSE3*>(*anchor_node->edges().begin());
				if (anchor_edge == nullptr) {
					std::cerr << "failed to cast anchor edge to EdgeSE3!!"
							<< std::endl;
					return false;
				}

			}
			// load floor node
			else if (tag == "floor_node") {
				long floor_node_id = -1;
				sst >> floor_node_id;

				if (floor_node_id < 0) {
					continue;
				}

				floor_node = dynamic_cast<g2o::VertexPlane*>(graph->vertex(
						floor_node_id));
				if (floor_node == nullptr) {
					std::cerr << "failed to cast floor node to VertexPlane!!"
							<< std::endl;
					return false;
				}
			}
		}
	}

	// create anchor node if it is not loaded yet
	if (anchor_node == nullptr) {
		std::cout << "create new anchor" << std::endl;
		using ID_Keyframe = std::pair<long, InteractiveKeyFrame::Ptr>;
		auto first_keyframe = std::min_element(keyframes.begin(),
				keyframes.end(),
				[=](const ID_Keyframe &lhs, const ID_Keyframe &rhs) {
					return lhs.first < rhs.first;
				});

		if (first_keyframe == keyframes.end()) {
			std::cerr << "corrupted graph file!!" << std::endl;
			return false;
		}

		anchor_node = add_se3_node(first_keyframe->second->node->estimate());
		anchor_edge = add_se3_edge(anchor_node, first_keyframe->second->node,
				Eigen::Isometry3d::Identity(),
				Eigen::MatrixXd::Identity(6, 6) * 0.1);
	}

	std::cout << "anchor_node:" << anchor_node->id() << std::endl;
	std::cout << "anchor_edge:" << anchor_edge->vertices()[0]->id() << " - "
			<< anchor_edge->vertices()[1]->id() << std::endl;

	anchor_node->setFixed(true);

	return true;
}

bool InteractiveGraph::load_keyframes(const std::string &directory,
		guik::ProgressInterface &progress) {
	progress.set_maximum(graph->vertices().size());
	for (int i = 0;; i++) {
		std::string keyframe_dir =
				(boost::format("%s/%06d") % directory % i).str();
		if (!boost::filesystem::is_directory(keyframe_dir)) {
			break;
		}

		InteractiveKeyFrame::Ptr keyframe = std::make_shared
				< InteractiveKeyFrame > (keyframe_dir, graph.get());
		if (!keyframe->node) {
			std::cerr << "error : failed to load keyframe!!" << std::endl;
			std::cerr << "      : " << keyframe_dir << std::endl;
		} else {
			keyframes[keyframe->id()] = keyframe;
			progress.increment();
		}
	}

	return true;
}

long InteractiveGraph::anchor_node_id() const {
	return anchor_node ? anchor_node->id() : -1;
}

g2o::EdgeSE3* InteractiveGraph::add_edge(const KeyFrame::Ptr &key1,
		const KeyFrame::Ptr &key2, const Eigen::Isometry3d &relative_pose,
		const std::string &robust_kernel, double robust_kernel_delta) {
	Eigen::MatrixXd inf = inf_calclator->calc_information_matrix(key1->cloud,
			key2->cloud, relative_pose);
	g2o::EdgeSE3 *edge = add_se3_edge(key1->node, key2->node, relative_pose,
			inf);
	edge->setId(edge_id_gen++);

	if (robust_kernel != "NONE") {
		add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
	}

	return edge;
}

g2o::VertexPlane* InteractiveGraph::add_plane(const Eigen::Vector4d &coeffs) {
	return add_plane_node(coeffs);
}

g2o::EdgeSE3Plane* InteractiveGraph::add_edge(const KeyFrame::Ptr &v_se3,
		g2o::VertexPlane *v_plane, const Eigen::Vector4d &coeffs,
		const Eigen::MatrixXd &information, const std::string &robust_kernel,
		double robust_kernel_delta) {
	g2o::EdgeSE3Plane *edge = add_se3_plane_edge(v_se3->node, v_plane, coeffs,
			information);
	edge->setId(edge_id_gen++);

	if (robust_kernel != "NONE") {
		add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
	}

	return edge;
}

void InteractiveGraph::apply_robust_kernel(g2o::HyperGraph::Edge *edge,
		const std::string &robust_kernel, double robust_kernel_delta) {
	add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
}

void InteractiveGraph::add_edge_identity(g2o::VertexPlane *v1,
		g2o::VertexPlane *v2, double information_scale,
		const std::string &robust_kernel, double robust_kernel_delta) {
	auto edge = add_plane_identity_edge(v1, v2, Eigen::Vector4d::Zero(),
			Eigen::Matrix4d::Identity() * information_scale);
	if (robust_kernel != "NONE") {
		add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
	}
	edge->setId(edge_id_gen++);
}

void InteractiveGraph::add_edge_parallel(g2o::VertexPlane *v1,
		g2o::VertexPlane *v2, double information_scale,
		const std::string &robust_kernel, double robust_kernel_delta) {
	auto edge = add_plane_parallel_edge(v1, v2, Eigen::Vector3d::Zero(),
			Eigen::Matrix3d::Identity() * information_scale);
	if (robust_kernel != "NONE") {
		add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
	}
	edge->setId(edge_id_gen++);
}

void InteractiveGraph::add_edge_perpendicular(g2o::VertexPlane *v1,
		g2o::VertexPlane *v2, double information_scale,
		const std::string &robust_kernel, double robust_kernel_delta) {
	auto edge = add_plane_perpendicular_edge(v1, v2, Eigen::Vector3d::Zero(),
			Eigen::MatrixXd::Identity(1, 1) * information_scale);
	if (robust_kernel != "NONE") {
		add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
	}
	edge->setId(edge_id_gen++);
}

bool InteractiveGraph::add_edge_prior_normal(long plane_vertex_id,
		const Eigen::Vector3d &normal, double information_scale,
		const std::string &robust_kernel, double robust_kernel_delta) {
	auto vertex = graph->vertex(plane_vertex_id);
	if (vertex == nullptr) {
		return false;
	}

	g2o::VertexPlane *vertex_plane = dynamic_cast<g2o::VertexPlane*>(vertex);
	if (vertex_plane == nullptr) {
		return false;
	}

	Eigen::Matrix3d inf = Eigen::Matrix3d::Identity() * information_scale;
	auto edge = this->add_plane_normal_prior_edge(vertex_plane, normal, inf);
	if (robust_kernel != "NONE") {
		add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
	}

	return true;
}

bool InteractiveGraph::add_edge_prior_distance(long plane_vertex_id,
		double distance, double information_scale,
		const std::string &robust_kernel, double robust_kernel_delta) {
	auto vertex = graph->vertex(plane_vertex_id);
	if (vertex == nullptr) {
		return false;
	}

	g2o::VertexPlane *vertex_plane = dynamic_cast<g2o::VertexPlane*>(vertex);
	if (vertex_plane == nullptr) {
		return false;
	}

	Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(1, 1) * information_scale;
	auto edge = this->add_plane_distance_prior_edge(vertex_plane, distance,
			inf);
	if (robust_kernel != "NONE") {
		add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
	}

	return true;
}

Eigen::Vector3f InteractiveGraph::get_vertex(int id) {
	auto vertex = keyframes[id]->node;
	double* estimated = new double[7];
	vertex->getEstimateData(estimated);
    // estimated = (x, y, z, qx, qy, qz, qw)
	auto ret = Eigen::Vector3f(estimated[0],estimated[1],estimated[2]);
	delete[] estimated;
	return ret;
}

void InteractiveGraph::move_vertex(int id, Eigen::Vector3f new_pos) {
    auto keyframe = keyframes[id];
    double *estimated = new double[7];
    keyframe->node->getEstimateData(estimated);
    estimated[0] = new_pos[0];
    estimated[1] = new_pos[1];
    estimated[2] = new_pos[2];
    keyframe->node->setEstimateData(estimated);
    keyframes.insert(std::pair<long, InteractiveKeyFrame::Ptr>(id,keyframe));
    delete[] estimated;
}

void InteractiveGraph::drag_vertex(int id, int axis, double delta,
		float speed) {
    if (axis == -1) return;
	auto keyframe = keyframes[id];
	double *estimated = new double[7];
	keyframe->node->getEstimateData(estimated);
    // estimated = (x, y, z, qx, qy, qz, qw)
	int sign = (delta > 0) - (delta < 0);
	estimated[axis] += speed * sign;
	keyframe->node->setEstimateData(estimated);
	keyframes.insert(std::pair<long, InteractiveKeyFrame::Ptr>(id,keyframe));
	delete[] estimated;
}

//special case when lock xy
void InteractiveGraph::drag_vertex(int id, double delta1, double delta2,
		float speed) {
	auto keyframe = keyframes[id];
	double *estimated = new double[7];
	keyframe->node->getEstimateData(estimated);

	// estimated = (x, y, z, qx, qy, qz, qw)
	estimated[0] += speed * delta1;
	estimated[1] += speed * delta2;
	//just move point
	keyframe->node->setEstimateData(estimated);
	delete[] estimated;
	keyframes.insert(std::pair<long, InteractiveKeyFrame::Ptr>(id, keyframe));
}

void InteractiveGraph::rotate_vertex
(int id, int axis, double delta, float speed) {
    if (axis == -1) return;
    auto keyframe = keyframes[id];

    double *estimated = new double[7];
    keyframe->node->getEstimateData(estimated);

    Eigen::Quaterniond q2;
    q2.w() = estimated[6];
    q2.z() = estimated[5];
    q2.y() = estimated[4];
    q2.x() = estimated[3];

    std::vector<double> rotate_angles = std::vector<double>{0.0, 0.0, 0.0};
    rotate_angles[axis] = delta * speed;
    // convert (roll, pitch, yaw) to quaternion value
    double cy = cos(rotate_angles[2] * 0.5);
    double sy = sin(rotate_angles[2] * 0.5);
    double cp = cos(rotate_angles[1] * 0.5);
    double sp = sin(rotate_angles[1] * 0.5);
    double cr = cos(rotate_angles[0] * 0.5);
    double sr = sin(rotate_angles[0] * 0.5);

    // estimated = (x, y, z, qx, qy, qz, qw)
    Eigen::Quaterniond q1;
    q1.w() = cr * cp * cy + sr * sp * sy;
    q1.x() = sr * cp * cy - cr * sp * sy;
    q1.y() = cr * sp * cy + sr * cp * sy;
    q1.z() = cr * cp * sy - sr * sp * cy;

    Eigen::Quaterniond q3;
    q3 = q2*q1;

    estimated[6] = q3.w();
    estimated[5] = q3.z();
    estimated[4] = q3.y();
    estimated[3] = q3.x();

    keyframe->node->setEstimateData(estimated);
    delete[] estimated;
    keyframes.insert(std::pair<long, InteractiveKeyFrame::Ptr>(id, keyframe));
}

void InteractiveGraph::drag_map(int flag1, int flag2, int axis, double delta, float speed) {
    if (axis == -1) return;
    for(const auto& keyframe: keyframes){
        int keyframe_id = keyframe.second->id();
        if (keyframe_id > flag1 && keyframe_id <= flag2){
            double  *estimated = new double[7];
            keyframe.second->node->getEstimateData(estimated);
            int sign = (delta > 0) - (delta < 0);
            estimated[axis] += speed * sign;
            keyframe.second->node->setEstimateData(estimated);
            delete[] estimated;
            keyframes.insert(keyframe);
        }
    }
}
//special case when lock xy
void InteractiveGraph::drag_map(int flag1, int flag2, double delta1, double delta2, float speed) {
    for(const auto& keyframe: keyframes){
        int keyframe_id = keyframe.second->id();
        if(keyframe_id > flag1 && keyframe_id <= flag2){
            double *estimated = new double[7];
            keyframe.second->node->getEstimateData(estimated);
            estimated[0] += speed * delta1;
            estimated[1] += speed * delta2;
            keyframe.second->node->setEstimateData(estimated);
            delete[] estimated;
            keyframes.insert(keyframe);
        }
    }
}

void InteractiveGraph::rotate_map(int flag1, int flag2,int axis ,double delta, float speed) {
    if (axis == -1) return;
    for(const auto& keyframe:keyframes){
        int keyframe_id = keyframe.second->node->id();
        if(keyframe_id  > flag1   and keyframe_id <= flag2){

            double *estimated = new double[7];
            keyframe.second->node->getEstimateData(estimated);

            // estimated = (x, y, z, qx, qy, qz, qw)
            Eigen::Quaterniond q2;
            q2.w() = estimated[6];
            q2.z() = estimated[5];
            q2.y() = estimated[4];
            q2.x() = estimated[3];

            std::vector<double> rotate_angles = std::vector<double>{0.0, 0.0, 0.0};

            rotate_angles[axis] = delta * speed;
            // convert (roll, pitch, yaw) to quaternion value
            double cy = cos(rotate_angles[2] * 0.5);
            double sy = sin(rotate_angles[2] * 0.5);
            double cp = cos(rotate_angles[1] * 0.5);
            double sp = sin(rotate_angles[1] * 0.5);
            double cr = cos(rotate_angles[0] * 0.5);
            double sr = sin(rotate_angles[0] * 0.5);

            // estimated = (x, y, z, qx, qy, qz, qw)
            Eigen::Quaterniond q1;
            q1.w() = cr * cp * cy + sr * sp * sy;
            q1.x() = sr * cp * cy - cr * sp * sy;
            q1.y() = cr * sp * cy + sr * cp * sy;
            q1.z() = cr * cp * sy - sr * sp * cy;

            // estimated = (x, y, z, qx, qy, qz, qw)
            Eigen::Quaterniond q3;
            q3 = q1*q2;

            estimated[6] = q3.w();
            estimated[5] = q3.z();
            estimated[4] = q3.y();
            estimated[3] = q3.x();

            Eigen::Quaterniond q4;
            q4.w() = 0.0;
            q4.x() = estimated[0];
            q4.y() = estimated[1];
            q4.z() = estimated[2];

            Eigen::Quaterniond q5;
            q5.w() = q1.w();
            q5.x() = -q1.x();
            q5.y() = -q1.y();
            q5.z() = -q1.z();

            Eigen::Quaterniond q6;
            q6 = q1*q4*q5;

            estimated[0] = q6.x();
            estimated[1] = q6.y();
            estimated[2] = q6.z();

            keyframe.second->node->setEstimateData(estimated);
            delete[] estimated;
            keyframes.insert(keyframe);
        }
    }
}

void InteractiveGraph::optimize(int num_iterations) {
	if (anchor_node) {
		// move the anchor node to the position of the very first keyframe
		// so that the keyframe can move freely while trying to stay around the origin
		g2o::VertexSE3 *first_keyframe =
				dynamic_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1]);
		if (first_keyframe == nullptr) {
			std::cerr
					<< "failed to cast the node which is likely to be the fist keyframe to VertexSE3";
		} else {
			anchor_node->setEstimate(first_keyframe->estimate());
		}
	}

	// !! bad tech !!
	// override std::cerr with optimization_stream to catch optimization progress messages
	optimization_stream.str("");
	optimization_stream.clear();

	std::streambuf *cerr_buf = std::cerr.rdbuf();
	std::cerr.rdbuf(optimization_stream.rdbuf());

	g2o::SparseOptimizer *graph =
			dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
	auto t1 = std::chrono::high_resolution_clock::now();

	if (num_iterations < 0) {
		num_iterations = params.param<int>("g2o_solver_num_iterations", 64);
	}

	chi2_before = graph->chi2();
	iterations = GraphSLAM::optimize(num_iterations);
	chi2_after = graph->chi2();

	auto t2 = std::chrono::high_resolution_clock::now();
	elapsed_time_msec = std::chrono::duration_cast < std::chrono::nanoseconds
			> (t2 - t1).count() / 1000000.0;

	std::cerr.rdbuf(cerr_buf);
}

void InteractiveGraph::optimize_background(int num_iterations) {
	if (optimization_thread.joinable()) {
		optimization_thread.join();
	}

	if (anchor_node) {
		// move the anchor node to the position of the very first keyframe
		// so that the keyframe can move freely while trying to stay around the origin
		g2o::VertexSE3 *first_keyframe =
				dynamic_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1]);
		if (first_keyframe == nullptr) {
			std::cerr
					<< "failed to cast the node which is likely to be the fist keyframe to VertexSE3";
		} else {
			anchor_node->setEstimate(first_keyframe->estimate());
		}
	}

	// !! bad tech !!
	// override std::cerr with optimization_stream to catch optimization progress messages
	optimization_stream.str("");
	optimization_stream.clear();

	auto task = [this, num_iterations]() {
		std::streambuf *cerr_buf = std::cerr.rdbuf();
		std::cerr.rdbuf(optimization_stream.rdbuf());

		std::lock_guard < std::mutex > lock(optimization_mutex);
		g2o::SparseOptimizer *graph =
				dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
		auto t1 = std::chrono::high_resolution_clock::now();

		int max_iterations = num_iterations;
		if (num_iterations < 0) {
			max_iterations = params.param<int>("g2o_solver_num_iterations", 64);
		}

		chi2_before = graph->chi2();
		iterations = GraphSLAM::optimize(max_iterations);
		chi2_after = graph->chi2();

		auto t2 = std::chrono::high_resolution_clock::now();
		elapsed_time_msec = std::chrono::duration_cast
				< std::chrono::nanoseconds > (t2 - t1).count() / 1000000.0;

		std::cerr.rdbuf(cerr_buf);
	};

	optimization_thread = std::thread(task);
}

std::string InteractiveGraph::graph_statistics(bool update) {
	if (optimization_mutex.try_lock()) {
		std::stringstream sst;
		sst << "Graph\n";
		sst << boost::format("# vertices: %d") % num_vertices() << "\n";
		sst << boost::format("# edges: %d") % num_edges() << "\n";
		sst << boost::format("time: %.1f[msec]") % elapsed_time_msec << "\n";
		sst << boost::format("chi2: %.3f -> %.3f") % chi2_before % chi2_after
				<< "\n";
		sst << boost::format("iterations: %d") % iterations;

		graph_stats = sst.str();
		optimization_mutex.unlock();
	}

	return graph_stats;
}

std::string InteractiveGraph::optimization_messages() const {
	return optimization_stream.str();
}

void InteractiveGraph::dump(const std::string &directory,
		guik::ProgressInterface &progress) {

	save(directory + "/graph.g2o");

	progress.set_text("saving keyframes");

	int keyframe_id = 0;
	for (const auto &keyframe : keyframes) {
		progress.increment();

		std::stringstream sst;
		sst << boost::format("%s/%06d") % directory % (keyframe_id++);
		keyframe.second->save(sst.str());
	}

	std::ofstream ofs(directory + "/special_nodes.csv");
	ofs << "anchor_node " << (anchor_node != nullptr ? anchor_node->id() : -1)
			<< std::endl;
	ofs << "anchor_edge " << -1 << std::endl;
	ofs << "floor_node " << (floor_node != nullptr ? floor_node->id() : -1)
			<< std::endl;
}

bool InteractiveGraph::save_pointcloud(const std::string &filename,
		guik::ProgressInterface &progress, float *voxel_leaf_sizes, int skip_frames) {
	progress.set_maximum(keyframes.size() + 1);
	progress.set_text("accumulate points");

	pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(
			new pcl::PointCloud<pcl::PointXYZI>());

    int count = -1;
    int frame_count = 0;
	for (const auto &keyframe: keyframes) {
		progress.increment();
        count += 1;
		if (count % skip_frames != 0 && count != keyframes.size()-1) {
		    continue;
		}
		pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(
				new pcl::PointCloud<pcl::PointXYZI>());
		
		keyframe.second->reloadCloud();
		pcl::transformPointCloud(*keyframe.second->cloud, *transformed,
				keyframe.second->node->estimate().cast<float>());

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filtered(
                new pcl::PointCloud<pcl::PointXYZI>());
        // Create the filtering object
        pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud (transformed);
        voxel.setLeafSize (voxel_leaf_sizes[0], voxel_leaf_sizes[1], voxel_leaf_sizes[2]);
        voxel.filter (*transformed_filtered);


        std::copy(transformed_filtered->begin(), transformed_filtered->end(),
				std::back_inserter(accumulated->points));
        frame_count += 1;
	}
    std::cout << "Total keyframes: " << count << std::endl;
    std::cout << "Skip frames: " << skip_frames << std::endl;
    std::cout << "Total exported keyframes: " << frame_count << std::endl;

	accumulated->is_dense = false;
	accumulated->width = accumulated->size();
	accumulated->height = 1;

	progress.set_text("saving pcd");
	progress.increment();

	return pcl::io::savePCDFileBinary(filename, *accumulated);
}
}  // namespace hdl_graph_slam
