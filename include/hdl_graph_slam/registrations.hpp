#ifndef HDL_GRAPH_SLAM_REGISTRATIONS_HPP
#define HDL_GRAPH_SLAM_REGISTRATIONS_HPP

#include <pcl/registration/registration.h>

namespace hdl_graph_slam {

/**
 * @brief select a scan matching algorithm according to rosparams
 * @param pnh
 * @return selected scan matching
 */
struct MethodHandle {
	std::string registration_method;
	double transformation_epsilon;
	int maximum_iterations;
	bool use_reciprocal_correspondences;
	int gicp_correspondence_randomness;
	int gicp_max_optimizer_iterations;
	double ndt_resolution;
	int ndt_num_threads;
	std::string ndt_nn_search_method;
};
boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> select_registration_method(MethodHandle& nh);

}

#endif //
