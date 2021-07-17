#include <hdl_graph_slam/registrations.hpp>

#include <iostream>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
namespace hdl_graph_slam {

boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> select_registration_method(MethodHandle pnh) {
  using PointT = pcl::PointXYZI;

  // select a registration method (ICP, GICP, NDT)
  std::string registration_method = pnh.registration_method;
  if(registration_method == "ICP") {
    std::cout << "registration: ICP" << std::endl;
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setTransformationEpsilon(pnh.transformation_epsilon);
    icp->setMaximumIterations(pnh.maximum_iterations);
    icp->setUseReciprocalCorrespondences(pnh.use_reciprocal_correspondences);
    return icp;
  } else if(registration_method.find("GICP") != std::string::npos) {
    if(registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: GICP" << std::endl;
      boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(pnh.transformation_epsilon);
      gicp->setMaximumIterations(pnh.maximum_iterations);
      gicp->setUseReciprocalCorrespondences(pnh.use_reciprocal_correspondences);
      gicp->setCorrespondenceRandomness(pnh.gicp_correspondence_randomness);
      gicp->setMaximumOptimizerIterations(pnh.gicp_max_optimizer_iterations);
      return gicp;
    } else {
      std::cout << "registration: GICP_OMP" << std::endl;
      boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
      gicp->setTransformationEpsilon(pnh.transformation_epsilon);
      gicp->setMaximumIterations(pnh.maximum_iterations);
      gicp->setUseReciprocalCorrespondences(pnh.use_reciprocal_correspondences);
      gicp->setCorrespondenceRandomness(pnh.gicp_correspondence_randomness);
      gicp->setMaximumOptimizerIterations(pnh.gicp_max_optimizer_iterations);
      return gicp;
    }
  } else {
    if(registration_method.find("NDT") == std::string::npos ) {
      std::cerr << "warning: unknown registration type(" << registration_method << ")" << std::endl;
      std::cerr << "       : use NDT" << std::endl;
    }

    double ndt_resolution = pnh.ndt_resolution;
    if(registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: NDT " << ndt_resolution << std::endl;
      boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(pnh.transformation_epsilon);
      ndt->setMaximumIterations(pnh.maximum_iterations);
      ndt->setResolution(ndt_resolution);
      return ndt;
    } else {
      int num_threads = pnh.ndt_num_threads;
      std::string nn_search_method = pnh.ndt_nn_search_method;
      std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;
      boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      if(num_threads > 0) {
        ndt->setNumThreads(num_threads);
      }
      ndt->setTransformationEpsilon(pnh.transformation_epsilon);
      ndt->setMaximumIterations(pnh.maximum_iterations);
      ndt->setResolution(ndt_resolution);
      if(nn_search_method == "KDTREE") {
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      } else if (nn_search_method == "DIRECT1") {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else {
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      }
      return ndt;
    }
  }

  return nullptr;
}

}
