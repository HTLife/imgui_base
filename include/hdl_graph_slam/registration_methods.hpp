#ifndef HDL_GRAPH_SLAM_REGISTRATION_METHODS_HPP
#define HDL_GRAPH_SLAM_REGISTRATION_METHODS_HPP

#include <string>
#include <vector>
#include <pcl/registration/registration.h>

namespace hdl_graph_slam {

class RegistrationMethods {
public:
  RegistrationMethods();
  ~RegistrationMethods();

  void draw_ui();

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr method() const;

  float getRegistrationResolution(){return registration_resolution;};
  void setErrorMessage(std::string error);

private:
  int registration_method;
  float registration_resolution;
  std::string error_message="";
  float transformation_epsilon;
  int max_iterations;

  std::vector<const char*> registration_methods;
};
}  // namespace hdl_graph_slam

#endif