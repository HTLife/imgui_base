//unused
#include <guik/vertex_control.hpp>

#include <memory>
#include <GL/gl3w.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Core>

namespace guik {

VertexControl::VertexControl(Eigen::Vector3f pos){
	position = pos;
}

VertexControl::~VertexControl() {}

void VertexControl::control(Eigen::Vector3f vertex){
	position = vertex;
}
void VertexControl::drag2d(float dragx, float dragy){
	position[0] += dragx;
	position[1] += dragy;
}

void VertexControl::drag1d(float dragz){
	position[2] += dragz;
}

Eigen::Vector3f generate_3d_location(Eigen::Vector3f old_pos, Eigen::Vector2f new_pos){

}

}  // namespace guik
