#ifndef HDL_GRAPH_SLAM_KEYFRAME_VIEW_HPP
#define HDL_GRAPH_SLAM_KEYFRAME_VIEW_HPP

#include <memory>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <pcl/common/common.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <hdl_graph_slam/view/vertex_view.hpp>
#include <hdl_graph_slam/interactive_keyframe.hpp>
#include <pcl/filters/approximate_voxel_grid.h>


namespace hdl_graph_slam {

class KeyFrameView: public VertexView {
public:
	using Ptr = std::shared_ptr<KeyFrameView>;

	KeyFrameView(const InteractiveKeyFrame::Ptr &kf) :
			VertexView(kf->node) {
		keyframe = kf;
        origin_cloud = kf->cloud;

        pointcloud_buffer.reset(new glk::PointCloudBuffer(origin_cloud));
        vertex_size = 1;
	}

	InteractiveKeyFrame::Ptr lock() const {
		return keyframe.lock();
	}

	virtual bool available() const override {
		return !keyframe.expired();
	}

	virtual void voxelize(float *voxel_leaf_sizes) override {

	    if (voxel_leaf_sizes == nullptr) {
            pointcloud_buffer.reset(new glk::PointCloudBuffer(origin_cloud));
	    } else {
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filtered(
                    new pcl::PointCloud<pcl::PointXYZI>());

            pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud (origin_cloud);
            voxel.setLeafSize (voxel_leaf_sizes[0], voxel_leaf_sizes[1], voxel_leaf_sizes[2]);
            voxel.filter (*transformed_filtered);

            pointcloud_buffer.reset(new glk::PointCloudBuffer(transformed_filtered));
	    }

	}

	virtual void draw(const DrawFlags &flags, glk::GLSLShader &shader)
		override {
		InteractiveKeyFrame::Ptr kf = keyframe.lock();
		Eigen::Matrix4f model_matrix = kf->estimate().matrix().cast<float>();
		shader.set_uniform("color_mode", 0);
		shader.set_uniform("model_matrix", model_matrix);
		shader.set_uniform("info_values", Eigen::Vector4i(POINTS, kf->id(), 0, 0));

		pointcloud_buffer->draw(shader);

		if (!flags.draw_verticies || !flags.draw_keyframe_vertices) {
			return;
		}

		shader.set_uniform("color_mode", 1);
		shader.set_uniform("material_color",
				Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
		shader.set_uniform("info_values",
				Eigen::Vector4i(VERTEX | KEYFRAME, kf->id(), 0, 0));

		model_matrix.block<3, 3>(0, 0) *= 0.35*vertex_size;
		shader.set_uniform("model_matrix", model_matrix);
		const auto &sphere = glk::Primitives::instance()->primitive(
				glk::Primitives::SPHERE);
		sphere.draw(shader);
	}
	//draw special keyframe
	virtual void draw(const DrawFlags &flags, glk::GLSLShader &shader, int id, const Eigen::Vector4f& color)
			override {
		InteractiveKeyFrame::Ptr kf = keyframe.lock();
		auto a = kf->estimate();
		Eigen::Matrix4f model_matrix = kf->estimate().matrix().cast<float>();
		shader.set_uniform("color_mode", 0);
		shader.set_uniform("model_matrix", model_matrix);
		shader.set_uniform("info_values", Eigen::Vector4i(POINTS, kf->id(), 0, 0));

		pointcloud_buffer->draw(shader);

		if (!flags.draw_verticies || !flags.draw_keyframe_vertices) {
			return;
		}
		shader.set_uniform("color_mode", 1);
		if (kf->id() == id)
			shader.set_uniform("material_color",color);
		else
			shader.set_uniform("material_color",
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
		shader.set_uniform("info_values",
				Eigen::Vector4i(VERTEX | KEYFRAME, kf->id(), 0, 0));

		model_matrix.block<3, 3>(0, 0) *= 0.35*vertex_size;
		shader.set_uniform("model_matrix", model_matrix);
		const auto &sphere = glk::Primitives::instance()->primitive(
				glk::Primitives::SPHERE);
		sphere.draw(shader);
	}
	virtual void draw(const DrawFlags &flags, glk::GLSLShader &shader,
			const Eigen::Vector4f &color, const Eigen::Matrix4f &model_matrix)
					override {
		if (!available()) {
			return;
		}
        Eigen::Matrix4f tmp_model_matrix = model_matrix;
		InteractiveKeyFrame::Ptr kf = keyframe.lock();

		shader.set_uniform("color_mode", 1);
		shader.set_uniform("material_color", color);

		shader.set_uniform("model_matrix", model_matrix);
		shader.set_uniform("info_values", Eigen::Vector4i(POINTS, kf->id(), 0, 0));

		pointcloud_buffer->draw(shader);
        tmp_model_matrix.block<3, 3>(0, 0) *= 0.5*vertex_size;
        shader.set_uniform("model_matrix", tmp_model_matrix);
		shader.set_uniform("color_mode", 1);
		shader.set_uniform("info_values",
				Eigen::Vector4i(VERTEX | KEYFRAME, kf->id(), 0, 0));
		const auto &sphere = glk::Primitives::instance()->primitive(
				glk::Primitives::SPHERE);
		sphere.draw(shader);
	}

    virtual void set_vertex_size(float vertex_size_) {
	    vertex_size = vertex_size_;
	}

private:

	std::weak_ptr<InteractiveKeyFrame> keyframe;
	std::unique_ptr<glk::PointCloudBuffer> pointcloud_buffer;
    pcl::PointCloud<KeyFrame::PointT>::ConstPtr origin_cloud;
    float vertex_size;
};

}  // namespace hdl_graph_slam

#endif
