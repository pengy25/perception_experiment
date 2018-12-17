#include "perception_experiment/new_algorithm.h"

#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/filter.h"
#include "ros/ros.h"

#include "surface_perception/shape_extraction.h"
#include "surface_perception/surface.h"
#include "surface_perception/surface_finder.h"

namespace {
void SetupROSParams() {
  if (!ros::param::has("crop_min_x")) {
    ros::param::set("crop_min_x", 0.0);
  }
  if (!ros::param::has("crop_min_y")) {
    ros::param::set("crop_min_y", -0.5);
  }
  if (!ros::param::has("crop_min_z")) {
    ros::param::set("crop_min_z", 0.05);
  }
  if (!ros::param::has("crop_max_x")) {
    ros::param::set("crop_max_x", 1.3);
  }
  if (!ros::param::has("crop_max_y")) {
    ros::param::set("crop_max_y", 0.5);
  }
  if (!ros::param::has("crop_max_z")) {
    ros::param::set("crop_max_z", 2.0);
  }
  if (!ros::param::has("angle_tolerance_degree")) {
    ros::param::set("angle_tolerance_degree", 10.0);
  }
  if (!ros::param::has("max_point_distance")) {
    ros::param::set("max_point_distance", 0.015);
  }
  if (!ros::param::has("surface_point_threshold")) {
    ros::param::set("surface_point_threshold", 5000);
  }
  if (!ros::param::has("min_iteration")) {
    ros::param::set("min_iteration", 1000);
  }
  return;
}
}  // namespace

namespace perception_experiment {
NewAlgorithm::NewAlgorithm()
    : algo_(surface_perception::SurfaceFinder()),
      uncropped_cloud_(new PointCloudC),
      cropped_cloud_(new PointCloudC) {
  ROS_INFO("Using New");
}

void NewAlgorithm::SetInputCloud(PointCloudC::Ptr input_cloud) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *uncropped_cloud_, indices);

  pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(uncropped_cloud_);

  SetupROSParams();

  double max_x, max_y, max_z, min_x, min_y, min_z;
  ros::param::param("crop_min_x", min_x, 0.0);
  ros::param::param("crop_min_y", min_y, -0.5);
  ros::param::param("crop_min_z", min_z, 0.05);
  ros::param::param("crop_max_x", max_x, 1.3);
  ros::param::param("crop_max_y", max_y, 0.5);
  ros::param::param("crop_max_z", max_z, 2.0);

  Eigen::Vector4f min;
  min << min_x, min_y, min_z, 1;
  crop.setMin(min);
  Eigen::Vector4f max;
  max << max_x, max_y, max_z, 1;
  crop.setMax(max);
  crop.filter(point_indices->indices);
  crop.filter(*cropped_cloud_);

  algo_.set_cloud(uncropped_cloud_);
  algo_.set_cloud_indices(point_indices);
}

void NewAlgorithm::SetParameters() {
  double angle_tolerance_degree;
  ros::param::param("angle_tolerance_degree", angle_tolerance_degree, 10.0);
  double max_point_distance;
  ros::param::param("max_point_distance", max_point_distance, 0.015);
  int surface_point_threshold;
  ros::param::param("surface_point_threshold", surface_point_threshold, 5000);
  int min_iteration;
  ros::param::param("min_iteration", min_iteration, 1000);

  int max_surface_amount = 10;

  surface_perception::EstimateParameters(
      cropped_cloud_->points.size(), surface_point_threshold, 0.30,
      &max_surface_amount, &min_iteration);

  ros::param::set("min_iteration", min_iteration);

  algo_.set_angle_tolerance_degree(angle_tolerance_degree);
  algo_.set_surface_point_threshold(surface_point_threshold);
  algo_.set_max_point_distance(max_point_distance);
  algo_.set_min_iteration(min_iteration);
}

void NewAlgorithm::RunAlgorithm(
    std::vector<surface_perception::Surface>* surfaces,
    ros::WallDuration* time_spent) {
  std::vector<pcl::ModelCoefficients> coeff_vec;
  std::vector<pcl::PointIndices::Ptr> indices_vec;

  // Actually time the code
  ros::WallTime start = ros::WallTime::now();
  algo_.ExploreSurfaces(&indices_vec, &coeff_vec);
  ros::WallTime end = ros::WallTime::now();
  *time_spent = end - start;

  for (size_t i = 0; i < indices_vec.size(); i++) {
    surface_perception::Surface surface;
    surface.coefficients.reset(new pcl::ModelCoefficients);
    surface.coefficients->values = coeff_vec[i].values;
    surface.pose_stamped.header.frame_id = uncropped_cloud_->header.frame_id;
    if (surface_perception::FitBox(
            uncropped_cloud_, indices_vec[i], surface.coefficients,
            &surface.pose_stamped.pose, &surface.dimensions)) {
      double offset = surface.coefficients->values[0] *
                          surface.pose_stamped.pose.position.x +
                      surface.coefficients->values[1] *
                          surface.pose_stamped.pose.position.y +
                      surface.coefficients->values[2] *
                          surface.pose_stamped.pose.position.z +
                      surface.coefficients->values[3];
      surface.pose_stamped.pose.position.z -= offset;
      surfaces->push_back(surface);
    }
  }
}

void NewAlgorithm::GetInputCloud(PointCloudC::Ptr input_cloud) {
  *input_cloud = *cropped_cloud_;
}

}  // namespace perception_experiment
