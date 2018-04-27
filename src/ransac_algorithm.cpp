#include "perception_experiment/ransac_algorithm.h"

#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/filter.h"
#include "ros/ros.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

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
  if (!ros::param::has("max_point_distance")) {
    ros::param::set("max_point_distance", 0.015);
  }

  return;
}
}  // namespace

namespace perception_experiment {
RANSACAlgorithm::RANSACAlgorithm()
    : algo_(pcl::SACSegmentation<PointC>()),
      uncropped_cloud_(new PointCloudC),
      cropped_cloud_(new PointCloudC) {}

void RANSACAlgorithm::SetInputCloud(PointCloudC::Ptr input_cloud) {
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

  algo_.setInputCloud(uncropped_cloud_);
  algo_.setIndices(point_indices);
}

void RANSACAlgorithm::SetParameters() {
  // May be useful...
  double angle_tolerance_degree;
  ros::param::param("angle_tolerance_degree", angle_tolerance_degree, 10.0);

  double max_point_distance;
  ros::param::param("max_point_distance", max_point_distance, 0.015);

  algo_.setModelType(pcl::SACMODEL_PLANE);
  algo_.setMethodType(pcl::SAC_RANSAC);
  algo_.setDistanceThreshold(max_point_distance);
}

void RANSACAlgorithm::RunAlgorithm(std::vector<PointCloudC::Ptr>* cloud_vec,
                                ros::WallDuration* time_spent) {
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  // Actually time the code
  ros::WallTime start = ros::WallTime::now();
  algo_.segment(*indices, *coeff);
  ros::WallTime end = ros::WallTime::now();
  *time_spent = end - start;

  PointCloudC::Ptr cloud(new PointCloudC);
  pcl::ExtractIndices<PointC> extract_indices;
  extract_indices.setInputCloud(uncropped_cloud_);
  extract_indices.setIndices(indices);
  extract_indices.filter(*cloud);
  cloud->header.frame_id = uncropped_cloud_->header.frame_id;
  cloud_vec->push_back(cloud);
}

void RANSACAlgorithm::GetInputCloud(PointCloudC::Ptr input_cloud) {
  *input_cloud = *cropped_cloud_;
}

}  // namespace perception_experiment
