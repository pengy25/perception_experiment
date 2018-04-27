#include "perception_experiment/exp_algorithm.h"

#include "pcl/PointIndices.h"
#include "ros/ros.h"
#include "surface_perception/typedefs.h"

namespace perception_experiment {
ExpAlgorithm::ExpAlgorithm() {}

void ExpAlgorithm::SetInputCloud(PointCloudC::Ptr input_cloud) {
  ROS_ERROR("You should override SetInputCloud!");
}

void ExpAlgorithm::SetParameters() {
  ROS_ERROR("You should override SetParameters!");
}

void ExpAlgorithm::RunAlgorithm(std::vector<PointCloudC::Ptr>* cloud_vec,
                                ros::WallDuration* time_spent) {
  ROS_ERROR("You should override RunAlgorithm!");
  return;
}

void ExpAlgorithm::GetInputCloud(PointCloudC::Ptr input_cloud) {
  ROS_ERROR("You should override GetInputCloud!");
}

}  // namespace perception_experiment
