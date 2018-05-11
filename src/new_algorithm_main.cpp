#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/extract_indices.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/typedefs.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#include "perception_experiment/exp_algorithm.h"
#include "perception_experiment/new_algorithm.h"
#include "perception_experiment/omps_algorithm.h"
#include "perception_experiment/ransac_algorithm.h"

#include "visualization_msgs/Marker.h"
#include "perception_experiment/visualization.h"

class Experiment {
 public:
  Experiment(const std::string& target_frame, const ros::Publisher& input_pub, const perception_experiment::SurfaceViz& viz);
  void Callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  bool get_is_done();
  double get_failure_rate();

 private:
  ros::Publisher input_pub_;
  perception_experiment::SurfaceViz viz_;
  std::string target_frame_;
  tf::TransformListener tf_listener_;
  size_t iterations_ran_;
  size_t failure_times_;
  size_t iteration_limit_;
  bool is_done_;
};

Experiment::Experiment(const std::string& target_frame,
                       const ros::Publisher& input_pub,
                       const perception_experiment::SurfaceViz& viz)
    : input_pub_(input_pub),
      target_frame_(target_frame),
      tf_listener_(),
      iteration_limit_(1000),
      iterations_ran_(0),
      failure_times_(0),
      is_done_(false),
      viz_(viz) {}

bool Experiment::get_is_done() { return is_done_; }

double Experiment::get_failure_rate() {
  if (!is_done_) {
    ROS_WARN("Warning: the experiment is not done yet.");
    return 0.0;
  }
  return 1.0 * failure_times_ / iterations_ran_;
}

void Experiment::Callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  PointCloudC::Ptr pcl_cloud_raw(new PointCloudC);
  pcl::fromROSMsg(*cloud, *pcl_cloud_raw);
  PointCloudC::Ptr pcl_cloud(new PointCloudC);

  if (cloud->header.frame_id != target_frame_) {
    tf_listener_.waitForTransform(target_frame_, cloud->header.frame_id,
                                  ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform;
    tf_listener_.lookupTransform(target_frame_, cloud->header.frame_id,
                                 ros::Time(0), transform);
    Eigen::Affine3d affine;
    tf::transformTFToEigen(transform, affine);
    pcl::transformPointCloud(*pcl_cloud_raw, *pcl_cloud, affine);
    pcl_cloud->header.frame_id = target_frame_;
  } else {
    pcl_cloud = pcl_cloud_raw;
  }

  perception_experiment::NewAlgorithm algo;
  algo.SetInputCloud(pcl_cloud);
  algo.SetParameters();

  PointCloudC::Ptr cropped_cloud(new PointCloudC);
  sensor_msgs::PointCloud2 msg_out;

  algo.GetInputCloud(cropped_cloud);
  pcl::toROSMsg(*cropped_cloud, msg_out);
  input_pub_.publish(msg_out);

  ros::WallDuration total_time = ros::WallDuration();
  while (iterations_ran_ < iteration_limit_ && ros::ok()) {
    iterations_ran_++;
    std::vector<surface_perception::Surface> surfaces;
    ros::WallDuration time_spent;
    algo.RunAlgorithm(&surfaces, &time_spent);
    total_time += time_spent;

    int min_iteration;
    ros::param::param("min_iteration", min_iteration, 1000);

    int expected_surface_amount;
    ros::param::param("expected_surface_amount", expected_surface_amount, 3);

    if (surfaces.size() != expected_surface_amount) {
      ROS_ERROR("Failed to find correct number of surfaces!");
      failure_times_++;
    }

    PointCloudC::Ptr output_cloud(new PointCloudC);
    output_cloud->header.frame_id = target_frame_;

    ROS_INFO(
        "Found %ld surfaces using %d iterations, %f milliseconds, at %ldth "
        "attempt",
        surfaces.size(), min_iteration, time_spent.toNSec() / 1000000.0,
        iterations_ran_);

    viz_.Hide();
    viz_.set_surfaces(surfaces);
    viz_.Show();
  }
  is_done_ = true;

  ROS_INFO(
      "The test finishes with the failure rate of %f, average time spent %f "
      "milliseconds",
      get_failure_rate(), total_time.toNSec() / 1000.0 / 1000000.0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_perception_experiment_test_new_algorithm");
  ros::NodeHandle nh;
  ros::Publisher input_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 100, true);
  ros::Publisher output_pub =
      nh.advertise<visualization_msgs::Marker>("detected_surface", 100, true);

  std::string target_frame("base_link");
  if (argc > 1) {
    target_frame = argv[1];
  }

  perception_experiment::SurfaceViz viz(output_pub);

  Experiment experiment(target_frame, input_pub, viz);
  ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "cloud_in", 1, &Experiment::Callback, &experiment);

  while (!experiment.get_is_done() && ros::ok()) {
    ros::spinOnce();
  }
}
