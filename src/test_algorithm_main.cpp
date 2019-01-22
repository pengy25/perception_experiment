#include <iostream>
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

#include "perception_experiment/visualization.h"
#include "visualization_msgs/Marker.h"

class Experiment {
 public:
  Experiment(const std::string& algo_name, const std::string& target_frame,
             const ros::Publisher& input_pub,
             const perception_experiment::SurfaceViz& viz);
  void Callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  bool get_is_done();

 private:
  std::string algo_name_;
  std::string target_frame_;
  ros::Publisher input_pub_;
  perception_experiment::SurfaceViz viz_;
  tf::TransformListener tf_listener_;
  size_t iterations_ran_;
  size_t iteration_limit_;
  bool is_done_;
};

Experiment::Experiment(const std::string& algo_name,
                       const std::string& target_frame,
                       const ros::Publisher& input_pub,
                       const perception_experiment::SurfaceViz& viz)
    : algo_name_(algo_name),
      target_frame_(target_frame),
      input_pub_(input_pub),
      viz_(viz),
      tf_listener_(),
      iteration_limit_(10),
      iterations_ran_(0),
      is_done_(false) {}

bool Experiment::get_is_done() { return is_done_; }

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

  perception_experiment::ExpAlgorithm* algo;
  if (algo_name_ == "new") {
    algo = new perception_experiment::NewAlgorithm();
  } else if (algo_name_ == "omps") {
    algo = new perception_experiment::OMPSAlgorithm();
  } else {
    ROS_ERROR("Not supported algo!");
    return;
  }

  algo->SetInputCloud(pcl_cloud);
  algo->SetParameters();

  PointCloudC::Ptr cropped_cloud(new PointCloudC);
  sensor_msgs::PointCloud2 msg_out;

  algo->GetInputCloud(cropped_cloud);
  pcl::toROSMsg(*cropped_cloud, msg_out);
  input_pub_.publish(msg_out);

  ros::WallDuration total_time = ros::WallDuration();
  while (iterations_ran_ < iteration_limit_ && ros::ok()) {
    iterations_ran_++;
    std::vector<surface_perception::Surface> surfaces;
    ros::WallDuration time_spent;
    algo->RunAlgorithm(&surfaces, &time_spent);
    ros::param::set("time_algo_overall", time_spent.toNSec() / 1000000.0);
    total_time += time_spent;

    if (algo_name_ == "new") {
      int min_iteration;
      ros::param::param("min_iteration", min_iteration, 1000);
      ROS_INFO(
          "%s algorithm found %ld surfaces using %d iterations, %f "
          "milliseconds, at %ldth "
          "attempt",
          algo_name_.c_str(), surfaces.size(), min_iteration,
          time_spent.toNSec() / 1000000.0, iterations_ran_);
    } else {
      ROS_INFO(
          "%s algorithm found %ld surfaces using %f milliseconds, at %ldth "
          "attempt",
          algo_name_.c_str(), surfaces.size(), time_spent.toNSec() / 1000000.0,
          iterations_ran_);
    }

    viz_.Hide();
    viz_.set_surfaces(surfaces);
    viz_.Show();
    viz_.Save(algo_name_, iterations_ran_, cropped_cloud);

    std::cout << "Press enter to continue!" << std::endl;
    std::string tmp;
    std::getline(std::cin, tmp);
  }
  is_done_ = true;

  ROS_INFO("The test finishes with average time spent %f milliseconds",
           total_time.toNSec() / iteration_limit_ / 1000000.0);

  delete algo;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_perception_experiment_test_algorithm");
  ros::NodeHandle nh;
  ros::Publisher input_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 100, true);
  ros::Publisher output_pub =
      nh.advertise<visualization_msgs::Marker>("detected_surface", 100, true);

  if (argc != 2) {
    ROS_ERROR("usage: rosrun run_algorithm algo");
    return 1;
  }

  std::string target_frame;
  ros::param::param<std::string>("target_frame", target_frame, "base_link");

  perception_experiment::SurfaceViz viz(output_pub);

  Experiment experiment(argv[1], target_frame, input_pub, viz);
  ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "cloud_in", 1, &Experiment::Callback, &experiment);

  while (!experiment.get_is_done() && ros::ok()) {
    ros::spinOnce();
  }
}
