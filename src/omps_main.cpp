#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/typedefs.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#include "pcl/features/normal_3d.h"
#include "pcl/segmentation/organized_multi_plane_segmentation.h"

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
  if (!ros::param::has("search_radius")) {
    ros::param::set("search_radius", 0.03);
  }

  return;
}
}  // Anonymous namespace

class Experiment {
 public:
  Experiment(const std::string& target_frame, const ros::Publisher& input_pub,
             const ros::Publisher& output_pub);
  void Callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
  bool get_is_done();
  double get_failure_rate();

 private:
  ros::Publisher input_pub_;
  ros::Publisher output_pub_;
  std::string target_frame_;
  tf::TransformListener tf_listener_;
  size_t iterations_ran_;
  size_t failure_times_;
  size_t iteration_limit_;
  bool is_done_;
};

Experiment::Experiment(const std::string& target_frame,
                       const ros::Publisher& input_pub,
                       const ros::Publisher& output_pub)
    : input_pub_(input_pub),
      output_pub_(output_pub),
      target_frame_(target_frame),
      tf_listener_(),
      iteration_limit_(1000),
      iterations_ran_(0),
      failure_times_(0),
      is_done_(false) {}

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

  pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
  PointCloudC::Ptr cropped_cloud(new PointCloudC);
  sensor_msgs::PointCloud2 msg_out;

  pcl::CropBox<PointC> crop;
  crop.setInputCloud(pcl_cloud);

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
  crop.filter(*cropped_cloud);

  pcl::toROSMsg(*cropped_cloud, msg_out);
  input_pub_.publish(msg_out);

  double angle_tolerance_degree;
  ros::param::param("angle_tolerance_degree", angle_tolerance_degree, 10.0);
  double max_point_distance;
  ros::param::param("max_point_distance", max_point_distance, 0.015);
  int surface_point_threshold;
  ros::param::param("surface_point_threshold", surface_point_threshold, 5000);
  double search_radius;
  ros::param::param("search_radius", search_radius, 0.03);

  pcl::NormalEstimation<PointC, pcl::Normal> normal;
  normal.setInputCloud(pcl_cloud);
  pcl::search::KdTree<PointC>::Ptr kdtree(new pcl::search::KdTree<PointC>());
  normal.setSearchMethod(kdtree);
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
  normal.setRadiusSearch(search_radius);
  normal.compute(*normal_cloud);

  pcl::OrganizedMultiPlaneSegmentation<PointC, pcl::Normal, pcl::Label> seg;
  seg.setMinInliers(surface_point_threshold);
  seg.setAngularThreshold(pcl::deg2rad(angle_tolerance_degree));
  seg.setDistanceThreshold(max_point_distance);
  seg.setInputNormals(normal_cloud);
  seg.setInputCloud(pcl_cloud);
  seg.setIndices(point_indices);

  while (iterations_ran_ < iteration_limit_ && ros::ok()) {
    iterations_ran_++;
    std::vector<pcl::PlanarRegion<PointC>, Eigen::aligned_allocator<pcl::PlanarRegion<PointC> > > regions;
    seg.segmentAndRefine(regions);

    if (regions.size() != 3) {
      ROS_ERROR("Failed to find the correct number of surfaces!");
      failure_times_++;
    }

    PointCloudC::Ptr output_cloud(new PointCloudC);
    output_cloud->header.frame_id = target_frame_;

    for (size_t i = 0; i < regions.size(); i++) {
      Eigen::Vector4f model = regions[i].getCoefficients();
      PointCloudC::Ptr part_cloud(new PointCloudC);

      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      for (size_t j = 0; j < point_indices->indices.size(); j++) {
        int index = point_indices->indices[j];  // This is one index among the cropped indices
        const PointC& pt = pcl_cloud->points[index];
        double dist =
          fabs(model[0] * pt.x + model[1] * pt.y + model[2] * pt.z + model[3])
          / sqrt(pow(model[0], 2) + pow(model[1], 2) + pow(model[2], 2));
        if (dist < 0.01) {
          indices->indices.push_back(index);
        }
      }

      pcl::ExtractIndices<PointC> extract_indices;
      extract_indices.setInputCloud(pcl_cloud);
      extract_indices.setIndices(indices);
      extract_indices.filter(*part_cloud);

      *output_cloud += *part_cloud;
    }

    ROS_INFO("Found %ld surfaces at %ldth attempt",
             regions.size(), iterations_ran_);

    pcl::toROSMsg(*output_cloud, msg_out);
    output_pub_.publish(msg_out);
  }
  is_done_ = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_perception_experiment");
  ros::NodeHandle nh;
  ros::Publisher input_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 100, true);
  ros::Publisher output_pub =
      nh.advertise<sensor_msgs::PointCloud2>("detected_surface", 100, true);

  std::string target_frame("base_link");
  if (argc > 1) {
    target_frame = argv[1];
  }

  Experiment experiment(target_frame, input_pub, output_pub);
  ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "cloud_in", 1, &Experiment::Callback, &experiment);

  while (!experiment.get_is_done() && ros::ok()) {
    ros::spinOnce();
  }
  ROS_INFO("The test finishes with the failure rate of %f",
           experiment.get_failure_rate());
}
