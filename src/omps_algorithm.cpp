#include "perception_experiment/omps_algorithm.h"

#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "ros/ros.h"

#include "pcl/PointIndices.h"
#include "pcl/features/normal_3d.h"
#include "pcl/segmentation/organized_multi_plane_segmentation.h"

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
  if (!ros::param::has("search_radius")) {
    ros::param::set("search_radius", 0.03);
  }

  return;
}
}  // namespace

namespace perception_experiment {
OMPSAlgorithm::OMPSAlgorithm()
    : algo_(pcl::OrganizedMultiPlaneSegmentation<PointC, pcl::Normal,
                                                 pcl::Label>()),
      normal_(pcl::NormalEstimation<PointC, pcl::Normal>()),
      uncropped_cloud_(new PointCloudC),
      cropped_cloud_(new PointCloudC),
      point_indices_(new pcl::PointIndices) {
  ROS_INFO("Using OMPS");
}

void OMPSAlgorithm::SetInputCloud(PointCloudC::Ptr input_cloud) {
  *uncropped_cloud_ = *input_cloud;

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
  crop.filter(point_indices_->indices);
  crop.filter(*cropped_cloud_);

  algo_.setInputCloud(uncropped_cloud_);
  algo_.setIndices(point_indices_);
}

void OMPSAlgorithm::SetParameters() {
  double angle_tolerance_degree;
  ros::param::param("angle_tolerance_degree", angle_tolerance_degree, 10.0);
  double max_point_distance;
  ros::param::param("max_point_distance", max_point_distance, 0.015);
  int surface_point_threshold;
  ros::param::param("surface_point_threshold", surface_point_threshold, 5000);
  double search_radius;
  ros::param::param("search_radius", search_radius, 0.03);

  algo_.setMinInliers(surface_point_threshold);
  algo_.setAngularThreshold(angle_tolerance_degree * 0.017453);
  algo_.setDistanceThreshold(max_point_distance);

  normal_.setRadiusSearch(search_radius);
}

void OMPSAlgorithm::RunAlgorithm(
    std::vector<surface_perception::Surface>* surfaces,
    ros::WallDuration* time_spent) {
  normal_.setInputCloud(uncropped_cloud_);
  pcl::search::KdTree<PointC>::Ptr kdtree(new pcl::search::KdTree<PointC>());
  normal_.setSearchMethod(kdtree);
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(
      new pcl::PointCloud<pcl::Normal>);

  std::vector<pcl::PlanarRegion<PointC>,
              Eigen::aligned_allocator<pcl::PlanarRegion<PointC> > >
      regions;

  // Actually time the code
  ros::WallTime start = ros::WallTime::now();
  normal_.compute(*normal_cloud);
  algo_.setInputNormals(normal_cloud);
  algo_.segmentAndRefine(regions);
  ros::WallTime end = ros::WallTime::now();
  *time_spent = end - start;

  std::vector<pcl::ModelCoefficients> coeff_vec;
  std::vector<pcl::PointIndices::Ptr> indices_vec;

  for (size_t i = 0; i < regions.size(); i++) {
    Eigen::Vector4f model = regions[i].getCoefficients();
    pcl::ModelCoefficients coeff;
    coeff.values.resize(4);
    coeff.values[0] = model(0);
    coeff.values[1] = model(1);
    coeff.values[2] = model(2);
    coeff.values[3] = model(3);
    coeff_vec.push_back(coeff);

    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    for (size_t j = 0; j < point_indices_->indices.size(); j++) {
      int index =
          point_indices_
              ->indices[j];  // This is one index among the cropped indices
      const PointC& pt = uncropped_cloud_->points[index];
      double dist =
          fabs(model[0] * pt.x + model[1] * pt.y + model[2] * pt.z + model[3]) /
          sqrt(pow(model[0], 2) + pow(model[1], 2) + pow(model[2], 2));
      if (dist < 0.01) {
        indices->indices.push_back(index);
      }
    }
    indices_vec.push_back(indices);
  }

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

void OMPSAlgorithm::GetInputCloud(PointCloudC::Ptr input_cloud) {
  *input_cloud = *cropped_cloud_;
}

}  // namespace perception_experiment
