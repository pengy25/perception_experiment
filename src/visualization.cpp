#include "perception_experiment/visualization.h"

#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"

#include "surface_perception/axes_marker.h"
#include "surface_perception/typedefs.h"

#include <json.h>
#include "rosbag/bag.h"
#include "std_msgs/String.h"

using visualization_msgs::Marker;

namespace {
void get_new_algo_param_json(json_object* obj_ptr) {
  double angle_tolerance_degree, max_point_distance, failure_prob_threshold;
  int surface_point_threshold, min_iteration, max_surface_amount;

  ros::param::get("angle_tolerance_degree", angle_tolerance_degree);
  ros::param::get("max_point_distance", max_point_distance);
  ros::param::get("surface_point_threshold", surface_point_threshold);
  ros::param::get("min_iteration", min_iteration);
  ros::param::get("max_surface_amount", max_surface_amount);
  ros::param::get("failure_prob_threshold", failure_prob_threshold);

  json_object_object_add(obj_ptr, "angle_tolerance_degree",
                         json_object_new_double(angle_tolerance_degree));
  json_object_object_add(obj_ptr, "max_point_distance",
                         json_object_new_double(max_point_distance));
  json_object_object_add(obj_ptr, "surface_point_threshold",
                         json_object_new_int(surface_point_threshold));
  json_object_object_add(obj_ptr, "min_iteration",
                         json_object_new_int(min_iteration));
  json_object_object_add(obj_ptr, "max_surface_amount",
                         json_object_new_int(max_surface_amount));
  json_object_object_add(obj_ptr, "failure_prob_threshold",
                         json_object_new_double(failure_prob_threshold));

  return;
}

void get_omps_algo_param_json(json_object* obj_ptr) {
  double angle_tolerance_degree, max_point_distance, search_radius,
      max_point_plane_distance_difference, time_normal, time_set_normal,
      time_omps_alone;
  int surface_point_threshold;

  ros::param::get("angle_tolerance_degree", angle_tolerance_degree);
  ros::param::get("max_point_distance", max_point_distance);
  ros::param::get("surface_point_threshold", surface_point_threshold);
  ros::param::get("search_radius", search_radius);
  ros::param::get("max_point_plane_distance_difference",
                  max_point_plane_distance_difference);
  ros::param::get("time_normal", time_normal);
  ros::param::get("time_set_normal", time_set_normal);
  ros::param::get("time_omps_alone", time_omps_alone);

  json_object_object_add(obj_ptr, "angle_tolerance_degree",
                         json_object_new_double(angle_tolerance_degree));
  json_object_object_add(obj_ptr, "max_point_distance",
                         json_object_new_double(max_point_distance));
  json_object_object_add(obj_ptr, "surface_point_threshold",
                         json_object_new_int(surface_point_threshold));
  json_object_object_add(obj_ptr, "search_radius",
                         json_object_new_double(search_radius));
  json_object_object_add(
      obj_ptr, "max_point_plane_distance_difference",
      json_object_new_double(max_point_plane_distance_difference));
  json_object_object_add(obj_ptr, "time_normal",
                         json_object_new_double(time_normal));
  json_object_object_add(obj_ptr, "time_set_normal",
                         json_object_new_double(time_set_normal));
  json_object_object_add(obj_ptr, "time_omps_alone",
                         json_object_new_double(time_omps_alone));

  return;
}
}  // namespace

namespace perception_experiment {
SurfaceViz::SurfaceViz(const ros::Publisher& marker_pub)
    : marker_pub_(marker_pub), surfaces_(), markers_() {}

void SurfaceViz::set_surfaces(
    const std::vector<surface_perception::Surface>& surfaces) {
  surfaces_ = surfaces;
}

void SurfaceViz::Show() {
  Hide();
  SurfaceMarkers(surfaces_, &markers_);
  for (size_t i = 0; i < markers_.size(); ++i) {
    const Marker& marker = markers_[i];
    marker_pub_.publish(marker);
  }
}

void SurfaceViz::Hide() {
  for (size_t i = 0; i < markers_.size(); ++i) {
    Marker marker;
    marker.ns = markers_[i].ns;
    marker.id = markers_[i].id;
    marker.action = Marker::DELETE;
    marker_pub_.publish(marker);
  }
  markers_.clear();
}

void SurfaceViz::Save(const std::string& algo_name, size_t current_iteration,
                      PointCloudC::Ptr cloud) {
  rosbag::Bag out_bag;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);

  std::stringstream file_name_stream;
  file_name_stream << "saved_exp_result_";
  file_name_stream << current_iteration;
  file_name_stream << ".bag";

  out_bag.open(file_name_stream.str(), rosbag::bagmode::Write);

  visualization_msgs::MarkerArray marker_array;
  ROS_INFO("Writing %ld markers in the scene", markers_.size());
  for (size_t i = 0; i < markers_.size(); i++) {
    marker_array.markers.push_back(markers_[i]);
  }

  double crop_min_x, crop_min_y, crop_min_z, crop_max_x, crop_max_y, crop_max_z,
      time_algo_overall;

  ros::param::get("crop_min_x", crop_min_x);
  ros::param::get("crop_min_y", crop_min_y);
  ros::param::get("crop_min_z", crop_min_z);
  ros::param::get("crop_max_x", crop_max_x);
  ros::param::get("crop_max_y", crop_max_y);
  ros::param::get("crop_max_z", crop_max_z);
  ros::param::get("time_algo_overall", time_algo_overall);

  json_object* result_obj = json_object_new_object();
  json_object_object_add(result_obj, "crop_min_x",
                         json_object_new_double(crop_min_x));
  json_object_object_add(result_obj, "crop_min_y",
                         json_object_new_double(crop_min_y));
  json_object_object_add(result_obj, "crop_min_z",
                         json_object_new_double(crop_min_z));
  json_object_object_add(result_obj, "crop_max_x",
                         json_object_new_double(crop_max_x));
  json_object_object_add(result_obj, "crop_max_y",
                         json_object_new_double(crop_max_y));
  json_object_object_add(result_obj, "crop_max_z",
                         json_object_new_double(crop_max_z));
  json_object_object_add(result_obj, "time_algo_overall",
                         json_object_new_double(time_algo_overall));

  if (algo_name == "new") {
    get_new_algo_param_json(result_obj);
  } else if (algo_name == "omps") {
    get_omps_algo_param_json(result_obj);
  } else {
    ROS_ERROR("Error: unsupported algorithm: %s", algo_name.c_str());
  }

  std_msgs::String result_str;
  result_str.data = json_object_to_json_string(result_obj);

  out_bag.write("cloud", ros::Time::now(), cloud_msg);
  out_bag.write("markers", ros::Time::now(), marker_array);
  out_bag.write("params_in_json", ros::Time::now(), result_str);

  out_bag.close();
}

void SurfaceMarker(const surface_perception::Surface& surface,
                   visualization_msgs::Marker* marker) {
  marker->type = Marker::CUBE;
  marker->header = surface.pose_stamped.header;
  marker->pose = surface.pose_stamped.pose;
  marker->scale = surface.dimensions;
  marker->color.r = 1;
  marker->color.b = 1;
  marker->color.a = 0.5;
}

void SurfaceMarkers(const std::vector<surface_perception::Surface>& surfaces,
                    std::vector<visualization_msgs::Marker>* markers) {
  for (size_t surface_i = 0; surface_i < surfaces.size(); ++surface_i) {
    const surface_perception::Surface& surface = surfaces[surface_i];
    Marker surface_marker;
    SurfaceMarker(surface, &surface_marker);
    surface_marker.ns = "surface";
    surface_marker.id = surface_i;
    markers->push_back(surface_marker);

    std::stringstream axes_ns;
    axes_ns << surface_marker.ns << surface_i << "_axes";
    visualization_msgs::MarkerArray axesMarkers =
        surface_perception::GetAxesMarkerArray(
            axes_ns.str(), surface_marker.header.frame_id, surface_marker.pose,
            std::min(surface_marker.scale.x, surface_marker.scale.y) / 2.0);

    for (size_t axis_i = 0; axis_i < axesMarkers.markers.size(); ++axis_i) {
      markers->push_back(axesMarkers.markers[axis_i]);
    }
  }
}
}  // namespace perception_experiment
