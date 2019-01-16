#include "perception_experiment/visualization.h"

#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

#include "surface_perception/axes_marker.h"
#include "surface_perception/typedefs.h"

#include "rosbag/bag.h"

using visualization_msgs::Marker;

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

void SurfaceViz::Save(PointCloudC::Ptr cloud) {
  rosbag::Bag out_bag;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);

  out_bag.open("saved_exp_result.bag", rosbag::bagmode::Write);

  visualization_msgs::MarkerArray marker_array;
  ROS_INFO("Writing %ld markers in the scene", markers_.size());
  for (size_t i = 0; i < markers_.size(); i++) {
    marker_array.markers.push_back(markers_[i]);
  }

  out_bag.write("cloud", ros::Time::now(), cloud_msg);
  out_bag.write("markers", ros::Time::now(), marker_array);

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
