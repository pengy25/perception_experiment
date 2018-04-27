#ifndef _PERCEPTION_EXPERIMENT_NEW_ALGORITHM_
#define _PERCEPTION_EXPERIMENT_NEW_ALGORITHM_

#include "perception_experiment/exp_algorithm.h"
#include "ros/ros.h"
#include "surface_perception/surface_finder.h"
#include "surface_perception/typedefs.h"

namespace perception_experiment {
class NewAlgorithm : public ExpAlgorithm {
 public:
  NewAlgorithm();
  void SetInputCloud(PointCloudC::Ptr input_cloud);
  void SetParameters();
  void RunAlgorithm(std::vector<PointCloudC::Ptr>* cloud_vec,
                    ros::WallDuration* time_spent);
  void GetInputCloud(PointCloudC::Ptr input_cloud);

 private:
  surface_perception::SurfaceFinder algo_;
  PointCloudC::Ptr uncropped_cloud_;
  PointCloudC::Ptr cropped_cloud_;
};
}  // namespace perception_experiment

#endif  // _PERCEPTION_EXPERIMENT_NEW_ALGORITHM_
