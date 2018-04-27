#ifndef _PERCEPTION_EXPERIMENT_EXP_ALGORITHM
#define _PERCEPTION_EXPERIMENT_EXP_ALGORITHM

#include <vector>

#include "pcl/PointIndices.h"
#include "ros/ros.h"
#include "surface_perception/typedefs.h"

namespace perception_experiment {
class ExpAlgorithm {
 public:
  ExpAlgorithm();
  virtual void SetInputCloud(PointCloudC::Ptr input_cloud);
  virtual void SetParameters();
  virtual void RunAlgorithm(std::vector<PointCloudC::Ptr>* cloud_vec,
                            ros::WallDuration* time_spent);
  virtual void GetInputCloud(PointCloudC::Ptr input_cloud);
};
}  // namespace perception_experiment

#endif  // _PERCEPTION_EXPERIMENT_EXP_ALGORITHM
