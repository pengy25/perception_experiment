#ifndef _PERCEPTION_EXPERIMENT_RANSAC_ALGORITHM_
#define _PERCEPTION_EXPERIMENT_RANSAC_ALGORITHM_

#include "perception_experiment/exp_algorithm.h"
#include "ros/ros.h"
#include "surface_perception/surface_finder.h"
#include "surface_perception/typedefs.h"

#include "pcl/segmentation/sac_segmentation.h"

namespace perception_experiment {
class RANSACAlgorithm : public ExpAlgorithm {
 public:
  RANSACAlgorithm();
  void SetInputCloud(PointCloudC::Ptr input_cloud);
  void SetParameters();
  void RunAlgorithm(std::vector<PointCloudC::Ptr>* cloud_vec,
                    ros::WallDuration* time_spent);
  void GetInputCloud(PointCloudC::Ptr input_cloud);

 private:
  pcl::SACSegmentation<PointC> algo_;
  PointCloudC::Ptr uncropped_cloud_;
  PointCloudC::Ptr cropped_cloud_;
};
}  // namespace perception_experiment

#endif  // _PERCEPTION_EXPERIMENT_RANSAC_ALGORITHM_
