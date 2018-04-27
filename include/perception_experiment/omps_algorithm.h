#ifndef _PERCEPTION_EXPERIMENT_OMPS_ALGORITHM_
#define _PERCEPTION_EXPERIMENT_OMPS_ALGORITHM_

#include "perception_experiment/exp_algorithm.h"
#include "pcl/segmentation/organized_multi_plane_segmentation.h"
#include "pcl/PointIndices.h"
#include "pcl/features/normal_3d.h"

namespace perception_experiment {
class OMPSAlgorithm: public ExpAlgorithm {
public:
 OMPSAlgorithm();
 void SetInputCloud(PointCloudC::Ptr input_cloud);
 void SetParameters();
 void RunAlgorithm(std::vector<PointCloudC::Ptr>* cloud_vec,
                   ros::WallDuration* time_spent);
 void GetInputCloud(PointCloudC::Ptr input_cloud);

private:
 pcl::OrganizedMultiPlaneSegmentation<PointC, pcl::Normal, pcl::Label> algo_;
 pcl::NormalEstimation<PointC, pcl::Normal> normal_;
 PointCloudC::Ptr uncropped_cloud_;
 PointCloudC::Ptr cropped_cloud_;
 pcl::PointIndices::Ptr point_indices_;
};
}  // perception_experiment namespace

#endif  // _PERCEPTION_EXPERIMENT_OMPS_ALGORITHM_
