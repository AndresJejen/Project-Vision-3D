#ifndef OBJECT_DETECTION_APP_H_
#define OBJECT_DETECTION_APP_H_

#include <pcl/people/ground_based_people_detection_app.h> // ACA DEFINICIONES OJO


template <typename PointT>
app::ObjectClassifierApp<PointT>::ObjectClassifierApp ()
{
    rgb_image_ = pcl::PointCloud<pcl::RGB>::Ptr(new pcl::PointCloud<pcl::RGB>);
    // set default values for optional parameters:
    voxel_size_ = 0.06;
    min_height_ = 1.3;
    max_height_ = 2.3;
    min_width_ = 0.1;
    max_width_ = 8.0;
    heads_minimum_distance_ = 0.3;
    updateMinMaxPoints ();   

    // set flag values for mandatory parameters:
    sqrt_ground_coeffs_ = std::numeric_limits<float>::quiet_NaN();
    ground_coeffs_set_ = false;
    intrinsics_matrix_set_ = false;
    person_classifier_set_flag_ = false;
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::setInputCloud (PointCloudPtr& cloud)
{
  cloud_ = cloud;
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::setGround (Eigen::VectorXf& ground_coeffs)
{
  ground_coeffs_ = ground_coeffs;
  ground_coeffs_set_ = true;
  sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();
  applyTransformationGround();
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::setVoxelSize (float voxel_size)
{
  voxel_size_ = voxel_size;
  updateMinMaxPoints ();
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::setIntrinsics (Eigen::Matrix3f intrinsics_matrix)
{
  intrinsics_matrix_ = intrinsics_matrix;
  intrinsics_matrix_set_ = true;
  applyTransformationIntrinsics();
}

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
template <typename PointT> void
app::ObjectClassifierApp<PointT>::setClassifier (pcl::people::PersonClassifier<pcl::RGB> person_classifier)
{
  person_classifier_ = person_classifier;
  person_classifier_set_flag_ = true;
}

template<typename PointT> void 
app::ObjectClassifierApp<PointT>::updateMinMaxPoints ()
{
  min_points_ = (int) (min_height_ * min_width_ / voxel_size_ / voxel_size_);
  max_points_ = (int) (max_height_ * max_width_ / voxel_size_ / voxel_size_);
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::setPersonClusterLimits (float min_height, float max_height, float min_width, float max_width)
{
  min_height_ = min_height;
  max_height_ = max_height;
  min_width_ = min_width;
  max_width_ = max_width;
  updateMinMaxPoints ();
}

template <typename PointT> Eigen::VectorXf
app::ObjectClassifierApp<PointT>::getGround ()
{
  if (!ground_coeffs_set_)
  {
    PCL_ERROR ("[app::ObjectClassifierApp::getGround] Floor parameters have not been set or they are not valid!\n");
  }
  return (ground_coeffs_);
}

template <typename PointT> typename app::ObjectClassifierApp<PointT>::PointCloudPtr
app::ObjectClassifierApp<PointT>::getFilteredCloud ()
{
  return (cloud_filtered_);
}

template <typename PointT> typename app::ObjectClassifierApp<PointT>::PointCloudPtr
app::ObjectClassifierApp<PointT>::getNoGroundCloud ()
{
  return (no_ground_cloud_);
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::extractRGBFromPointCloud (PointCloudPtr input_cloud, pcl::PointCloud<pcl::RGB>::Ptr& output_cloud)
{
  // Extract RGB information from a point cloud and output the corresponding RGB point cloud  
  output_cloud->points.resize(input_cloud->height*input_cloud->width);
  output_cloud->width = input_cloud->width;
  output_cloud->height = input_cloud->height;

  pcl::RGB rgb_point;
  for (uint32_t j = 0; j < input_cloud->width; j++)
  {
    for (uint32_t i = 0; i < input_cloud->height; i++)
    { 
      rgb_point.r = (*input_cloud)(j,i).r;
      rgb_point.g = (*input_cloud)(j,i).g;
      rgb_point.b = (*input_cloud)(j,i).b;    
      (*output_cloud)(j,i) = rgb_point; 
    }
  }
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::applyTransformationGround ()
{
    ground_coeffs_transformed_ = ground_coeffs_;
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::applyTransformationIntrinsics ()
{
    intrinsics_matrix_transformed_ = intrinsics_matrix_;
}

template <typename PointT> void
app::ObjectClassifierApp<PointT>::filter ()
{
  cloud_filtered_ = PointCloudPtr (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud(cloud_);
  grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  grid.setFilterFieldName("z");
  grid.setFilterLimits(min_fov_, max_fov_);
  grid.filter(*cloud_filtered_);
}

// :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
template <typename PointT> bool
app::ObjectClassifierApp<PointT>::compute (std::vector<pcl::people::PersonCluster<PointT> >& clusters)
{
  // Check if all mandatory variables have been set:
  if (!ground_coeffs_set_)
  {
    PCL_ERROR ("[app::ObjectClassifierApp::compute] Floor parameters have not been set or they are not valid!\n");
    return (false);
  }
  if (cloud_ == NULL)
  {
    PCL_ERROR ("[app::ObjectClassifierApp::compute] Input cloud has not been set!\n");
    return (false);
  }
  if (!intrinsics_matrix_set_)
  {
    PCL_ERROR ("[app::ObjectClassifierApp::compute] Camera intrinsic parameters have not been set!\n");
    return (false);
  }
  if (!person_classifier_set_flag_)
  {
    PCL_ERROR ("[app::ObjectClassifierApp::compute] Person classifier has not been set!\n");
    return (false);
  }

  // Fill rgb image:
  rgb_image_->points.clear();                            // clear RGB pointcloud
  extractRGBFromPointCloud(cloud_, rgb_image_);          // fill RGB pointcloud

  applyTransformationPointCloud();

  filter();

  // Ground removal and update:
  pcl::IndicesPtr inliers(new std::vector<int>);
  boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_filtered_));
  ground_model->selectWithinDistance(ground_coeffs_transformed_, 2 * voxel_size_, *inliers);
  no_ground_cloud_ = PointCloudPtr (new PointCloud);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered_);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud_);
  if (inliers->size () >= (300 * 0.06 / voxel_size_ / std::pow (static_cast<double> (sampling_factor_), 2)))
    ground_model->optimizeModelCoefficients (*inliers, ground_coeffs_transformed_, ground_coeffs_transformed_);
  else
    PCL_INFO ("No groundplane update!\n");

  // Euclidean Clustering:
  std::vector<pcl::PointIndices> cluster_indices;
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(no_ground_cloud_);
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(2 * voxel_size_);
  ec.setMinClusterSize(min_points_);
  ec.setMaxClusterSize(max_points_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(no_ground_cloud_);
  ec.extract(cluster_indices);

  // ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  // Head based sub-clustering //
  pcl::people::HeadBasedSubclustering<PointT> subclustering;
  subclustering.setInputCloud(no_ground_cloud_);
  subclustering.setGround(ground_coeffs_transformed_);
  subclustering.setInitialClusters(cluster_indices);
  subclustering.setHeightLimits(min_height_, max_height_);
  subclustering.setMinimumDistanceBetweenHeads(heads_minimum_distance_);
  subclustering.setSensorPortraitOrientation(false);
  subclustering.subcluster(clusters);

  // Person confidence evaluation with HOG+SVM:
  // ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
  for(typename std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    //Evaluate confidence for the current PersonCluster:
    Eigen::Vector3f centroid = intrinsics_matrix_transformed_ * (it->getTCenter());
    centroid /= centroid(2);
    Eigen::Vector3f top = intrinsics_matrix_transformed_ * (it->getTTop());
    top /= top(2);
    Eigen::Vector3f bottom = intrinsics_matrix_transformed_ * (it->getTBottom());
    bottom /= bottom(2);
    it->setPersonConfidence(person_classifier_.evaluate(rgb_image_, bottom, top, centroid, vertical_));
  }
 
  return (true);
}

template <typename PointT>
app::ObjectClassifierApp<PointT>::~GroundBasedPeopleDetectionApp ()
{
  // TODO Auto-generated destructor stub
}
#endif /* PCL_PEOPLE_GROUND_BASED_PEOPLE_DETECTION_APP_HPP_ */
