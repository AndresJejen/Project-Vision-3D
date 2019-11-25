#ifndef APP_GROUND_BASED_PEOPLE_DETECTION_APP_H_
#define APP_GROUND_BASED_PEOPLE_DETECTION_APP_H_

#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <pcl/people/person_cluster.h>
#include <pcl/people/head_based_subcluster.h>
#include <pcl/people/person_classifier.h>


namespace app{
    template <typename PointT> class ObjectClassifierApp;

    template <typename PointT> 
    class ObjectClassifierApp{
        public:
            typedef pcl::PointCloud<PointT> PointCloud;
            typedef boost::shared_ptr<PointCloud> PointCloudPtr;
            typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
        
            /** \brief Constructor. */
            ObjectClassifierApp();

            /** \brief Destructor. */
            virtual ~ObjectClassifierApp();

            /**
             * \brief Set the pointer to the input cloud.
             *
             * \param[in] cloud A pointer to the input cloud.
             */
            void
            setInputCloud (PointCloudPtr& cloud);

            /**
             * \brief Set the ground coefficients.
             *
             * \param[in] ground_coeffs Vector containing the four plane coefficients.
             */
            void
            setGround (Eigen::VectorXf& ground_coeffs);

            /**
             * \brief Set voxel size. 
             *
             * \param[in] voxel_size Value of the voxel dimension (default = 0.06m.).
             */
            void
            setVoxelSize (float voxel_size);

            /**
             * \brief Set intrinsic parameters of the RGB camera.
             *
             * \param[in] intrinsics_matrix RGB camera intrinsic parameters matrix.
             */
            void
            setIntrinsics (Eigen::Matrix3f intrinsics_matrix);

            // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
            /**
             * \brief Set SVM-based person classifier.
             *
             * \param[in] person_classifier Needed for people detection on RGB data.
             */
            void
            setClassifier (app::PersonClassifier<pcl::RGB> person_classifier);

            /**
             * \brief Set minimum and maximum allowed height and width for a person cluster.
             *
             * \param[in] min_height Minimum allowed height for a person cluster (default = 1.3).
             * \param[in] max_height Maximum allowed height for a person cluster (default = 2.3).
             * \param[in] min_width Minimum width for a person cluster (default = 0.1).
             * \param[in] max_width Maximum width for a person cluster (default = 8.0).
             */
            void
            setPersonClusterLimits (float min_height, float max_height, float min_width, float max_width);

            /**
             * \brief Get floor coefficients.
             */
            Eigen::VectorXf
            getGround ();

            /**
             * \brief Get the filtered point cloud.
             */
            PointCloudPtr
            getFilteredCloud ();

            /**
             * \brief Get pointcloud after voxel grid filtering and ground removal.
             */
            PointCloudPtr
            getNoGroundCloud ();

            /**
             * \brief Extract RGB information from a point cloud and output the corresponding RGB point cloud.
             *
             * \param[in] input_cloud A pointer to a point cloud containing also RGB information.
             * \param[out] output_cloud A pointer to a RGB point cloud.
             */
            void
            extractRGBFromPointCloud (PointCloudPtr input_cloud, pcl::PointCloud<pcl::RGB>::Ptr& output_cloud);

            /**
             * \brief Estimates min_points_ and max_points_ based on the minimal and maximal cluster size and the voxel size.
             */
            void
            updateMinMaxPoints ();

            /**
             * \brief Applies the transformation to the ground plane.
             */
            void
            applyTransformationGround ();

            /**
             * \brief Applies the transformation to the intrinsics matrix.
             */
            void
            applyTransformationIntrinsics ();

            /**
             * \brief Reduces the input cloud to one point per voxel and limits the field of view.
             */
            void
            filter ();

            /**
             * \brief Perform people detection on the input data and return people clusters information.
             * 
             * \param[out] clusters Vector of PersonCluster.
             * 
             * \return true if the compute operation is successful, false otherwise.
             */
            bool
            compute (std::vector<app::PersonCluster<PointT> >& clusters);

        protected:
            /** \brief voxel size */
            float voxel_size_;                  
            
            /** \brief ground plane coefficients */
            Eigen::VectorXf ground_coeffs_;

            /** \brief flag stating whether the ground coefficients have been set or not */
            bool ground_coeffs_set_;

            /** \brief the transformed ground coefficients */
            Eigen::VectorXf ground_coeffs_transformed_;

            /** \brief ground plane normalization factor */
            float sqrt_ground_coeffs_;

            /** \brief pointer to the input cloud */
            PointCloudPtr cloud_;

            /** \brief pointer to the filtered cloud */
            PointCloudPtr cloud_filtered_;

            /** \brief pointer to the cloud after voxel grid filtering and ground removal */
            PointCloudPtr no_ground_cloud_; 

            /** \brief pointer to a RGB cloud corresponding to cloud_ */
            pcl::PointCloud<pcl::RGB>::Ptr rgb_image_;      
            
            /** \brief person clusters maximum height from the ground plane */
            float max_height_;                  
            
            /** \brief person clusters minimum height from the ground plane */
            float min_height_;

            /** \brief person clusters maximum width, used to estimate how many points maximally represent a person cluster */
            float max_width_;

            /** \brief person clusters minimum width, used to estimate how many points minimally represent a person cluster */
            float min_width_;
             
            /** \brief maximum number of points for a person cluster */
            int max_points_;                  
            
            /** \brief minimum number of points for a person cluster */
            int min_points_;       

            /** \brief minimum distance between persons' heads */
            float heads_minimum_distance_;      

            /** \brief intrinsic parameters matrix of the RGB camera */
            Eigen::Matrix3f intrinsics_matrix_;

            /** \brief flag stating whether the intrinsics matrix has been set or not */
            bool intrinsics_matrix_set_;     

            /** \brief the transformed intrinsics matrix */
            Eigen::Matrix3f intrinsics_matrix_transformed_;

            // ->>>>>>> CLASIFICADOR OJO ACA   
            /** \brief SVM-based person classifier */
            // app::PersonClassifier<pcl::RGB> person_classifier_;  

            /** \brief flag stating if the classifier has been set or not */
            bool person_classifier_set_flag_;   
    }
}

#include "./imp/objectdetector.hpp" // ACA RUTA DE LA IMPLEMENTACION OJO
#endif /* APP_GROUND_BASED_PEOPLE_DETECTION_APP_H_ */