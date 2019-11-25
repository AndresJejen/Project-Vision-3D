#ifndef APP_HEIGHT_MAP_2D_H_
#define APP_HEIGHT_MAP_2D_H_

#include <pcl/people/person_cluster.h>
#include <pcl/point_types.h>

namespace app
  {
    /** \brief @b HeightMap2D represents a class for creating a 2D height map from a point cloud and searching for its local maxima
      * \author Matteo Munaro
      * \ingroup people
    */
    template <typename PointT> class HeightMap2D;

    template <typename PointT>
    class HeightMap2D
    {
    public:

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef boost::shared_ptr<PointCloud> PointCloudPtr;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      /** \brief Constructor. */
      HeightMap2D();

      /** \brief Destructor. */
      virtual ~HeightMap2D ();

      /**
       * \brief Compute the height map with the projection of cluster points onto the ground plane.
       * 
       * \param[in] cluster The PersonCluster used to compute the height map.
       */
      void
      compute (app::PersonCluster<PointT>& cluster);

      /**
       * \brief Compute local maxima of the height map.
       */
      void
      searchLocalMaxima ();

      /**
       * \brief Filter maxima of the height map by imposing a minimum distance between them.
       */
      void
      filterMaxima ();

      /**
       * \brief Set initial cluster indices.
       * 
       * \param[in] cloud A pointer to the input cloud.
       */
      void
      setInputCloud (PointCloudPtr& cloud);

      /**
       * \brief Set the ground coefficients.
       * 
       * \param[in] ground_coeffs The ground plane coefficients.
       */
      void
      setGround (Eigen::VectorXf& ground_coeffs);

      /**
       * \brief Set bin size for the height map. 
       * 
       * \param[in] bin_size Bin size for the height map (default = 0.06).
       */
      void
      setBinSize (float bin_size);

      /**
       * \brief Set minimum distance between maxima. 
       * 
       * \param[in] minimum_distance_between_maxima Minimum allowed distance between maxima (default = 0.3).
       */
      void
      setMinimumDistanceBetweenMaxima (float minimum_distance_between_maxima);

      /**
       * \brief Set sensor orientation to landscape mode (false) or portrait mode (true).
       * 
       * \param[in] vertical Landscape (false) or portrait (true) mode (default = false).
       */
      void
      setSensorPortraitOrientation (bool vertical);

      /**
       * \brief Get the height map as a vector of int.
       */
      std::vector<int>&
      getHeightMap ();

      /**
       * \brief Get bin size for the height map. 
       */
      float
      getBinSize ();

      /**
       * \brief Get minimum distance between maxima of the height map. 
       */
      float
      getMinimumDistanceBetweenMaxima ();

      /**
       * \brief Return the maxima number after the filterMaxima method.
       */
      int&
      getMaximaNumberAfterFiltering ();

      /**
       * \brief Return the point cloud indices corresponding to the maxima computed after the filterMaxima method.
       */
      std::vector<int>&
      getMaximaCloudIndicesFiltered ();

    protected:
      /** \brief ground plane coefficients */
      Eigen::VectorXf ground_coeffs_;            
      
      /** \brief ground plane normalization factor */
      float sqrt_ground_coeffs_;              
      
      /** \brief pointer to the input cloud */
      PointCloudPtr cloud_;                
      
      /** \brief if true, the sensor is considered to be vertically placed (portrait mode) */
      bool vertical_;                    
      
      /** \brief vector with maximum height values for every bin (height map) */
      std::vector<int> buckets_;              
      
      /** \brief indices of the pointcloud points with maximum height for every bin */
      std::vector<int> buckets_cloud_indices_;      
      
      /** \brief bin dimension */
      float bin_size_;                  
      
      /** \brief number of local maxima in the height map */
      int maxima_number_;                  
      
      /** \brief contains the position of the maxima in the buckets vector */
      std::vector<int> maxima_indices_;          
      
      /** \brief contains the point cloud position of the maxima (indices of the point cloud) */
      std::vector<int> maxima_cloud_indices_;        
      
      /** \brief number of local maxima after filtering */
      int maxima_number_after_filtering_;          
      
      /** \brief contains the position of the maxima in the buckets array after filtering */
      std::vector<int> maxima_indices_filtered_;      
      
      /** \brief contains the point cloud position of the maxima after filtering */
      std::vector<int> maxima_cloud_indices_filtered_;  
      
      /** \brief minimum allowed distance between maxima */
      float min_dist_between_maxima_;            
    };

  } /* namespace people */
#include <pcl/people/impl/height_map_2d.hpp>
#endif /* APP_HEIGHT_MAP_2D_H_ */
