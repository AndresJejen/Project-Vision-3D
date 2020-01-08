// Importacion de librerias

#include <pcl/console/parse.h>  // Para leer las variables en la ejecución por terminal
#include <pcl/point_types.h>    // Tipos de datos
#include <pcl/visualization/pcl_visualizer.h>   // Visualizador  
#include <pcl/io/openni_grabber.h>              // Lector de Kinnect
#include <pcl/sample_consensus/sac_model_plane.h>   // Estimador del plano (Suelo)
#include <pcl/common/time.h>    // Tiempo

#include "./libraries/ground_based_object_detection.h"   // Libreria para encontrar personas
#include "./libraries/person_cluster.h"

#include<mutex>
#include<thread>
#include<iostream>

using namespace std::literals::chrono_literals;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr cloud (new PointCloudT)
bool new_cloud_available_flag = false;


// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
std::mutex cloud_mutex;

enum { COLS = 640, ROWS = 480 };

int print_help()
{
  std::cout << "*******************************************************" << std::endl;
  std::cout << "Ground based people detection app options:" << std::endl;
  std::cout << "   --help    <show_this_help>" << std::endl;
  std::cout << "   --resnet18     <path_to_resnet18_file>" << std::endl;
  std::cout << "   --conf    <minimum_HOG_confidence (default = -1.5)>" << std::endl;
  std::cout << "   --min_h   <minimum_person_height (default = 1.3)>" << std::endl;
  std::cout << "   --max_h   <maximum_person_height (default = 2.3)>" << std::endl;
  std::cout << "*******************************************************" << std::endl;
  return 0;
}

void
grabberCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& callback_cloud)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}


struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}


int main (int argc, char** argv) {

    if(pcl::console::find_switch (argc, argv, "--help") || pcl::console::find_switch (argc, argv, "-h"))
        return print_help();

    std::string resnetfilename = "./resnetpretrained.pt";
    float min_confidence = 0.6;
    float min_height = 1.3;
    float max_height = 2.3;
    float voxel_size = 0.06;
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

    pcl::console::parse_argument (argc, argv, "--min_h", min_height);
    pcl::console::parse_argument (argc, argv, "--max_h", max_height);

    // Read Kinect live stream:
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&grabberCallback, _1);
    interface->registerCallback (f);
    interface->start ();

    // Wait for the first frame:
    while(!new_cloud_available_flag) 
        std::this_thread::sleep_for(1ms);
    new_cloud_available_flag = false;

    cloud_mutex.lock ();    // for not overwriting the point cloud

    // Display pointcloud:
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
    viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    // Spin until 'Q' is pressed:
    viewer.spin();
    std::cout << "done." << std::endl;
    
    cloud_mutex.unlock ();

    // Ground plane estimation:
    Eigen::VectorXf ground_coeffs;
    ground_coeffs.resize(4);
    std::vector<int> clicked_points_indices;
    for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
        clicked_points_indices.push_back(i);
    pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
    model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
    std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

    // Initialize new viewer:
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

    // __________________________________________________________________________________________________________________________________-
    // DEFINICION DEL CLASIFICADOR
    // Carga de parametros para la clasificación
    pcl::people::ObjectClassifier<pcl::RGB> objectClassifier;
    objectClassifier.loadPytorchClassifier(resnetfilename);

    // DEFINICION DEL DETECTOR
    // Person cluster limits
    // Tamaño del voxel
    // Intrisecos del kinnect
    // ingreso del clasificador
    // People detection app initialization:
    pcl::people::GroundBasedPeopleDetectionApp<PointT> object_detector;    // object detection object
    object_detector.setVoxelSize(voxel_size);                        // set the voxel size
    object_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
    object_detector.setClassifier(person_classifier);                // set person classifier
    object_detector.setPersonClusterLimits(min_height, max_height, 0.1, 8.0);  // set person classifier
    //  people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical
    // ____________________________

    // For timing:
    static unsigned count = 0;
    static double last = pcl::getTime ();

    // Main loop:
    while (!viewer.wasStopped())
    {
        if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
        {
            new_cloud_available_flag = false;
        // Perform people detection on the new cloud:

        // Perform people detection on the new cloud:
        std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
        object_detector.setInputCloud(cloud);
        object_detector.setGround(ground_coeffs);                    // set floor coefficients
        object_detector.compute(clusters);                           // perform people detection

        ground_coeffs = people_detector.getGround();                 // get updated floor coefficients
        // Dicujar la caja para las personas
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
        viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
        unsigned int k = 0;

      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        if(it->getObjectConfidence()[1] > min_confidence)             // draw only people with confidence above a threshold
        {
          // draw theoretical person bounding box in the PCL viewer:
          it->drawTBoundingBoxObject(viewer, k, it->getObjectConfidence()[0]);
          k++;
        }
        
      }
        std::cout << k << " objects found" << std::endl;
        viewer.spinOnce();

        if (++count == 30)
        {
            double now = pcl::getTime ();
            std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
            count = 0;
            last = now;
        }
        cloud_mutex.unlock ();
        }
    }
    return 0;
}
