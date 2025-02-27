/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene,
                                        viewer);
    
    // Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.);

    // Create data point via a scan of the environment
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanned_pointCloud = lidar->scan();
   
    //Render the laser beams
    //renderRays(viewer, lidar->position, scanned_pointCloud);

    // Render the resulting point cloud
    //renderPointCloud(viewer, scanned_pointCloud, "results");

    // Create point processor by using a pointer
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor;

    // Separate the groundplane from the rest of the data
    // a) definition of the hyperparameters
    int iterations = 100;
    float distance = 0.2;

    // b) Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor -> SegmentPlane(scanned_pointCloud,
                                                                                                                                      iterations,
                                                                                                                                      distance);
    // c) Rendering both plane and obstacles
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));


    // Cluster the obstCloud which is the first segmented cloud
    // a) definition of the hyperparameters
    float cluster_tolerance = 1.0;
    int min_cluster_size = 3;
    int max_cluster_size = 30;

    // b) clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first,
                                                                                                cluster_tolerance,
                                                                                                min_cluster_size,
                                                                                                max_cluster_size);
    std::vector<Color> render_colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    // c) rednering the different clusters
    int ClusterId = 0;

    for (auto cluster : cloudClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud_" + std::to_string(ClusterId), render_colours[ClusterId]);
        // adding bounding boxes to the cluster of obstacles
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, ClusterId);

        ++ClusterId;
    }
  
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)

{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  
  int flag_pcl = 1; //1 = is used

  // create new instance of Porcess Point Cloud with intensities
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

  // load point cloud data
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000006.pcd");
  // // render data for check
  // renderPointCloud(viewer,inputCloud,"inputCloud");

  // 1. Downsampling
  // downsampling hyperparams
  float filter_resolution = 0.2;
  Eigen::Vector4f min_point (-10, -6.0, -2, 1);
  Eigen::Vector4f max_point (30, 6.0, 1, 1);

  pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud = pointProcessorI->FilterCloud(input_cloud,
                                                                                    filter_resolution,
                                                                                    min_point,
                                                                                    max_point);
  
  // 2. Segmentaion
//   int iterations = 100;
//   float distance = 0.2;
//   std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlane(sampled_cloud,
//                                                                                                                                       iterations,
//                                                                                                                                       distance);
  int iterations = 30;
  float distance = 0.2;
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlaneRansac3D(sampled_cloud,
                                                                                                                                               iterations,
                                                                                                                                               distance);
  
  renderPointCloud(viewer,segment_cloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segment_cloud.second,"planeCloud",Color(0,1,0));

  // 3. Clustering
  // a) definition of the hyperparameters
  float cluster_tolerance = 0.5;
  int min_cluster_size = 5;
  int max_cluster_size = 600;

  // b) clustering
//   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustered_cloud = pointProcessorI->Clustering(segment_cloud.first,
//                                                                                                  cluster_tolerance,
//                                                                                                   min_cluster_size,
//                                                                                                   max_cluster_size );
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustered_cloud = pointProcessorI->EuclideanCluster(segment_cloud.first,
                                                                                                   cluster_tolerance,
                                                                                                   min_cluster_size,
                                                                                                   max_cluster_size);

  // 4. Rendering and bounding boxes
  std::vector<Color> render_colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  int ClusterId = 0;

  for (auto cluster : clustered_cloud)
  {
      std::cout << "Cluster " << std::to_string(ClusterId) << " has a size of ";
      pointProcessorI->numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstCloud_" + std::to_string(ClusterId), render_colours[ClusterId]);  
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, ClusterId);  
      ++ClusterId;
  }

}


void cityBlockStream(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // 1. Downsampling
    // downsampling hyperparams
    float filter_resolution = 0.2;
    Eigen::Vector4f min_point (-15, -6.0, -3, 1);
    Eigen::Vector4f max_point (30, 6.0, 10, 1);

    pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud = pointProcessorI->FilterCloud(inputCloud, 0.18, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 4, 1));

    // 2. Segmentation of the ground plane
    // a) definition of the hyperparameters
    int iterations = 20;
    float distance = 0.2;

    // b) Segmentation
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlane(sampledCloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlaneRansac3D(sampledCloud,
                                                                                                                                                iterations,
                                                                                                                                                distance);
    // c) render cloud
    renderPointCloud(viewer, segment_cloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segment_cloud.second, "planeCloud", Color(0,1,0));


    // a) definition of the hyperparameters
    float cluster_tolerance = 0.5;
    int min_cluster_size = 5;
    int max_cluster_size = 500;

    // b) clustering
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustered_cloud = pointProcessorI->Clustering(segment_cloud.first,
    //                                                                                                 cluster_tolerance,
    //                                                                                                 min_cluster_size,
    //                                                                                                 max_cluster_size);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustered_cloud = pointProcessorI->EuclideanCluster(segment_cloud.first,
                                                                                                    cluster_tolerance,
                                                                                                    min_cluster_size,
                                                                                                    max_cluster_size);
    // 4. Rendering and bounding boxes
    int ClusterId = 0;

    std::vector<Color> render_colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clustered_cloud)
    {
        std::cout << "Cluster " << std::to_string(ClusterId) << " has a size of ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstcleCloud" + std::to_string(ClusterId), render_colors[ClusterId]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, ClusterId);

        ++ClusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    bool city_block = true;
    bool stream_city_block = true;
    bool simple_highway = false;

    
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // No stream
    if (stream_city_block!=true)
    {
        if (simple_highway==true){
            simpleHighway(viewer);
        } else if (city_block==true) {
            cityBlock(viewer);
        }
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        } 

    } else {
    // Stream
    //Definitions outside the loop
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        cityBlockStream(viewer, pointProcessorI, inputCloudI);
        
        streamIterator++;
        
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
    }
}