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
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanned_pointCloud = lidar->scan();
   
    //Render the laser beams
    //renderRays(viewer, lidar->position, scanned_pointCloud);

    // Render the resulting point cloud
    //renderPointCloud(viewer, scanned_pointCloud, "results");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor -> SegmentPlane(scanned_pointCloud, 100, 0.2);
    // // Rendering both plane and obstacles
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // Cluster the obstCloud which is the first segmented cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    std::vector<Color> render_colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    int ClusterId = 0;

    for (auto cluster : cloudClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud_" + std::to_string(ClusterId), render_colours[ClusterId]);

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
  
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //  renderPointCloud(viewer,inputCloud,"inputCloud");

  // 1. Downsampling
  pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud = pointProcessorI->FilterCloud(input_cloud, 0.2, Eigen::Vector4f(-15, -6.0, -3, 1), Eigen::Vector4f(30, 6.0, 10, 1));
  
  // 2. Segmentaion
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlane(sampled_cloud, 100, 0.2);
  // Rendering both plane and obstacles
  //renderPointCloud(viewer,segment_cloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segment_cloud.second,"planeCloud",Color(0,1,0));

  // 3. Clustering
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustered_cloud = pointProcessorI->Clustering(segment_cloud.first, 0.5, 10, 600);

  // 4. Rendering
  std::vector<Color> render_colours = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  int ClusterId = 0;

  for (auto cluster : clustered_cloud)
  {
      renderPointCloud(viewer, cluster, "obstCloud_" + std::to_string(ClusterId), render_colours[ClusterId]);  
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, ClusterId);  
      ++ClusterId;
  }

}


void cityBlockStream(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // downsampled cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr sampledCloud = pointProcessorI->FilterCloud(inputCloud, 0.18, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 4, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlane(sampledCloud, 100, 0.2);

    //renderPointCloud(viewer, segment_cloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segment_cloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustered_cloud = pointProcessorI->Clustering(segment_cloud.first, 0.44, 15, 700);

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
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // No stream
    //simpleHighway(viewer);
    //cityBlock(viewer);


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

        //std::this_thread::sleep_for(std::chrono::milliseconds(300));

        viewer->spinOnce ();
    } 
}