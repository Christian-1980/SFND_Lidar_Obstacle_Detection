/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// for etsimation how long this process takes
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult; // a set always has unique elements -> no identicals coming from rand
	srand(time(NULL));
	
	while (maxIterations--) {
        std::unordered_set<int> temp_result; // create a temporary result to store and compare the size

        //UDACITY solution
		//while (temp_result.size() < 2) // just draw 2 points from the cloud randomlyinliersResult
        //    temp_result.insert(rand()%(cloud->points.size()));

		// 2 Point form a line
		pcl::PointXYZ point_1 = cloud -> points.at(rand()%(cloud->points.size()));
		pcl::PointXYZ point_2 = cloud -> points.at(rand()%(cloud->points.size()));

		// Define helpers A,B,C
        float A, B, C;

        A = point_1.y - point_2.y;
        B = point_2.x - point_1.x;
        C = point_1.x*point_2.y - point_2.x*point_1.y;

        for (int index = 0; index < cloud->points.size(); index++) {
            // Skip the two points that are sample points; if the count is larger than 0 then it is already in the set!
            if (temp_result.count(index) > 0)
                continue;

			// the point to check whether it is in the line
            pcl::PointXYZ point_3 = cloud->points[index];

            float d = fabs(A*point_3.x+B*point_3.y+C)/sqrt(A*A+B*B);
            if (d <= distanceTol) {
                temp_result.insert(index);
            }
        }
        if (temp_result.size() > inliersResult.size()) {
            inliersResult = temp_result;
        }
    }

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << "ms" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// for etsimation how long this process takes
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult; // a set always has unique elements -> no identicals coming from rand
	srand(time(NULL));
	
	while (maxIterations--) {
        std::unordered_set<int> temp_result; // create a temporary result to store and compare the size

        //UDACITY solution
		//while (temp_result.size() < 2) // just draw 2 points from the cloud randomlyinliersResult
        //    temp_result.insert(rand()%(cloud->points.size()));

		// 3 Point form a plane
		pcl::PointXYZ point_1 = cloud -> points.at(rand()%(cloud->points.size()));
		pcl::PointXYZ point_2 = cloud -> points.at(rand()%(cloud->points.size()));
		pcl::PointXYZ point_3 = cloud -> points.at(rand()%(cloud->points.size()));

		// Define helpers A,B,C
        float A, B, C, D;

        A = (point_2.y - point_1.y) * (point_3.z - point_1.z) - (point_2.z - point_1.z) * (point_3.y - point_1.y);
        B = (point_2.z - point_1.z) * (point_3.x - point_1.x) - (point_2.x - point_1.x) * (point_3.z - point_1.z);
        C = (point_2.x - point_1.x) * (point_3.y - point_1.y) - (point_2.y - point_1.y) * (point_3.x - point_1.x);
        D = -1 * (A * point_1.x + B * point_1.y + C * point_1.z);

        for (int index = 0; index < cloud->points.size(); index++) {
            // Skip the two points that are sample points; if the count is larger than 0 then it is already in the set!
            if (temp_result.count(index) > 0)
                continue;

			// the point to check whether it is in the line
            pcl::PointXYZ point_4 = cloud->points[index];

            float d = fabs(A * point_4.x + B * point_4.y + C * point_4.z + D) / sqrt(A * A + B * B + C * C);
            if (d <= distanceTol) {
                temp_result.insert(index);
            }
        }
        if (temp_result.size() > inliersResult.size()) {
            inliersResult = temp_result;
        }
    }

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << "ms" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 100, 1);
	std::unordered_set<int> inliers = RansacPlane(cloud, 1000, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
