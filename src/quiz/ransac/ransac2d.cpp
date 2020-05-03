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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	
	srand(time(NULL));

	// For max iterations 
    for (int i = 0; i < maxIterations; i++) {

        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;

		//*****Creation of two arrays to host the points that define the line *****//
        float x[2], y[2];

        int j = 0;

		//*****Check that we are not picking the same point several times *****//
		
        while(inliers.size() < 2) {

            auto index = rand() % ( cloud->points.size() );

            x[j] = cloud -> points[index].x;
            y[j] = cloud -> points[index].y;

            if ( inliers.count (index ) == 0 ) {
                inliers.insert(index);
                j++;
            }
        }

        float a = y[0] - y[1];
        float b = x[1] - x[0];
        float c = x[0] * y[1] - x[1] * y[0];
        float dist = sqrt(a*a + b*b);

		//***** Measuring the distance between points and fitted line *****//
		
        for ( int index = 0; index < cloud -> points.size(); index++ ) {
            
			if ( inliers.count(index) > 0 )    continue;


            auto point = cloud->points[index];
            float x = point.x;
            float y = point.y;

            float d = fabs( a*x + b*y + c ) / dist;
            //***** Verification: if the distance is less than the tolerance, it becomes an inlier *****//

            if ( d <= distanceTol ) inliers.insert ( index );
            
        }

        // Return indices of inliers from fitted line with most inliers

        if ( inliers.size() > inliersResult.size() ) inliersResult = inliers;

    }

    return inliersResult;

}
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
    srand(time(NULL));

    for (int i = 0; i < maxIterations; i++) {
        
        std::unordered_set<int> inliers;

        int dim = 3;
        float x[dim], y[dim], z[dim];
        int j = 0;
        // avoid picking the same point twice
        while(inliers.size() < dim) {
            
            auto index = rand() % (cloud->points.size());

            x[j] = cloud->points[index].x;
            y[j] = cloud->points[index].y;
            z[j] = cloud->points[index].z;

            if (0 == inliers.count(index)) {

                inliers.insert(index);
                j++;
            }
        }


        float v0[3], v1[3];

        v0[0] = {x[1] - x[0]}; v0[1]= {y[1] - y[0]}; v0[2]={z[1] - z[0]};
        v1[0] = {x[2] - x[0]}; v1[1]= {y[2] - y[0]}; v1[2]={z[2] - z[0]};

        float a , b , c , d, dist;

        a = v0[1] * v1[2] - v0[2] * v1[1];
        b = v0[2] * v1[0] - v0[0] * v1[2];
        c = v0[0] * v1[1] - v0[1] * v1[0];
        d = -(a * x[0] + b * y[0] + c * z[0]);
        
        dist = sqrt(a*a + b*b + c*c);

        for (int index = 0; index < cloud->points.size(); index++) {
            
            if (inliers.count(index) > 0) continue;

            auto point = cloud->points[index];
            float x0 = point.x;
            float y0 = point.y;
            float z0 = point.z;

            float distance = fabs(a*x0 + b*y0 + c*z0 + d) / dist;
            
            if (distance <= distanceTol)       inliers.insert(index);
            
        }

        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 20, 0.4);

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
