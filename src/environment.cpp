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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer){


    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    constexpr double slope = 0;
    std::shared_ptr<Lidar> lidar_ptr = std::make_shared <Lidar> ( cars , slope );
    auto input_cloud = lidar_ptr -> scan();

    ProcessPointClouds<pcl::PointXYZ> pointCloud;
    constexpr int maxIterations = 100;
    constexpr float distThreshold = 0.2;
    auto segmentCloud = pointCloud.SegmentPlane ( input_cloud , maxIterations, distThreshold );

    renderPointCloud ( viewer, segmentCloud.first, "obstacleCloud", Color (1, 0, 0) );
    renderPointCloud ( viewer, segmentCloud.second, "groundCloud", Color (0, 0, 0) );

}


void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (255, 255, 255);
    viewer->initCameraParameters();

    
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


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI> pointProcessor,pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud){


    constexpr float resolucion = 0.2;
    const Eigen::Vector4f minimo (-50, -6.0, -3, 1);
    const Eigen::Vector4f maximo (60, 6.5, 4, 1);
    auto filteredCloud = pointProcessor.FilterCloud(inputCloud, resolucion, minimo, maximo );

    constexpr int iteracionesMax = 100;
    constexpr float distThreshold = 0.2;
    auto segmentCloud = pointProcessor.SegmentPlane ( filteredCloud , iteracionesMax, distThreshold );

    renderPointCloud ( viewer , segmentCloud.second , "groundCloud", Color(0.5, 0.5, 0.5) );

    constexpr float clusteringTol = 0.35;
    constexpr int minimoDim = 15;
    constexpr int maximoDim = 600;
    auto cloudClusters = pointProcessor.Clustering ( segmentCloud.first, clusteringTol, minimoDim, maximoDim );

    int clusterID = 1;
    std::vector<Color> colors = { Color ( 1, 0, 0 ) , Color ( 0, 0, 1 ) , Color ( 0.5, 0, 1 ) };
    int num_of_colors = colors.size();

    Box myCar = {-1.5, -1.5, -1, 2.5, 1.5, -0.5};
    renderBox ( viewer, myCar, 0, Color ( 0, 1, 0 )  , 0.8 ) ;

    constexpr float boxHeight = 0.75;

    for ( const auto & cluster : cloudClusters ) {
        
        std::cout << "cluster size ";
        pointProcessor.numPoints ( cluster ) ;

        renderPointCloud ( viewer, cluster, "ObstacleCloud" + std::to_string ( clusterID ), colors [ clusterID % num_of_colors ] );

        Box box = pointProcessor.BoundingBox ( cluster );
        
        if (box.z_max - box.z_min >= boxHeight || cluster -> points.size() >= minimoDim * 2)     renderBox(viewer, box, clusterID);

        clusterID++;
    }
}


int main (int argc, char** argv) {

    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

//    simpleHighway(viewer);
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce();
//    }

    ProcessPointClouds<pcl::PointXYZI> point_cloud_processor;
    std::vector<boost::filesystem::path> stream = point_cloud_processor.streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    while (!viewer->wasStopped()) {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        input_cloud = point_cloud_processor.loadPcd((*stream_iterator).string());
        cityBlock(viewer, point_cloud_processor, input_cloud);

        stream_iterator++;
        // keep looping
        if(stream_iterator == stream.end())
            stream_iterator = stream.begin();

        // viewer spin
        viewer->spinOnce();
    } 
}