#include "processPointClouds.h"

template<typename PointT>

void ProcessPointClouds<PointT>::numPoints(const typename pcl::PointCloud<PointT>::Ptr& cloud) {

    std::cout << cloud->points.size() << std::endl;

}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud,float filterRes, const Eigen::Vector4f& minPoint, const Eigen::Vector4f& maxPoint) {
    
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());
    
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*filteredCloud);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*filteredCloud);

    std::vector<int> indices;
    region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    region.setInputCloud(filteredCloud);
    region.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int index : indices)  inliers->indices.push_back(index);


    // Create the filtering object
    pcl::ExtractIndices<PointT> extraction;
    // Extract the point cloud on roof
    extraction.setInputCloud(filteredCloud);
    extraction.setIndices(inliers);
    extraction.setNegative(true);
    extraction.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(const pcl::PointIndices::Ptr& inliers, const typename pcl::PointCloud<PointT>::Ptr& cloud) {
  
  
    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)    planeCloud->points.push_back(cloud->points[index]);
    
    pcl::ExtractIndices<PointT> extraction;
    extraction.setInputCloud(cloud);
    extraction.setIndices(inliers);
    extraction.setNegative(true);
    extraction.filter(*obstaclesCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceTol) {
    
    std::unordered_set<int> inliersResult;

    //  Creation of a random number generator to produce random numbers.
    // https://en.cppreference.com/w/cpp/numeric/random/random_device
    std::random_device rd;

    //Creation of a pseudo-random generator of 32-bit numbers.
    //http://www.cplusplus.com/reference/random/mt19937/ 
    std::mt19937 gen(rd());
    
    std::uniform_int_distribution<> dis(0 , cloud->points.size());

    for (int i = 0; i < maxIterations; i++) {

        std::unordered_set<int> inliers;

        constexpr int dim = 3;
        float x[dim], y[dim], z[dim];
        int j = 0;

        while(inliers.size() < dim) {

            auto index = dis(gen);
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

            if (inliers.count(index) > 0)  continue;

            auto point = cloud->points[index];
            float x0 = point.x;
            float y0 = point.y;
            float z0 = point.z;

            float distance = fabs(a*x0 + b*y0 + c*z0 + d) / dist;
            
            if (distance <= distanceTol)     inliers.insert(index);
            
        }

        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr& cloud, int maxIterations, float distanceThreshold) {

    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers_set = RansacPlane(cloud, maxIterations, distanceThreshold);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    for (auto i : inliers_set)  inliers->indices.push_back(i);

    if (inliers->indices.empty())   std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Plane segmentation took " << elapsedTime.count() << " milliseconds." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;

}


template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int index, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol) {

    processed[index] = true;
    cluster.push_back(index);

    std::vector<int> closest = tree->search(points[index], distanceTol);

    for (int id : closest) {

        if (!processed[id]) clusterHelper(id, points, cluster, processed, tree, distanceTol);

    }

}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol) {

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while (i < points.size()) {

        if (processed[i]) {
            i++;
            continue;
        }

        std::vector<int> cluster;
        clusterHelper(i, points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;

    }

    return clusters;

}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(const typename pcl::PointCloud<PointT>::Ptr& cloud, float clusterTolerance, int minSize, int maxSize) {
    
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    auto* tree = new KdTree;

    std::vector<std::vector<float>> points;

    for (int i=0; i< cloud->points.size(); i++) {

        std::vector<float> point({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
        points.push_back(point);
        tree->insert(points[i], i);

    }

    std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, clusterTolerance);

    for (const auto& getIndices : clusterIndices) {

        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>());

        for (const auto index : getIndices)     cloudCluster->points.push_back(cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        if (cloudCluster->width >= minSize && cloudCluster->width <= maxSize) clusters.push_back(cloudCluster);
        
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters." << std::endl;

    return clusters;

}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(const typename pcl::PointCloud<PointT>::Ptr& cluster) {
    
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box{};
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(const typename pcl::PointCloud<PointT>::Ptr& cloud, std::string file) {

    pcl::io::savePCDFileASCII(file, *cloud);
    std::cout << "Saved " << cloud->points.size () << " data points to " + file << std::endl;

}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {
    
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    //load the pcd file
    if (-1 == pcl::io::loadPCDFile<PointT> (file, *cloud)) {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cout << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;
    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    sort(paths.begin(), paths.end());

    return paths;
    
}