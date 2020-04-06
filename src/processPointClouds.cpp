// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>
#include "quiz/ransac/ransac3d.h"

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane

  // From the PCL tutorial
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Two point clouds - Obstacles (cars) and the road
  typename pcl::PointCloud<PointT>::Ptr obstacle_point_cloud{
      new pcl::PointCloud<PointT>};
  typename pcl::PointCloud<PointT>::Ptr road_point_cloud{
      new pcl::PointCloud<PointT>};

  // Set up the point cloud for the filtering object
  extract.setInputCloud(cloud);

  // Set up the indices
  extract.setIndices(inliers);

  // Extract the inliers
  extract.setNegative(false); // False means points belonging to inliers remain
                              // inliers are the points belonging to the road
  extract.filter(*road_point_cloud);

  // Extract the outliers
  extract.setNegative(
      true); // True means points belonging to outliers remain
             // outliers are the points not belonging to the road
  extract.filter(*obstacle_point_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obstacle_point_cloud, road_point_cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold, const bool useCustom) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in this function to find inliers for the cloud.

  // From the PCL tutorial
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

  if (!useCustom) {
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Set up the options
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud); // Set up the point cloud

    // Perform RANSAC to find the plane for the point cloud (i.e. the road)
    seg.segment(*inliers, *coefficients);
  } else { // Using custom implementation made for the quiz
    std::unordered_set<int> inliers_set =
        Ransac3D(cloud, maxIterations, distanceThreshold);
    for (const int index : inliers_set) {
      inliers->indices.push_back(index);
    }
  }

  // Report if there are no inliers
  if (inliers->indices.size() == 0) {
    std::cerr << "There were no inliers with this point cloud - plane "
                 "extraction is not possible"
              << std::endl;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}