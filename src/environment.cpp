/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr& viewer) {
  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  const bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // SPECIFY FLAG TO ENABLE CUSTOM IMPLEMENTATION OF CLUSTERING
  // AND ROAD SEGMENTATION
  const bool useCustom = true;

  // TODO:: Create lidar sensor
  Lidar* lidar = new Lidar(cars, 0);

  // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> point_processor;

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = lidar->scan();

  // Lesson 1 - render rays and point cloud
  // renderRays(viewer, lidar->position, point_cloud);
  // renderPointCloud(viewer, point_cloud, "pointcloud", Color(0, 1, 0));

  // Lesson 2 - Show segmented point cloud
  // Changed minimum distance from 0.2 to 0.3
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segment_cloud =
          point_processor.SegmentPlane(point_cloud, 100, 0.3, useCustom);

  // Comment out to remove road and obstacle point clouds
  renderPointCloud(viewer, segment_cloud.first, "obstCloud", Color(1, 0, 0));
  renderPointCloud(viewer, segment_cloud.second, "planeCloud", Color(0, 1, 0));

  // Lesson 3 - from Euclidean clustering with PCL video
  // Changed cluster tolerance from 1.0 to 2.0
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters =
      point_processor.Clustering(segment_cloud.first, 2.0, 3, 30, useCustom);

  int cluster_id = 0;
  std::vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1),
                               Color(1, 0, 1)};  // Changed colours

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloud_clusters) {
    std::cout << "cluster size ";
    point_processor.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(cluster_id),
                     colors[cluster_id]);
    // Added from Lesson 3 - draw bounding boxes for obstacle point cloud
    Box box = point_processor.BoundingBox(cluster);
    renderBox(viewer, box, cluster_id);
    ++cluster_id;
  }

  delete lidar;
  lidar = nullptr;
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr& viewer) {
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS) viewer->addCoordinateSystem(1.0);
}

int main(int argc, char** argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  simpleHighway(viewer);

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}