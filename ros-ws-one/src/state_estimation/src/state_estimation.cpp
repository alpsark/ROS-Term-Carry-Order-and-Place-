#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

class StateEstimation {
  ros::Subscriber cloudSubs;
  ros::Subscriber ballPoseSubs;
  ros::NodeHandle nh;

  tf::TransformListener tfListener;

  bool isAdded;
  
  pcl::visualization::PCLVisualizer* viewer;

 public:
  StateEstimation() {
    cloudSubs = nh.subscribe("/vrep/kinectPoints", 1,
                             &StateEstimation::pointCloudCallback, this);

    ballPoseSubs = nh.subscribe("/vrep/ballPose", 1,
                                &StateEstimation::ballPoseCallback, this);

    isAdded = false;

    viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
    viewer->initCameraParameters();
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addCoordinateSystem (1.0);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // wait from transform
    tfListener.waitForTransform("base_link", "kinect_depth", ros::Time::now(),
                                ros::Duration(1.0));

    // convert point cloud frame to robot frame
    sensor_msgs::PointCloud2 transformedPoints;
    pcl_ros::transformPointCloud("base_link", *msg, transformedPoints,
                                 tfListener);
    pcl::PointCloud<pcl::PointXYZ> *pointCloud = new pcl::PointCloud<pcl::PointXYZ>();
    pcl::fromROSMsg(transformedPoints, *pointCloud);

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr constPointCloud(pointCloud);

      viewer->removePointCloud("cloud");
      viewer->addPointCloud(constPointCloud, "cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
      
     viewer->spinOnce(100);

    // print point values for debugging purposes
//    std::cout << "size=" << pointCloud.points.size() << std::endl;
//    for (int i = 0; i < pointCloud.points.size(); i++) {
//      std::cout << "p[" << i << "].x=" << pointCloud.points.at(i).x << " .y="
//          << pointCloud.points.at(i).y << " .z=" << pointCloud.points.at(i).z
//          << std::endl;
//    }

    // TODO: detect ball here

    // TODO: apply kalman filter here

  }

  // get the ground truth position of the ball for debugging and plotting
  void ballPoseCallback(const geometry_msgs::PoseStamped& ballPose) {
    std::cout << "real ball.x=" << ballPose.pose.position.x << " ball.y="
              << ballPose.pose.position.y << std::endl;

  }
};

int main(int argc, char **argv) {
  std::cout << "StateEstimation Start..." << std::endl;
  ros::init(argc, argv, "StateEstimation");
  StateEstimation stateEstimation;
  ros::spin();
  printf("StateEstimation Finish...");
}
