#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

void getRandomPointCloud(sensor_msgs::PointCloud2& pc, float center_x, float center_y, int& size_of_cloud) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dist_x(center_x, 2.0f);
  std::normal_distribution<float> dist_y(center_y, 2.0f);

  sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pc, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pc, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pc, "b");

  for (int i = 0; i < size_of_cloud; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
    float x_value = dist_x(gen);
    float y_value = dist_y(gen);
    *(iter_x)     = x_value;
    *(iter_y)     = y_value;
    *(iter_z)     = std::exp(-((x_value * x_value) + (y_value * y_value)) / 4.0f);

    int color = i % 32;
    *(iter_r) = (color * 16) & 0x0000ff;
    *(iter_g) = (color * 8) & 0x0000ff;
    *(iter_b) = (color * 4) & 0x0000ff;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_pcl_tutorial");
  auto nh = ros::NodeHandle();

  sensor_msgs::PointCloud2 keypoints;
  sensor_msgs::PointCloud2Modifier modifier(keypoints);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  auto height        = 10;
  auto width         = 10;
  auto size_of_cloud = height * width;
  modifier.resize(size_of_cloud);
  getRandomPointCloud(keypoints, 0.5, 0.5, size_of_cloud);

  keypoints.header.frame_id = "base_link";
  keypoints.header.stamp    = ros::Time::now();

  auto keypoints_publisher = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
  ros::Rate rate(30);
  while (ros::ok()) {
    keypoints.header.stamp = ros::Time::now();
    keypoints_publisher.publish(keypoints);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
