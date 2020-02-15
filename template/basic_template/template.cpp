#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::string point_cloud_name;
std::string point_cloud_frame;

ros::Subscriber cloud_sub;
ros::Publisher pub_cloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_output_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*input, *pcl_cloud_ptr);

  // code here

  sensor_msgs::PointCloud2::Ptr ros_output_cloud(new sensor_msgs::PointCloud2);
  ros_output_cloud->header.frame_id = point_cloud_frame;

  // this should be `input->header.stamp` if accurate timing is required
  ros_output_cloud->header.stamp = ros::Time::now();

  pcl::toROSMsg(*pcl_cloud_output_ptr, *ros_output_cloud);

  pub_cloud.publish(ros_output_cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "passthrough");
  ros::NodeHandle nh;

  YAML::Node conf = YAML::LoadFile(ros::package::getPath("pass_through_filter") + "/config/pass_through_filter.yaml");

  point_cloud_name = conf["point_cloud_name"].as<std::string>();
  point_cloud_frame = conf["point_cloud_frame"].as<std::string>();

  cloud_sub = nh.subscribe(point_cloud_name, 1, cloud_cb);

  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);

  ros::spin();

  return 0;
}