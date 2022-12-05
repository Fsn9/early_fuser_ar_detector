#ifndef TIMESYNC_H
#define TIMESYNC_H


#include <string>
#include <stdio.h>
#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
//#include <roboteq_msgs/Status.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>

enum topicType {GPS=0, ODOM, IMG, IMGCOMP, PCL, TIME, IMU, SCAN, IMGINFO, STATUS, TWISTCOV, TWISTSTAMP};
ros::Publisher pub;
ros::Time last_time_header, prev_time;
bool first = true;

/**
 * @brief publishData - data publisher template for any type of msg
 * @param pub_
 * @param data_
 */
template <class DataType>
void publishData(ros::Publisher& pub_, const DataType& data_)
{
  if(pub_.getNumSubscribers()>0)
    pub_.publish(data_);

  return;
}

#endif
