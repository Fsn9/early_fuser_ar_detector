#include "time_sync.h"

void processTimeStamp (ros::Time &stamp)
{
  if(first)
  {
    last_time_header = stamp;
    prev_time = stamp = ros::Time::now();
    first = false;
  }
  else
  {
    ros::Duration dt = stamp - last_time_header;
    last_time_header = stamp;

    prev_time += dt;
    stamp = prev_time;
  }
}

template<typename MsgPtr>
void headerCb (MsgPtr m)
{
  processTimeStamp(m.header.stamp);
  publishData<MsgPtr>(pub,m);
}

void laserCb (sensor_msgs::LaserScan m)
{
  processTimeStamp(m.header.stamp);
  m.time_increment = 0;
  m.scan_time = 0;

  publishData<sensor_msgs::LaserScan>(pub,m);
}

/** @function main */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "time_sync");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  int data_type = -1;
  n.param<int>("data_type", data_type, -1);

  std::string topic_sub = "";
  n.param<std::string>("topic_subscriber", topic_sub, "");

  std::string topic_pub = "";
  n.param<std::string>("topic_publisher", topic_pub, "");

  ros::Subscriber sub;

  /// @brief If namespace needed in topic name
  std::string ns = "";
  nh.param<std::string>("namespace", ns, "");
  if (!ns.empty())
  {
    topic_sub = "/" + ns + topic_sub;
    topic_pub = "/" + ns + topic_pub;
  }

  switch (data_type)
  {
  case GPS:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<sensor_msgs::NavSatFix>);
    pub = nh.advertise<sensor_msgs::NavSatFix>( topic_pub, 0 );
    break;
  }
  case ODOM:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<nav_msgs::Odometry>);
    pub = nh.advertise<nav_msgs::Odometry>(topic_pub, 0);
    break;
  }
  case IMG:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<sensor_msgs::Image>);
    pub = nh.advertise<sensor_msgs::Image>(topic_pub, 0);
    break;
  }
  case IMGCOMP:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<sensor_msgs::CompressedImage>);
    pub = nh.advertise<sensor_msgs::CompressedImage>(topic_pub, 0);
    break;
  }
  case PCL:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<sensor_msgs::PointCloud2>);
    pub = nh.advertise<sensor_msgs::PointCloud2>(topic_pub, 0);
    break;
  }
  case TIME:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<sensor_msgs::TimeReference>);
    pub = nh.advertise<sensor_msgs::TimeReference>(topic_pub, 0);
    break;
  }
  case IMU:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<sensor_msgs::Imu>);
    pub = nh.advertise<sensor_msgs::Imu>(topic_pub, 0);
    break;
  }
  case SCAN:
  {
    sub = nh.subscribe(topic_sub, 1, laserCb);
    pub = nh.advertise<sensor_msgs::LaserScan>(topic_pub, 0);
    break;
  }
  case IMGINFO:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<sensor_msgs::CameraInfo>);
    pub = nh.advertise<sensor_msgs::CameraInfo>(topic_pub, 0);
    break;
  }
  case STATUS:
  {
    /*
    sub = nh.subscribe(topic_sub, 1, headerCb<roboteq_msgs::Status>);
    pub = nh.advertise<roboteq_msgs::Status>(topic_pub, 0);
    */
    break;
  }
  case TWISTCOV:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<geometry_msgs::TwistWithCovarianceStamped>);
    pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(topic_pub, 0);
    break;
  }
  case TWISTSTAMP:
  {
    sub = nh.subscribe(topic_sub, 1, headerCb<geometry_msgs::TwistStamped>);
    pub = nh.advertise<geometry_msgs::TwistStamped>(topic_pub, 0);
    break;
  }
  default:
    return 0;
  }

  ros::spin();
  return 0;
}
