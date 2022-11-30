#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/**
 * @brief tf_broad - Broadcast the pose to the tf tree
 * @param broadcaster_
 * @param data - Pose stamped to broadcast. VERY IMPORTANT TO FILL CORRECTLY THE HEADER!
 * @param parent_frame - Top level frame
 * @param child_frame - Bottom leve frame
 */
void tf_broad(tf::TransformBroadcaster* broadcaster_, geometry_msgs::PoseStamped data, std::string parent_frame, std::string child_frame)
{
    geometry_msgs::TransformStamped broad_data;
    broad_data.header.stamp = data.header.stamp;
    broad_data.header.frame_id = parent_frame;
    broad_data.child_frame_id = child_frame;
    broad_data.transform.translation.x = data.pose.position.x;
    broad_data.transform.translation.y = data.pose.position.y;
    broad_data.transform.translation.z = data.pose.position.z;
    broad_data.transform.rotation =  data.pose.orientation;

    try{
        broadcaster_->sendTransform(broad_data);
    } catch (tf::TransformException& e) {
        ROS_INFO ("Not saving scan due to tf lookup exception: %s", e.what());
    }
}

// Downsample Point Cloud with Voxel Grid
void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    int voxel;
    ros::param::param<int>("~voxel_size", voxel, 0);

    if(voxel){
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(voxel, voxel, voxel);
        sor.filter(*cloud);
    }
}

// Align Point Cloud (ICP) and returns its Fitness
float alignPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr& src)
{
    // If one of the Point Clouds is empty, exit
    if(tgt->points.empty() || src->points.empty()) return 0.0;

    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);

    // Compute surface normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(100);
    // Target Normals
    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);
    // Source Normals
    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    // Align
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setTransformationEpsilon(1e-6);///
    // Set the maximum distance between two correspondences (tgt<->src)
    reg.setMaxCorrespondenceDistance(0.1);///
    reg.setEuclideanFitnessEpsilon(0.01);///
    reg.setMaximumIterations(100);
    reg.setInputSource(points_with_normals_tgt);
    reg.setInputTarget(points_with_normals_src);

    // Align Point Cloud
    reg.align(*points_with_normals_tgt);

    if(reg.hasConverged())
        // Transform target to source frame
        pcl::transformPointCloud(*tgt, *tgt, reg.getFinalTransformation());
    else
        // Discard Unaligned Point Cloud
        tgt->points.clear();

    return 1.0/reg.getFitnessScore();
}

/** */
// Gets Cloud Time Stamp
ros::Time cloudStamp(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){

    ros::Time stamp;
    pcl_conversions::fromPCL(cloud->header.stamp, stamp);
    return stamp;
}
/** */

// Callback for Point Cloud & Drone Local Pose
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // Point cloud read from callback
geometry_msgs::PoseStamped::Ptr pose (new geometry_msgs::PoseStamped);
void cb_getData(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg){

    *cloud = *cloud_msg;

    pose->pose = odom_msg->pose.pose;
    pose->header = odom_msg->header;

    // Corrects Position
    double tmp = pose->pose.position.x;
    pose->pose.position.x = pose->pose.position.y;
    pose->pose.position.y = -tmp;

    // Corrects Orientation
    tf::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose->pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw-M_PI/2.0);

    // Frame id
    pose->header.frame_id = "world";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "model_reconstruction");
    ros::NodeHandle nh;

    std::string lidar_topic, odom_topic;
    ros::param::param< std::string>("~lidar_topic", lidar_topic, "");
    ros::param::param< std::string>("~odom_topic", odom_topic, "");

    // Point Cloud and Pose Subscrivers
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ>> sub_pcl(nh, lidar_topic, 1); // Point cloud subscriver
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, odom_topic, 1); // Odometry subscriver
    typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZ>, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), sub_pcl, sub_odom);
    sync.registerCallback(boost::bind(&cb_getData, _1, _2));

    // Registed Point Cloud Publisher
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("reg_cloud", 1);

    // Tranforms
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    // Registred Point Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr reg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reg_cloud->header.frame_id = "world";

    // PCD file directory
    std::string pcd_dir;
    ros::param::param<std::string>("~pcd", pcd_dir, "model.pcd");

    ros::Rate rate(100);
    while(ros::ok()){

      ros::spinOnce();

      if(cloud->points.size()){

        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
        quaternionMsgToTF(pose->pose.orientation , q);
        transform.setRotation(q);
//        broadcaster.sendTransform(tf::StampedTransform(transform, cloudStamp(cloud), "world", "base_link"));
        tf_broad(&broadcaster, *pose, "world", "base_link");

        try{ // Transforms point cloud from "os_sensor" to "world" frame
            /** */if(listener.waitForTransform("os_sensor", "world", cloudStamp(cloud), rate.expectedCycleTime()))/** */
            if(pcl_ros::transformPointCloud("world", *cloud, *cloud, listener)){

                //pcl_ros::transformPointCloud(*cloud, *cloud, transform);

                transform.setOrigin(tf::Vector3(0, 0, 0));
                q = tf::createQuaternionFromRPY(M_PI, 0, 0);
                transform.setRotation(q);
                pcl_ros::transformPointCloud(*cloud, *cloud, transform);

                // Align Point Cloud (ICP) and get Fitness Score
                float fitness = alignPointCloud(cloud, reg_cloud);

                // Registred Cloud
                *reg_cloud += *cloud;

                // Downsample Point Cloud
                downsamplePointCloud(reg_cloud);
            }
        }catch(tf::TransformException ex){ROS_ERROR("%s",ex.what());}

        // Publish Registred Point Cloud
        pub.publish(reg_cloud);

        cloud->points.clear();
      }

      std::cout << "\r" << "Registred Point Cloud: " << reg_cloud->size() << " number of points." << std::flush;
      rate.sleep();
    }

    // Save Point Cloud to PCD file
    pcl::io::savePCDFileASCII(pcd_dir, *reg_cloud);
    std::cout << std::endl;
}
