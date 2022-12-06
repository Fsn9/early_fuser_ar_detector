#include <ros/ros.h>
#include <tf/transform_listener.h>

// Time
#include <ctime>

// Messages
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/CameraInfo.h>

// OpenCV2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/calib3d.hpp>

// ROS Synchronize
#include <memory>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Include CvBridge, Image Transport, Image msg
//#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PCL;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> SyncPolicy;

PCL::Ptr pcl_output(new PCL);
const bool rgb_to_gray = true;
const float leaf_size = 0.05;

class Fuser
{
	public:
		Fuser(std::shared_ptr<ros::NodeHandle> nh)
		{
			nh_ = nh;
			// Set parameters
			dataset_path_ = "/home/fsn9/Desktop/presentation/";
			im_format_dataset_ = ".bmp";
			num_frames_ = 0;
			max_range_ = 8.0;
			tx_thermal_ = 50;
			ty_thermal_ = 120;
			float warp_values[] = {1.0, 0.0, tx_thermal_, 0.0, 1.0, ty_thermal_};
			cv::Mat translation_thermal_(2, 3, CV_32F, warp_values);
			fused_image_pub_ = nh->advertise<sensor_msgs::Image>("early_fused_input", 1);
			final_pcl_pub_ = nh->advertise<sensor_msgs::PointCloud2>("final_pcl", 1);
			cv::Mat black_image_(cv::Size(1440,1080), CV_8UC1); // @TODO: solve hardcoded width and height

			// Initialize empty pointcloud
			last_filtered_pcl_ = boost::make_shared<PCL>();

			// Load ros params
			nh->getParam("pcl_data_topic", pcl_data_topic_);
			nh->getParam("thermal_data_topic", thermal_data_topic_);
			nh->getParam("thermal_info_topic", thermal_info_topic_);
			nh->getParam("rgb_data_topic", rgb_data_topic_);
			nh->getParam("rgb_info_topic", rgb_info_topic_);
			nh->getParam("sync_sensors", sync_sensors_);
			
			// Load camera infos
			load_cameras_info();

			/* Subscribers */
			// Synchronization
			if(sync_sensors_)
			{
				// Create synched subscriber objects
				pcl_sub_sync_ = std::make_unique<message_filters::Subscriber<sensor_msgs::PointCloud2>>(*nh, pcl_data_topic_, 1);
				sensor_msgs::PointCloud2 pcl_first_msg = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_data_topic_));
				
				thermal_sub_sync_ = std::make_unique<message_filters::Subscriber<sensor_msgs::CompressedImage>>(*nh, thermal_data_topic_, 1);
				sensor_msgs::CompressedImage thermal_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>(thermal_data_topic_));

				rgb_sub_sync_ = std::make_unique<message_filters::Subscriber<sensor_msgs::CompressedImage>>(*nh, rgb_data_topic_, 1);
				sensor_msgs::CompressedImage rgb_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>(rgb_data_topic_));

				synchronizer_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100), *rgb_sub_sync_, *thermal_sub_sync_, *pcl_sub_sync_);				
				synchronizer_->registerCallback(boost::bind(&Fuser::cb_sensors, this, _1, _2, _3));
			}
			// Async
			else
			{
				pcl_sub_ = nh->subscribe(pcl_data_topic_, 1000, &Fuser::cb_pcl, this);
				sensor_msgs::PointCloud2 pcl_first_msg = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_data_topic_));

				thermal_sub_ = nh->subscribe(thermal_data_topic_, 1000, &Fuser::cb_thermal, this);
				sensor_msgs::CompressedImage thermal_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>(thermal_data_topic_));

				rgb_sub_ = nh->subscribe(rgb_data_topic_, 1000, &Fuser::cb_rgb, this);
				sensor_msgs::CompressedImage rgb_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>(rgb_data_topic_));
			}
		}

		void cb_pcl(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg)
		{
			ROS_INFO("Pcl arrived");
			last_pcl_ = *pcl_msg;
		}
		
		void cb_thermal(const sensor_msgs::CompressedImage::ConstPtr &thermal_msg)
		{
			ROS_INFO("Thermal arrived");
			last_thermal_ = *thermal_msg;
		}
		
		void cb_rgb(const sensor_msgs::CompressedImage::ConstPtr &rgb_msg)
		{
			ROS_INFO("Rgb arrived");
			last_rgb_ = *rgb_msg;
		}

		void cb_sensors(const sensor_msgs::CompressedImage::ConstPtr& rgb_msg, const sensor_msgs::CompressedImage::ConstPtr& thermal_msg, const sensor_msgs::PointCloud2::ConstPtr& pcl_msg)
		{
			ROS_INFO("Synchronized sensor data arrived");
			last_thermal_ = *thermal_msg;
			last_rgb_ = *rgb_msg;
			last_pcl_ = *pcl_msg;
		}
		
		void fuse()
		{
			ROS_INFO(">> Fusing");
			/// @brief header
			std_msgs::Header msg_header = last_rgb_.header;
			/// @brief frame id
			std::string frame_id = msg_header.frame_id.c_str();
			/// @brief image data
			cv_bridge::CvImagePtr rgb_image_ptr;
			cv_bridge::CvImagePtr thermal_image_ptr;

			// 1. Save image rgb message
			try
			{
				if (rgb_to_gray) rgb_image_ptr = cv_bridge::toCvCopy(last_rgb_, sensor_msgs::image_encodings::MONO8);
				else rgb_image_ptr = cv_bridge::toCvCopy(last_rgb_, sensor_msgs::image_encodings::RGB8);
			}
			catch (cv::Exception &e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			// 2. Save image thermal message
			try
			{
				thermal_image_ptr = cv_bridge::toCvCopy(last_thermal_, sensor_msgs::image_encodings::MONO8);
			}
			catch (cv::Exception &e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}

			/* VISUAL */
			// Desired final size of fused input
			cv::Size desired_size(1440, 1080);

			// UNDISTORT + RECTIFY (visual)

			cv::Mat rm1, rm2;
			cv::initUndistortRectifyMap(K_rgb_, D_rgb_, R_rgb_, K_rgb_, cv::Size(info_rgb_.width, info_rgb_.height), CV_32FC1, rm1, rm2);
			cv::remap(rgb_image_ptr->image, rgb_image_ptr->image, rm1, rm2, cv::INTER_LINEAR);

			/* THERMAL */
			// UNDISTORT + RECTIFY
			rm1.release();
			rm2.release();
			cv::initUndistortRectifyMap(K_rgb_, D_rgb_, R_rgb_, K_rgb_, cv::Size(info_thermal_.width, info_thermal_.height), CV_32FC1, rm1, rm2);
			cv::remap(thermal_image_ptr->image, thermal_image_ptr->image, rm1, rm2, cv::INTER_LINEAR);
			
			// PAINT (flir water mark)
			cv::rectangle(thermal_image_ptr->image, cv::Point(460,10), cv::Point(560,70), cv::Scalar(0,0,0), cv::FILLED);

			// RESIZE (thermal) to rgb size
			cv::resize(thermal_image_ptr->image, thermal_image_ptr->image, rgb_image_ptr->image.size());

			// AFFINE (align thermal with rgb)
			//cv::warpAffine(thermal_image_ptr->image, thermal_image_ptr->image, translation_thermal_, thermal_image_ptr->image.size());
			//cv::namedWindow("thermal_affine", cv::WINDOW_NORMAL);
			//cv::resizeWindow("thermal_affine", 640, 480);
			//cv::imshow("thermal_affine", thermal_image_ptr->image);

			/* Point Cloud */
			/// @brief Containers for original & filtered point cloud data
			pcl::PCLPointCloud2* original_pcl = new pcl::PCLPointCloud2;
			pcl::PCLPointCloud2ConstPtr original_pcl_const_ptr(original_pcl);
			pcl::PCLPointCloud2 filtered_pcl;

			/// @brief Get last pcl message data and convert it to pcl native
			pcl_conversions::toPCL(last_pcl_, *original_pcl);
			
			/// @brief Downsample the point cloud
			pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
			grid.setInputCloud(original_pcl_const_ptr);
			grid.setLeafSize(leaf_size, leaf_size, leaf_size);
			grid.filter(filtered_pcl);

			/// @brief convert PCLPointCloud2 to pcl::PCLPointCloud<PointXYZ>
			pcl::fromPCLPointCloud2(filtered_pcl, *last_filtered_pcl_);
			
			/// @brief Crop pointcloud
			pcl::CropBox<pcl::PointXYZ> crop;
			crop.setInputCloud(last_filtered_pcl_);
			crop.setMin(Eigen::Vector4f(-2.0, 0, -2.0, 1.0));
			crop.setMax(Eigen::Vector4f(2.0, max_range_, 2.0, 1.0));
			crop.filter(*last_filtered_pcl_);

			// Get transform from lidar to the camera frame
			try
			{
				tf_listener_.lookupTransform("/os_sensor", "/camera_frame", ros::Time(0), lidar2cam_tf_);
    		}
			catch (tf::TransformException &ex) 
			{
		    	ROS_WARN("%s",ex.what());
    		}
			/// @brief Transform point cloud frame to camera frame
			pcl_ros::transformPointCloud(*last_filtered_pcl_, *last_filtered_pcl_, lidar2cam_tf_);

			/// @brief Publish final point cloud for visualization
			sensor_msgs::PointCloud2 filtered_pcl_msg;
			pcl::toROSMsg(*last_filtered_pcl_.get(), filtered_pcl_msg);
			final_pcl_pub_.publish(filtered_pcl_msg);

			/* Transform pointcloud to 2D grayscale image */
			cv::Point3d points3D;
			cv::Point2d points2D;
			cv::Point pixels;
			image_geometry::PinholeCameraModel camera_geometry;
			camera_geometry.fromCameraInfo(info_rgb_);

			// Auxiliar variables
			int pointscount = 0;
			int j = 0;
			float depth;
			float u, v;
			int color;
			// Image of depths
			cv::Mat lidar_image = cv::Mat::zeros(cv::Size(info_rgb_.width, info_rgb_.height), CV_8UC1);
			for (pcl::PointXYZ point : pcl_output->points)
			{
				points3D.x = point.y; // xcam = zlaser
				points3D.y = point.z; // ycam = xlaser
				points3D.z = point.x; // zcam = ylaser
				depth = points3D.z; 
				
				// Project
				//points2D = camera_geometry.project3dToPixel(points3D);

				u = (camera_geometry.fx() * points3D.x / depth) + camera_geometry.cx();
				v = (camera_geometry.fy() * points3D.y / depth) + camera_geometry.cy();
				points2D.x = (int)u;
				points2D.y = (int)v;

				if (points2D.x > 0 && points2D.x < info_rgb_.width && points2D.y > 0 && points2D.y < info_rgb_.height)
				{
					// Saturate depth
					if(depth >= max_range_) depth = max_range_;

					// Define colour between 0 and 255
					color = 255 * (1.0 - depth / max_range_);

					pointscount++;
					//cv::circle(rgb_image_ptr->image, points2D, 0, cv::Scalar(color), 5);

					// Draw pointcloud 2D image
					lidar_image.at<uchar>((int)points2D.y, (int)points2D.x) = color;
				}
				j++;
			}

			//pcl::PointCloud::Ptr received_cloud_ptr;
			//received_cloud_ptr.reset(new pcl::PointCloud);
			//sensor_msgs::PointCloud2ConstPtr pointcloud_msg;
			//pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());

			// Make depthmap message
			std_msgs::Header fused_image_header;
			fused_image_header.seq = num_frames_;
			fused_image_header.stamp = msg_header.stamp;

			// Make final image message
			sensor_msgs::Image image_msg;
			// o 32FC3 nao vai de 0 a 255. sao floats tipo pontos xyz
			// mudar para bgr8 ou rgb8
			//fused_image_bridge = cv_bridge::CvImage(fused_image_header, sensor_msgs::image_encodings::TYPE_32FC3, lidar_image);
			//fused_image_bridge = cv_bridge::CvImage(fused_image_header, sensor_msgs::image_encodings::BGR8, lidar_image);
			//fused_image_bridge.toImageMsg(image_msg);
			//fused_image_pub.publish(image_msg);

			//std::cout << "lidarpoints: " << pointscount << std::endl;
			num_frames_++;

			//cv::namedWindow("rgb_fused", cv::WINDOW_NORMAL);
			//cv::resizeWindow("rgb_fused", 640, 480);
			//cv::imshow("rgb_fused", rgb_image_ptr->image);

			//rgb_image_ptr->image.convertTo(rgb_image_ptr->image, CV_32FC1, 1.0 / 255.0);
			//lidar_image.convertTo(lidar_image, CV_32FC1, 1.0 / 255.0);

			//cv::namedWindow("depth_map", cv::WINDOW_NORMAL);
			//cv::resizeWindow("depth_map", 640, 480);
			//cv::imshow("depth_map", lidar_image);
			//cv::waitKey(3);

			// Dilate depth map
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11,11));
			cv::dilate(lidar_image, lidar_image, kernel);

			// Concatenate images
			cv::Mat input;
			std::vector<cv::Mat> channels;
			cv::Mat black_image_(cv::Size(1440,1080), CV_8UC1);
			black_image_ = 0; 
			//channels.push_back(black_image_);
			channels.push_back(rgb_image_ptr->image);
			//channels.push_back(black_image_);
			//channels.push_back(black_image_);
			channels.push_back(thermal_image_ptr->image);
			//channels.push_back(black_image_);
			//channels.push_back(lidar_image);
			channels.push_back(black_image_);
			//channels.push_back(black_image_);

			try{
				cv::merge(channels, input);
			}
			catch (cv::Exception &e)
			{
				ROS_ERROR("cv exception: %s", e.what());
				return;
			}

			// Publish fused inputs
			cv_bridge::CvImage fused_image_bridge = cv_bridge::CvImage(fused_image_header, sensor_msgs::image_encodings::BGR8, input);
			fused_image_bridge.toImageMsg(image_msg);
			fused_image_pub_.publish(image_msg);

			/*
			cv::namedWindow("input", cv::WINDOW_NORMAL);
			cv::resizeWindow("input", 640, 480);
			cv::imshow("input", input);
			cv::waitKey(3);
			*/
			// Save images
			/*
			cv::imwrite(dataset_path + "im_visual_" + std::to_string(num_frames_) + im_format_dataset_, true_rgb_image_ptr->image);
			cv::imwrite(dataset_path + "im_thermal_" + std::to_string(num_frames_) + im_format_dataset_, thermal_image_ptr->image);
			cv::imwrite(dataset_path + "im_lidar_" + std::to_string(num_frames_) + im_format_dataset_, lidar_image);
			cv::imwrite(dataset_path + "im_input_" + std::to_string(num_frames_) + im_format_dataset_, input);
			std::cout << "PATH: " << dataset_path + "im_" + std::to_string(num_frames_) + im_format_dataset_ << "\n";	
			*/
			// Data augment
		}
		
		void load_cameras_info()
		{
			ROS_INFO("Loading Camera info...");
			sensor_msgs::CameraInfoConstPtr info_rgb_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rgb_info_topic_, ros::Duration(20));
			sensor_msgs::CameraInfoConstPtr info_thermal_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(thermal_info_topic_, ros::Duration(20));
			if (info_rgb_ptr == nullptr) 
			{
				ROS_ERROR("Timeout. No RGB camera info. Quitting...\n");
				ros::shutdown();
			}
			if (info_thermal_ptr == nullptr) 
			{
				ROS_ERROR("Timeout. No thermal camera info. Quitting...\n");
				ros::shutdown();
			}
			info_rgb_ = *info_rgb_ptr;
			info_thermal_ = *info_thermal_ptr;

			// RGB
			info_rgb_.D = {-0.5138254596294459, 0.44290503681520377, 0.0020747506912668404, 0.0011692118540784398, -0.3681143872182688};
			info_rgb_.K = {1743.4035352713363, 0.0, 760.3723854064434, 0.0, 1739.4423246973906, 595.5405415362117, 0.0, 0.0, 1.0};	
			info_rgb_.R = {1,0,0,0,1,0,0,0,1};
			info_rgb_.P = {1743.4035352713363, 0.0, 760.3723854064434, 0.0,
				0.0, 1739.4423246973906, 595.5405415362117, 0.0,
				0.0, 0.0, 1.0, 0.0};
			K_rgb_ = cv::Mat(3, 3, CV_64F, &info_rgb_.K[0]);
			D_rgb_ = cv::Mat(5, 1, CV_64F, &info_rgb_.D[0]);
			R_rgb_ = cv::Mat(3, 3, CV_64F, &info_rgb_.R[0]);

			// Thermal
			info_thermal_.D = {-8.7955719162052803e-03, 2.7957338512854757e-01, 2.9514273519729906e-03, -7.8091815268012512e-03, -1.0969845111284882e+00};
			info_thermal_.K = {5.8651197564377128e+02, 0., 3.0317247522782532e+02, 0.,7.3675903031957341e+02,2.5406537636242152e+02,0.,0.,1.};	
			info_thermal_.R = {1,0,0,0,1,0,0,0,1};
			info_thermal_.P = {5.8651197564377128e+02, 0., 3.0317247522782532e+02, 0.0, 
				0.,7.3675903031957341e+02,2.5406537636242152e+02, 0.0,
				0.,0.,1.,0.};
			K_thermal_ = cv::Mat(3, 3, CV_64F, &info_thermal_.K[0]);
			D_thermal_ = cv::Mat(5, 1, CV_64F, &info_thermal_.D[0]);
			R_thermal_ = cv::Mat(3, 3, CV_64F, &info_thermal_.R[0]);
		}
	private:
		PCL::Ptr last_filtered_pcl_;
		sensor_msgs::PointCloud2 last_pcl_;
		sensor_msgs::CompressedImage last_thermal_;
		sensor_msgs::CompressedImage last_rgb_;
		sensor_msgs::CameraInfo info_thermal_;
		sensor_msgs::CameraInfo info_rgb_;
		unsigned int num_frames_;
		std::string im_format_dataset_;
		std::string dataset_path_;
		std::string pcl_data_topic_;
		std::string thermal_data_topic_;
		std::string thermal_info_topic_;
		std::string rgb_data_topic_;
		std::string rgb_info_topic_;
		bool sync_sensors_;
		float max_range_;
		float tx_thermal_, ty_thermal_; // Translation to align thermal with visual image
		cv::Mat K_thermal_, D_thermal_, R_thermal_;
		cv::Mat K_rgb_, D_rgb_, R_rgb_;
		cv::Mat translation_thermal_; // The translation matrix
		ros::Publisher fused_image_pub_;
		ros::Publisher final_pcl_pub_;
		std::shared_ptr<ros::NodeHandle> nh_;
		cv::Mat black_image_;
		ros::Subscriber pcl_sub_;
		ros::Subscriber thermal_sub_;
		ros::Subscriber rgb_sub_;
		std::unique_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>> rgb_sub_sync_;
		std::unique_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>> thermal_sub_sync_;
		std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pcl_sub_sync_;
		std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
		tf::TransformListener tf_listener_;
		tf::StampedTransform lidar2cam_tf_;
};

int main(int argc, char **argv)
{
	// Initialize node
	ros::init(argc, argv, "fuser");
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	
	// Initialize fuser
	Fuser fuser(nh);

	tf::TransformListener tf_listener;
	ros::Rate rate(6);
	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		fuser.fuse();
	}
	return 0;
}
