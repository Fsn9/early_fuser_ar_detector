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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> SyncPolicy;

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
const bool rgb_to_gray = true;

struct PclMsg
{
	std_msgs::Header header;
	uint32_t height;
	uint32_t width;
	std::vector<sensor_msgs::PointField> fields;
	bool is_bigendian;
	uint32_t point_step;
	uint32_t  row_step;
	std::vector<uint8_t> data;
	bool is_dense;
};

struct CompressedImageMsg
{
	std_msgs::Header header;
	std::string format;
	std::vector<uint8_t> data;
};	

struct CameraInfoMsg
{
	std_msgs::Header header;
	uint32_t height;
	uint32_t width;
	std::string distortion_model;
	std::vector<double> D;
	boost::array<double, 9> K;
	boost::array<double, 9> R;
	boost::array<double, 12> P;
	uint32_t binning_x;
	uint32_t binning_y;
	sensor_msgs::RegionOfInterest roi;
};

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
				sensor_msgs::PointCloud2ConstPtr pcl_first_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_data_topic_);

				thermal_sub_ = nh->subscribe(thermal_data_topic_, 1000, &Fuser::cb_thermal, this);
				sensor_msgs::CompressedImage thermal_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>(thermal_data_topic_));

				rgb_sub_ = nh->subscribe(rgb_data_topic_, 1000, &Fuser::cb_rgb, this);
				sensor_msgs::CompressedImage rgb_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>(rgb_data_topic_));
			}
		}

		void cb_pcl(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg)
		{
			ROS_INFO("Pcl arrived");
			last_pcl_.header = pcl_msg->header;
			last_pcl_.height = pcl_msg->height;
			last_pcl_.width = pcl_msg->width;
			last_pcl_.fields = pcl_msg->fields;
			last_pcl_.is_bigendian = pcl_msg->is_bigendian;
			last_pcl_.point_step = pcl_msg->point_step;
			last_pcl_.row_step = pcl_msg->row_step;
			last_pcl_.data = pcl_msg->data;
			last_pcl_.is_dense = pcl_msg->is_dense;
		}
		
		void cb_thermal(const sensor_msgs::CompressedImage::ConstPtr &thermal_msg)
		{
			ROS_INFO("Thermal arrived");
			last_thermal_.header = thermal_msg->header;
			last_thermal_.format = thermal_msg->format;
			last_thermal_.data = thermal_msg->data;
		}
		
		void cb_rgb(const sensor_msgs::CompressedImage::ConstPtr &rgb_msg)
		{
			ROS_INFO("Rgb arrived");
			last_rgb_.header = rgb_msg->header;
			last_rgb_.format = rgb_msg->format;
			last_rgb_.data = rgb_msg->data;
		}

		void cb_sensors(const sensor_msgs::CompressedImage::ConstPtr& rgb_msg, const sensor_msgs::CompressedImage::ConstPtr& thermal_msg, const sensor_msgs::PointCloud2::ConstPtr& pcl_msg)
		{
			ROS_INFO("Synchronized sensor data arrived");
			last_rgb_.header = rgb_msg->header;
			last_rgb_.format = rgb_msg->format;
			last_rgb_.data = rgb_msg->data;

			last_thermal_.header = thermal_msg->header;
			last_thermal_.format = thermal_msg->format;
			last_thermal_.data = thermal_msg->data;

			last_pcl_.header = pcl_msg->header;
			last_pcl_.height = pcl_msg->height;
			last_pcl_.width = pcl_msg->width;
			last_pcl_.fields = pcl_msg->fields;
			last_pcl_.is_bigendian = pcl_msg->is_bigendian;
			last_pcl_.point_step = pcl_msg->point_step;
			last_pcl_.row_step = pcl_msg->row_step;
			last_pcl_.data = pcl_msg->data;
			last_pcl_.is_dense = pcl_msg->is_dense;
		}
		
		void fuse()
		{
			ROS_INFO(">> Fusing");

			// Get data from last iteration
			/// @brief header
			std_msgs::Header msg_header = last_rgb_.header;
			/// @brief frame id
			std::string frame_id = msg_header.frame_id.c_str();
			/// @brief image data
			sensor_msgs::CompressedImage rgb_msg, thermal_msg;
			rgb_msg.header = last_rgb_.header;
			rgb_msg.format = last_rgb_.format;
			rgb_msg.data = last_rgb_.data;
			thermal_msg.header = last_thermal_.header;
			thermal_msg.format = last_thermal_.format;
			thermal_msg.data = last_thermal_.data;
			cv_bridge::CvImagePtr rgb_image_ptr;
			cv_bridge::CvImagePtr thermal_image_ptr;

			// 1. Save image rgb message
			try
			{
				if (rgb_to_gray) rgb_image_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::MONO8);
				else rgb_image_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8);
			}
			catch (cv::Exception &e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			// 2. Save image thermal message
			try
			{
				thermal_image_ptr = cv_bridge::toCvCopy(thermal_msg, sensor_msgs::image_encodings::MONO8);
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
			cv::initUndistortRectifyMap(K_rgb_, D_rgb_, R_rgb_, K_rgb_, cv::Size(cam_info_rgb_.width, cam_info_rgb_.height), CV_32FC1, rm1, rm2);
			cv::remap(rgb_image_ptr->image, rgb_image_ptr->image, rm1, rm2, cv::INTER_LINEAR);

			/* THERMAL */
			// UNDISTORT + RECTIFY
			rm1.release();
			rm2.release();
			cv::initUndistortRectifyMap(K_rgb_, D_rgb_, R_rgb_, K_rgb_, cv::Size(cam_info_thermal_.width, cam_info_thermal_.height), CV_32FC1, rm1, rm2);
			cv::remap(thermal_image_ptr->image, thermal_image_ptr->image, rm1, rm2, cv::INTER_LINEAR);
			
			// PAINT (flir water mark)
			cv::rectangle(thermal_image_ptr->image, cv::Point(460,10), cv::Point(560,70), cv::Scalar(0,0,0), cv::FILLED);

			// RESIZE (thermal) to rgb size
			cv::resize(thermal_image_ptr->image, thermal_image_ptr->image, rgb_image_ptr->image.size());

			// cv::namedWindow("visual", cv::WINDOW_NORMAL);
			// cv::resizeWindow("visual", 640, 480);
			// cv::imshow("visual", rgb_image_ptr->image);
			// cv::waitKey(3);

			// AFFINE (align thermal with rgb)
			//cv::warpAffine(thermal_image_ptr->image, thermal_image_ptr->image, translation_thermal_, thermal_image_ptr->image.size());
			//cv::namedWindow("thermal_affine", cv::WINDOW_NORMAL);
			//cv::resizeWindow("thermal_affine", 640, 480);
			//cv::imshow("thermal_affine", thermal_image_ptr->image);

			/* Point Cloud */
			// Container for original & filtered data
			pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
			pcl::PCLPointCloud2ConstPtr cloud_ptr(cloud);
			pcl::PCLPointCloud2 cloud_filtered;

			// Convert to PCL data type
			sensor_msgs::PointCloud2 pcl_msg;
			pcl_msg.header = last_pcl_.header;
			pcl_msg.height = last_pcl_.height;
			pcl_msg.width = last_pcl_.width;
			pcl_msg.fields = last_pcl_.fields;
			pcl_msg.is_bigendian = last_pcl_.is_bigendian;
			pcl_msg.point_step = last_pcl_.point_step;
			pcl_msg.row_step = last_pcl_.row_step;
			pcl_msg.data = last_pcl_.data;
			pcl_msg.is_dense = last_pcl_.is_dense;
			pcl_conversions::toPCL(pcl_msg, *cloud);
			
			// Downsample the point cloud
			pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
			sor.setInputCloud(cloud_ptr);
			sor.setLeafSize(0.05, 0.05, 0.05);
			sor.filter(cloud_filtered);	

			// Convert to ROS data type
			pcl::fromPCLPointCloud2(cloud_filtered, *pcl_output);
			
			// Perform cropping
			pcl::CropBox<pcl::PointXYZ> crop;
			crop.setInputCloud(pcl_output);
			crop.setMin(Eigen::Vector4f(-2.0, 0, -2.0, 1.0));
			crop.setMax(Eigen::Vector4f(2.0, max_range_, 2.0, 1.0));
			//crop.setMin(Eigen::Vector4f(-1.5, 0, -1.5, 1.0));
			//crop.setMax(Eigen::Vector4f(1.5, max_range_, 1.5, 1.0));
			crop.filter(*pcl_output);

			// Publish resulting point cloud
			// Apply TF
			try
			{
				tf_listener_.lookupTransform("/os_sensor", "/camera_frame", ros::Time(0), lidar2cam_tf_);
    		}
			catch (tf::TransformException &ex) 
			{
		    	ROS_WARN("%s",ex.what());
    		}
			pcl_ros::transformPointCloud(*pcl_output, *pcl_output, lidar2cam_tf_);

			// Publish final point cloud for visualization
			sensor_msgs::PointCloud2 final_pcl_msg;
			pcl::toROSMsg(*pcl_output.get(), final_pcl_msg);
			final_pcl_pub_.publish(final_pcl_msg);

			// Points
			cv::Point3d points3D;
			cv::Point2d points2D;
			cv::Point pixels;
			
			// Create camera info message
			sensor_msgs::CameraInfo cam_info_msg;
			image_geometry::PinholeCameraModel camera_geometry;
			cam_info_msg.header = cam_info_rgb_.header;
			cam_info_msg.width = cam_info_rgb_.width;
			cam_info_msg.height = cam_info_rgb_.height;
			cam_info_msg.distortion_model = cam_info_rgb_.distortion_model;
			cam_info_msg.D = cam_info_rgb_.D;
			cam_info_msg.K = cam_info_rgb_.K;
			cam_info_msg.R = cam_info_rgb_.R;
			cam_info_msg.P = cam_info_rgb_.P;
			cam_info_msg.binning_x = cam_info_rgb_.binning_x;
			cam_info_msg.binning_y = cam_info_rgb_.binning_y;
			cam_info_msg.roi = cam_info_rgb_.roi;
			camera_geometry.fromCameraInfo(cam_info_msg);

			// Auxiliar variables
			int pointscount = 0;
			int j = 0;
			float depth;
			float u, v;
			int color;
			// Image of depths
			cv::Mat lidar_image = cv::Mat::zeros(cv::Size(cam_info_rgb_.width, cam_info_rgb_.height), CV_8UC1);
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

				if (points2D.x > 0 && points2D.x < cam_info_rgb_.width && points2D.y > 0 && points2D.y < cam_info_rgb_.height)
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
			sensor_msgs::CameraInfoConstPtr info_rgb = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rgb_info_topic_, ros::Duration(2));
			sensor_msgs::CameraInfoConstPtr info_thermal = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(thermal_info_topic_, ros::Duration(2));

			if (info_rgb == nullptr) 
			{
				ROS_ERROR("No RGB camera info. Quitting...\n");
				ros::shutdown();
			}
			if (info_thermal == nullptr) 
			{
				ROS_ERROR("No thermal camera info. Quitting...\n");
				ros::shutdown();
			}

			// RGB
			cam_info_rgb_.header = info_rgb->header;
			cam_info_rgb_.height = info_rgb->height;
			cam_info_rgb_.width = info_rgb->width;
			cam_info_rgb_.distortion_model = info_rgb->distortion_model;
			cam_info_rgb_.D = {-0.5138254596294459, 0.44290503681520377, 0.0020747506912668404, 0.0011692118540784398, -0.3681143872182688};
			cam_info_rgb_.K = {1743.4035352713363, 0.0, 760.3723854064434, 0.0, 1739.4423246973906, 595.5405415362117, 0.0, 0.0, 1.0};	
			cam_info_rgb_.R = {1,0,0,0,1,0,0,0,1};
			cam_info_rgb_.P = {1743.4035352713363, 0.0, 760.3723854064434, 0.0,
				0.0, 1739.4423246973906, 595.5405415362117, 0.0,
				0.0, 0.0, 1.0, 0.0};
			cam_info_rgb_.binning_x = info_rgb->binning_x;
			cam_info_rgb_.binning_y = info_rgb->binning_y;
			cam_info_rgb_.roi = info_rgb->roi;
			K_rgb_ = cv::Mat(3, 3, CV_64F, &cam_info_rgb_.K[0]);
			D_rgb_ = cv::Mat(5, 1, CV_64F, &cam_info_rgb_.D[0]);
			R_rgb_ = cv::Mat(3, 3, CV_64F, &cam_info_rgb_.R[0]);

			// Thermal
			cam_info_thermal_.header = info_thermal->header;
			cam_info_thermal_.height = info_thermal->height;
			cam_info_thermal_.width = info_thermal->width;
			cam_info_thermal_.distortion_model = info_thermal->distortion_model;
			cam_info_thermal_.D = {-8.7955719162052803e-03, 2.7957338512854757e-01, 2.9514273519729906e-03, -7.8091815268012512e-03, -1.0969845111284882e+00};
			cam_info_thermal_.K = {5.8651197564377128e+02, 0., 3.0317247522782532e+02, 0.,7.3675903031957341e+02,2.5406537636242152e+02,0.,0.,1.};	
			cam_info_thermal_.R = {1,0,0,0,1,0,0,0,1};
			cam_info_thermal_.P = {5.8651197564377128e+02, 0., 3.0317247522782532e+02, 0.0, 
				0.,7.3675903031957341e+02,2.5406537636242152e+02, 0.0,
				0.,0.,1.,0.};
			cam_info_thermal_.binning_x = info_thermal->binning_x;
			cam_info_thermal_.binning_y = info_thermal->binning_y;
			cam_info_thermal_.roi = info_thermal->roi;
			K_thermal_ = cv::Mat(3, 3, CV_64F, &cam_info_thermal_.K[0]);
			D_thermal_ = cv::Mat(5, 1, CV_64F, &cam_info_thermal_.D[0]);
			R_thermal_ = cv::Mat(3, 3, CV_64F, &cam_info_thermal_.R[0]);
		}
	private:
		PclMsg last_pcl_;
		CompressedImageMsg last_thermal_;
		CompressedImageMsg last_rgb_;
		CameraInfoMsg cam_info_rgb_;
		CameraInfoMsg cam_info_thermal_;
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
		std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pcl_sub_sync_;
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
