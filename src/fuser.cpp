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

//std::string dataset_path = "/home/fsn9/catkin_ws/src/platform_detector/dataset_fused/";
//std::string dataset_path = "/home/fsn9/catkin_ws/src/platform_detector/dataset_rgb_equiv/";
std::string dataset_path = "/home/fsn9/Desktop/presentation/";
std::string im_format_dataset = ".bmp";
unsigned int im_num = 0;
time_t now = time(0);
tm *ltm = localtime(&now);
const float max_range = 8.0;
float tx_thermal = 50;
float ty_thermal = 120;
float warp_values[] = {1.0, 0.0, tx_thermal, 0.0, 1.0, ty_thermal};
cv::Mat translation_thermal(2,3, CV_32F, warp_values);
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> SyncPolicy_vt;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, pcl::PointCloud<pcl::PointXYZ>> SyncPolicy_vtl;
float max_est_depth = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
tf::StampedTransform lidar2cam_tf;
cv::Mat imagemap;
cv_bridge::CvImage img_bridge;
std_msgs::Header depthmat_header;
std_msgs::Header msg_header;
sensor_msgs::Image img_msg;
ros::Publisher pubmatrix;
ros::Publisher final_pcl_pub;
image_geometry::PinholeCameraModel camera_geometry;

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
		Fuser()
		{
			num_frames_ = 0;
		}

		void listen_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pcl_msg)
		{
			ROS_INFO("Pcl arrived");
			/*
			last_pcl_.header = pcl_msg->header;
			last_pcl_.height = pcl_msg->height;
			last_pcl_.width = pcl_msg->width;
			last_pcl_.fields = pcl_msg->fields;
			last_pcl_.is_bigendian = pcl_msg->is_bigendian;
			last_pcl_.point_step = pcl_msg->point_step;
			last_pcl_.row_step = pcl_msg->row_step;
			last_pcl_.data = pcl_msg->data;
			last_pcl_.is_dense = pcl_msg->is_dense;
			*/
		}

		void republish_pcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pcl_msg)
		{
			ROS_INFO("Pcl arrived");
			/*
			sensor_msgs::PointCloud2 new_pcl_msg = *pcl_msg;
			std::cout << "before timestamp" << new_pcl_msg.header.stamp << "\n";
			// https://answers.ros.org/question/172241/pcl-and-rostime/
			new_pcl_msg.header.stamp = ros::Time::now();
			//pcl_conversions::toPCL(ros::Time::now(), new_pcl_msg.header.stamp);
			std::cout << "after timestamp" << new_pcl_msg.header.stamp << "\n";

			last_pcl_.header = pcl_msg->header;
			last_pcl_.height = pcl_msg->height;
			last_pcl_.width = pcl_msg->width;
			last_pcl_.fields = pcl_msg->fields;
			last_pcl_.is_bigendian = pcl_msg->is_bigendian;
			last_pcl_.point_step = pcl_msg->point_step;
			last_pcl_.row_step = pcl_msg->row_step;
			last_pcl_.data = pcl_msg->data;
			last_pcl_.is_dense = pcl_msg->is_dense;
			*/
		}
		
		void listen_thermal(const sensor_msgs::CompressedImage::ConstPtr &thermal_msg)
		{
			ROS_INFO("Thermal arrived");
			last_thermal_.header = thermal_msg->header;
			last_thermal_.format = thermal_msg->format;
			last_thermal_.data = thermal_msg->data;
		}
		
		void listen_rgb(const sensor_msgs::CompressedImage::ConstPtr &rgb_msg)
		{
			ROS_INFO("Rgb arrived");
			last_rgb_.header = rgb_msg->header;
			last_rgb_.format = rgb_msg->format;
			last_rgb_.data = rgb_msg->data;
		}

		void listen_cameras(const sensor_msgs::CompressedImage::ConstPtr &rgb_msg, const sensor_msgs::CompressedImage::ConstPtr &thermal_msg)
		{
			ROS_INFO("Both cameras arrived");
			last_rgb_.header = rgb_msg->header;
			last_rgb_.format = rgb_msg->format;
			last_rgb_.data = rgb_msg->data;

			last_thermal_.header = thermal_msg->header;
			last_thermal_.format = thermal_msg->format;
			last_thermal_.data = thermal_msg->data;
		}

		void cb_sensors(const sensor_msgs::CompressedImage::ConstPtr& rgb_msg, const sensor_msgs::CompressedImage::ConstPtr& thermal_msg, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl_msg)
		{
			ROS_INFO("All sensors arrived");
			last_rgb_.header = rgb_msg->header;
			last_rgb_.format = rgb_msg->format;
			last_rgb_.data = rgb_msg->data;

			last_thermal_.header = thermal_msg->header;
			last_thermal_.format = thermal_msg->format;
			last_thermal_.data = thermal_msg->data;

			/*
			last_pcl_.header = pcl_msg->header;
			last_pcl_.height = pcl_msg->height;
			last_pcl_.width = pcl_msg->width;
			last_pcl_.fields = pcl_msg->fields;
			last_pcl_.is_bigendian = pcl_msg->is_bigendian;
			last_pcl_.point_step = pcl_msg->point_step;
			last_pcl_.row_step = pcl_msg->row_step;
			last_pcl_.data = pcl_msg->data;
			last_pcl_.is_dense = pcl_msg->is_dense;
			*/
		}
		
		void fuse()
		{
			ROS_INFO(">> Fusing");

			msg_header = last_rgb_.header;
			std::string frame_id = msg_header.frame_id.c_str();

			// Collect images from last saved messages
			sensor_msgs::CompressedImage rgb_msg, thermal_msg;
			rgb_msg.header = last_rgb_.header;
			rgb_msg.format = last_rgb_.format;
			rgb_msg.data = last_rgb_.data;
			thermal_msg.header = last_thermal_.header;
			thermal_msg.format = last_thermal_.format;
			thermal_msg.data = last_thermal_.data;
			cv_bridge::CvImagePtr visual_data_ptr;
			cv_bridge::CvImagePtr thermal_image_ptr;
			std::cout << "\n##W##\n";
			// Image message content to CV image
			try
			{
				std::cout << "\n##X##\n";
				if (rgb_to_gray) visual_data_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::MONO8);
				else visual_data_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8);
				std::cout << "\n##X2##\n";
			}
			catch (cv::Exception &e)
			{
				std::cout << "\n##X3##\n";
				ROS_ERROR("cv_bridge exception: %s", e.what());
				std::cout << "\n##X4##\n";
				return;
			}
			try
			{
				std::cout << "\n##Y##\n";
				thermal_image_ptr = cv_bridge::toCvCopy(thermal_msg, sensor_msgs::image_encodings::MONO8);
				std::cout << "\n##Y2##\n";
			}
			catch (cv::Exception &e)
			{
				std::cout << "\n##Y3##\n";
				ROS_ERROR("cv_bridge exception: %s", e.what());
				std::cout << "\n##Y4##\n";
			}
			std::cout << "\n##A##\n";

			/////////////////
			// Visual //////
			////////////////

			// Desired final size of fused input
			cv::Size desired_size(1440,1080);

			// UNDISTORT + RECTIFY (visual)
			cv::Mat rm1, rm2, K, D, R;
			K = cv::Mat(3, 3, CV_64F, &cam_info_rgb_.K[0]);
			D = cv::Mat(5, 1, CV_64F, &cam_info_rgb_.D[0]);
			R = cv::Mat(3, 3, CV_64F, &cam_info_rgb_.R[0]);
			cv::initUndistortRectifyMap(K, D, R, K, cv::Size(cam_info_rgb_.width, cam_info_rgb_.height), CV_32FC1, rm1, rm2);
			cv::remap(visual_data_ptr->image, visual_data_ptr->image, rm1, rm2, cv::INTER_LINEAR);

			// UNDISTORT + RECTIFY (visual2)
			/*
			rm1.release();
			rm2.release();
			cv::initUndistortRectifyMap(K, D, R, K, cv::Size(cam_info_rgb_.width, cam_info_rgb_.height), CV_32FC1, rm1, rm2);
			cv::remap(true_rgb_image_ptr->image, true_rgb_image_ptr->image, rm1, rm2, cv::INTER_LINEAR);
			*/
			// RESIZE (visual)
			// cv::resize(visual_data_ptr->image, visual_data_ptr->image, desired_size);
			std::cout << "\n##B##\n";
			/////////////////
			// Thermal ///// 
			////////////////			

			// UNDISTORT + RECTIFY (thermal)
			rm1.release();
			rm2.release();
			K = cv::Mat(3, 3, CV_64F, &cam_info_thermal_.K[0]);
			D = cv::Mat(5, 1, CV_64F, &cam_info_thermal_.D[0]);
			R = cv::Mat(3, 3, CV_64F, &cam_info_thermal_.R[0]);
			cv::initUndistortRectifyMap(K, D, R, K, cv::Size(cam_info_thermal_.width, cam_info_thermal_.height), CV_32FC1, rm1, rm2);
			cv::remap(thermal_image_ptr->image, thermal_image_ptr->image, rm1, rm2, cv::INTER_LINEAR);
			
			// PAINT (flir water mark)
			cv::rectangle(thermal_image_ptr->image, cv::Point(460,10), cv::Point(560,70), cv::Scalar(0,0,0), cv::FILLED);

			// RESIZE (thermal) to rgb size
			cv::resize(thermal_image_ptr->image, thermal_image_ptr->image, desired_size);

			/*
			cv::namedWindow("thermal", cv::WINDOW_NORMAL);
			cv::resizeWindow("thermal", 640, 480);
			cv::imshow("thermal", thermal_image_ptr->image);
			cv::namedWindow("rgb", cv::WINDOW_NORMAL);
			cv::resizeWindow("rgb", 640, 480);
			cv::imshow("rgb", true_rgb_image_ptr->image);
			cv::waitKey(3);
			*/
			// AFFINE (align thermal with rgb)
			cv::warpAffine(thermal_image_ptr->image, thermal_image_ptr->image, translation_thermal, desired_size);
			std::cout << "\n##C##\n";
			//cv::namedWindow("thermal_affine", cv::WINDOW_NORMAL);
			//cv::resizeWindow("thermal_affine", 640, 480);
			//cv::imshow("thermal_affine", thermal_image_ptr->image);

			/////////////////
			// Point Cloud // 
			////////////////

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
			crop.setMax(Eigen::Vector4f(2.0, max_range, 2.0, 1.0));
			//crop.setMin(Eigen::Vector4f(-1.5, 0, -1.5, 1.0));
			//crop.setMax(Eigen::Vector4f(1.5, max_range, 1.5, 1.0));
			crop.filter(*pcl_output);

			// Publish resulting point cloud
			// Apply TF
			pcl_ros::transformPointCloud(*pcl_output, *pcl_output, lidar2cam_tf);

			// Publish final point cloud for visualization
			sensor_msgs::PointCloud2 final_pcl_msg;
			pcl::toROSMsg(*pcl_output.get(), final_pcl_msg);
			final_pcl_pub.publish(final_pcl_msg);
			

			// Points
			cv::Point3d points3D;
			cv::Point2d points2D;
			cv::Point pixels;
			
			// Create camera info message
			sensor_msgs::CameraInfo cam_info_msg;
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
			imagemap = cv::Mat::zeros(cv::Size(cam_info_rgb_.width, cam_info_rgb_.height), CV_8UC1);
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
					if(depth >= max_range) depth = max_range;

					// Define colour between 0 and 255
					color = 255 * (1.0 - depth / max_range);

					pointscount++;
					//cv::circle(visual_data_ptr->image, points2D, 0, cv::Scalar(color), 5);

					// Draw pointcloud 2D image
					imagemap.at<uchar>((int)points2D.y, (int)points2D.x) = color;
				}
				j++;
			}

			//pcl::PointCloud::Ptr received_cloud_ptr;
			//received_cloud_ptr.reset(new pcl::PointCloud);
			//sensor_msgs::PointCloud2ConstPtr pointcloud_msg;
			//pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());

			// Make depthmap message
			depthmat_header.seq = num_frames_;
			depthmat_header.stamp = msg_header.stamp;
			// o 32FC3 nao vai de 0 a 255. sao floats tipo pontos xyz
			// mudar para bgr8 ou rgb8
			//img_bridge = cv_bridge::CvImage(depthmat_header, sensor_msgs::image_encodings::TYPE_32FC3, imagemap);
			//img_bridge = cv_bridge::CvImage(depthmat_header, sensor_msgs::image_encodings::BGR8, imagemap);
			//img_bridge.toImageMsg(img_msg);
			//pubmatrix.publish(img_msg);

			//std::cout << "lidarpoints: " << pointscount << std::endl;
			num_frames_++;

			//cv::namedWindow("rgb_fused", cv::WINDOW_NORMAL);
			//cv::resizeWindow("rgb_fused", 640, 480);
			//cv::imshow("rgb_fused", visual_data_ptr->image);

			//visual_data_ptr->image.convertTo(visual_data_ptr->image, CV_32FC1, 1.0 / 255.0);
			//imagemap.convertTo(imagemap, CV_32FC1, 1.0 / 255.0);

			//cv::namedWindow("depth_map", cv::WINDOW_NORMAL);
			//cv::resizeWindow("depth_map", 640, 480);
			//cv::imshow("depth_map", imagemap);
			//cv::waitKey(3);

			// Dilate depth map
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11,11));
			cv::dilate(imagemap, imagemap, kernel);

			// Concatenate images
			cv::Mat input;
			std::vector<cv::Mat> channels;
			cv::Mat empty(cv::Size(1440,1080), CV_8UC1);
			empty = 0; 
			//channels.push_back(empty);
			channels.push_back(visual_data_ptr->image);
			//channels.push_back(empty);
			//channels.push_back(empty);
			channels.push_back(thermal_image_ptr->image);
			//channels.push_back(empty);
			//channels.push_back(imagemap);
			channels.push_back(empty);
			//channels.push_back(empty);
			cv::merge(channels, input);

			// Publish fused inputs
			img_bridge = cv_bridge::CvImage(depthmat_header, sensor_msgs::image_encodings::BGR8, input);
			img_bridge.toImageMsg(img_msg);
			pubmatrix.publish(img_msg);

			/*
			cv::namedWindow("input", cv::WINDOW_NORMAL);
			cv::resizeWindow("input", 640, 480);
			cv::imshow("input", input);
			cv::waitKey(3);
			*/
			// Save images
			/*
			cv::imwrite(dataset_path + "im_visual_" + std::to_string(im_num) + im_format_dataset, true_rgb_image_ptr->image);
			cv::imwrite(dataset_path + "im_thermal_" + std::to_string(im_num) + im_format_dataset, thermal_image_ptr->image);
			cv::imwrite(dataset_path + "im_lidar_" + std::to_string(im_num) + im_format_dataset, imagemap);
			cv::imwrite(dataset_path + "im_input_" + std::to_string(im_num) + im_format_dataset, input);
			std::cout << "PATH: " << dataset_path + "im_" + std::to_string(im_num) + im_format_dataset << "\n";	
			*/
			// Data augment
			++im_num;
		}
		
		void set_camera_info(sensor_msgs::CameraInfo::ConstPtr &camera_info_msg)
		{
			ROS_INFO("Setting Camera info...");
			if(camera_info_msg->header.frame_id == "cam")
			{				
				cam_info_rgb_.header = camera_info_msg->header;
				cam_info_rgb_.height = camera_info_msg->height;
				cam_info_rgb_.width = camera_info_msg->width;
				cam_info_rgb_.distortion_model = camera_info_msg->distortion_model;
				cam_info_rgb_.D = {-0.5138254596294459, 0.44290503681520377, 0.0020747506912668404, 0.0011692118540784398, -0.3681143872182688};
				cam_info_rgb_.K = {1743.4035352713363, 0.0, 760.3723854064434, 0.0, 1739.4423246973906, 595.5405415362117, 0.0, 0.0, 1.0};	
				//cam_info_rgb_.R = camera_info_msg->R;
				//cam_info_rgb_.P = camera_info_msg->P;
				cam_info_rgb_.R = {1,0,0,0,1,0,0,0,1};
				cam_info_rgb_.P = {1743.4035352713363, 0.0, 760.3723854064434, 0.0,
				0.0, 1739.4423246973906, 595.5405415362117, 0.0,
				0.0, 0.0, 1.0, 0.0};
				cam_info_rgb_.binning_x = camera_info_msg->binning_x;
				cam_info_rgb_.binning_y = camera_info_msg->binning_y;
				cam_info_rgb_.roi = camera_info_msg->roi;	
			}
			else
			{
				cam_info_thermal_.header = camera_info_msg->header;
				cam_info_thermal_.height = camera_info_msg->height;
				cam_info_thermal_.width = camera_info_msg->width;
				cam_info_thermal_.distortion_model = camera_info_msg->distortion_model;
				cam_info_thermal_.D = {-8.7955719162052803e-03, 2.7957338512854757e-01, 2.9514273519729906e-03, -7.8091815268012512e-03, -1.0969845111284882e+00};
				cam_info_thermal_.K = {5.8651197564377128e+02, 0., 3.0317247522782532e+02, 0.,7.3675903031957341e+02,2.5406537636242152e+02,0.,0.,1.};	
				//cam_info_thermal_.R = camera_info_msg->R;
				//cam_info_thermal_.P = camera_info_msg->P;
				cam_info_thermal_.R = {1,0,0,0,1,0,0,0,1};
				cam_info_thermal_.P = {5.8651197564377128e+02, 0., 3.0317247522782532e+02, 0.0, 
				0.,7.3675903031957341e+02,2.5406537636242152e+02, 0.0,
				0.,0.,1.,0.};
				cam_info_thermal_.binning_x = camera_info_msg->binning_x;
				cam_info_thermal_.binning_y = camera_info_msg->binning_y;
				cam_info_thermal_.roi = camera_info_msg->roi;
			}
		}
	private:
		PclMsg last_pcl_;
		CompressedImageMsg last_thermal_;
		CompressedImageMsg last_rgb_;
		CameraInfoMsg cam_info_rgb_;
		CameraInfoMsg cam_info_thermal_;
		unsigned int num_frames_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fuser");
	ros::NodeHandle nh;
	
	Fuser fuser;
	
	// Subscribers
	// LiDAR
	
	ros::Subscriber pcl_sub = nh.subscribe("/raven/os_cloud_node/points", 1000, &Fuser::listen_pcl, &fuser);
	//sensor_msgs::PointCloud2ConstPtr pcl_first_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/raven/os_cloud_node/points", nh);
	pcl::PointCloud<pcl::PointXYZ> pcl_first_msg = *(ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>("/raven/os_cloud_node/points", nh));
	// Thermal
	ros::Subscriber thermal_sub = nh.subscribe("/raven/usb_cam/image_raw/compressed", 1000, &Fuser::listen_thermal, &fuser);
	sensor_msgs::CompressedImage thermal_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/raven/usb_cam/image_raw/compressed", nh));
	// Visual
	ros::Subscriber rgb_sub = nh.subscribe("/raven/image_raw/compressed", 1000, &Fuser::listen_rgb, &fuser);
	sensor_msgs::CompressedImage rgb_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/raven/image_raw/compressed", nh));
	
	// Create synched subscriber objects
	//message_filters::Subscriber<sensor_msgs::CompressedImage> rgb_sub_sync(nh, "/raven/image_raw/compressed", 1);
	//sensor_msgs::CompressedImage rgb_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/raven/image_raw/compressed", nh));

	//message_filters::Subscriber<sensor_msgs::CompressedImage> thermal_sub_sync(nh, "/raven/usb_cam/image_raw/compressed", 1);
	//sensor_msgs::CompressedImage thermal_first_msg = *(ros::topic::waitForMessage<sensor_msgs::CompressedImage>("/raven/usb_cam/image_raw/compressed", nh));

	//message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ>> lidar_sub_sync(nh, "/raven/os_cloud_node/points", 1);
	//pcl::PointCloud<pcl::PointXYZ> pcl_first_msg = *(ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>("/raven/os_cloud_node/points", nh));

	//message_filters::Synchronizer<SyncPolicy_vt> sync_vt(SyncPolicy_vt(1000), rgb_sub_sync, thermal_sub_sync);
	//message_filters::Synchronizer<SyncPolicy_vtl> sync_vtl(SyncPolicy_vtl(1000), rgb_sub_sync, thermal_sub_sync, lidar_sub_sync);
    //sync_vt.registerCallback(boost::bind(&Fuser::listen_cameras, &fuser, _1, _2));
	//sync_vtl.registerCallback(boost::bind(&Fuser::cb_sensors, &fuser, _1, _2, _3));
	//boost::shared_ptr<message_filters::Synchronizer<v_t_SyncPolicy>> sync;
	//boost::shared_ptr<message_filters::Synchronizer<v_t_l_SyncPolicy>> sync;
	//sync_vt.reset(new message_filters::Synchronizer<SyncPolicy_vt>(SyncPolicy_vt(10), rgb_sub_sync, thermal_sub_sync));
	//sync.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), rgb_sub, thermal_sub)); // 10 para 100
	// Subscribe
	//sync->registerCallback(boost::bind(&Fuser::listen_cameras, &fuser,_1, _2));
	
	// Camera infos
	sensor_msgs::CameraInfoConstPtr info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/raven/image_raw/camera_info", nh);
	fuser.set_camera_info(info);
	info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/raven/usb_cam/image_raw/camera_info", nh);
	fuser.set_camera_info(info);
	pubmatrix = nh.advertise<sensor_msgs::Image>("early_fused_input", 1);
	final_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("final_pcl", 1);

	tf::TransformListener tf_listener;
	
	ros::Rate rate(6);
	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		try
		{
			tf_listener.lookupTransform("/os_sensor", "/camera_frame", ros::Time(0), lidar2cam_tf);
    	}
		catch (tf::TransformException &ex) 
		{
		      ROS_WARN("%s",ex.what());
    	}
		//fuser.fuse();
	}
	return 0;
}