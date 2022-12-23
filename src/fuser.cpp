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

// Aruco detector
#include <opencv2/aruco.hpp>

// C++ libs
#include <algorithm>

typedef pcl::PointCloud<pcl::PointXYZI> PCL;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> SyncPolicy;

const float leaf_size = 0.05;

std::string datetime()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y_%H-%M-%S",timeinfo);
    return std::string(buffer);
}

class Fuser
{
	public:
		Fuser(std::shared_ptr<ros::NodeHandle> nh)
		{
			nh_ = nh;
			// Set parameters
			num_frames_ = 0;
			max_depth_ = 7.0;
			tx_thermal_ = 0;
			ty_thermal_ = 0;
			remove_watermark_ = false;

			// Publishers
			concatenated_input_pub_ = nh->advertise<sensor_msgs::Image>("early_fused_input", 1);
			final_pcl_pub_ = nh->advertise<sensor_msgs::PointCloud2>("final_pcl", 1);

			// Aruco third-party library detection
			aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
			aruco_params_visual_ = cv::aruco::DetectorParameters::create();
			aruco_params_thermal_ = cv::aruco::DetectorParameters::create();

			// Initialize empty container for filtered pointcloud
			last_filtered_pcl_ = boost::make_shared<PCL>();

			// Load ros params
			nh->getParam("binary_threshold", binary_threshold_);
			nh->getParam("area_aruco_threshold", area_aruco_threshold_);
			nh->getParam("binarize_thermal", binarize_thermal_);
			nh->getParam("rgb_to_gray", rgb_to_gray_);
			nh->getParam("pcl_intensity_threshold", pcl_intensity_threshold_);
			nh->getParam("bag_namespace", bag_namespace_);
			nh->getParam("bag_path", bag_path_);
			nh->getParam("dataset_main_dir", dataset_main_dir_);
			nh->getParam("dataset_img_format", im_format_dataset_);
			nh->getParam("pcl_data_topic", pcl_data_topic_);
			nh->getParam("thermal_data_topic", thermal_data_topic_);
			nh->getParam("thermal_info_topic", thermal_info_topic_);
			nh->getParam("rgb_data_topic", rgb_data_topic_);
			nh->getParam("rgb_info_topic", rgb_info_topic_);
			nh->getParam("sync_sensors", sync_sensors_);
			nh->getParam("remove_watermark", remove_watermark_);
			nh->getParam("image_width", image_width_);
			nh->getParam("image_height", image_height_);
			nh->getParam("translation_thermal_x", tx_thermal_);
			nh->getParam("translation_thermal_y", ty_thermal_);

			// Spatial offset between thermal and visual source
			float warp_values[] = {1.0, 0.0, tx_thermal_, 0.0, 1.0, ty_thermal_};
			translation_thermal_ = new cv::Mat(2, 3, CV_32F, warp_values);

			// Auxiliar black image
			desired_size_ = cv::Size(image_width_, image_height_);
			black_image_ = cv::Mat::zeros(desired_size_, CV_8UC1);

			// Create datasets folders for all combination of inputs
			hash_dir_ = datetime() + "_" + "bag-" + bag_namespace_;

			datasets_dirs_["complete"] = dataset_main_dir_ + hash_dir_ + "_dataset-complete-input/";
			if (mkdir(datasets_dirs_["complete"].c_str(), 0777) == -1) std::cerr << "Error :  " << strerror(errno) << std::endl;
			else std::cout << "Directory " + datasets_dirs_["complete"] + "created\n";

			datasets_dirs_["visual"] = dataset_main_dir_ + hash_dir_ + "_dataset-visual-input/";
			if (mkdir(datasets_dirs_["visual"].c_str(), 0777) == -1) std::cerr << "Error :  " << strerror(errno) << std::endl;
			else std::cout << "Directory " + datasets_dirs_["visual"] + "created\n";

			datasets_dirs_["thermal"] = dataset_main_dir_ + hash_dir_ + "_dataset-thermal-input/";
			if (mkdir(datasets_dirs_["thermal"].c_str(), 0777) == -1) std::cerr << "Error :  " << strerror(errno) << std::endl;
			else std::cout << "Directory " + datasets_dirs_["thermal"] + "created\n";

			datasets_dirs_["pcl"] = dataset_main_dir_ + hash_dir_ + "_dataset-pcl-input/";
			if (mkdir(datasets_dirs_["pcl"].c_str(), 0777) == -1) std::cerr << "Error :  " << strerror(errno) << std::endl;
			else std::cout << "Directory " + datasets_dirs_["pcl"] + "created\n";

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
			fuse(*rgb_msg, *thermal_msg, *pcl_msg);
		}
		
		void fuse(sensor_msgs::CompressedImage rgb_msg, sensor_msgs::CompressedImage thermal_msg, sensor_msgs::PointCloud2 pcl_msg)
		{
			/// @brief image data
			cv_bridge::CvImagePtr rgb_image_ptr;
			cv_bridge::CvImagePtr thermal_image_ptr;

			// 1. Save image rgb message
			try
			{
				if (rgb_to_gray_) rgb_image_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::MONO8);
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
			// UNDISTORT + RECTIFY
			cv::Mat rm1, rm2;
			cv::initUndistortRectifyMap(K_rgb_, D_rgb_, R_rgb_, K_rgb_, cv::Size(info_rgb_.width, info_rgb_.height), CV_32FC1, rm1, rm2);
			cv::remap(rgb_image_ptr->image, rgb_image_ptr->image, rm1, rm2, cv::INTER_LINEAR);
			// RESIZE
			cv::resize(rgb_image_ptr->image, rgb_image_ptr->image, desired_size_);

			/* THERMAL */
			// UNDISTORT + RECTIFY
			rm1.release();
			rm2.release();
			cv::initUndistortRectifyMap(K_thermal_, D_thermal_, R_thermal_, K_thermal_, cv::Size(info_thermal_.width, info_thermal_.height), CV_32FC1, rm1, rm2); // @TODO: mudar isto dependendo se é crow ou raven
			cv::remap(thermal_image_ptr->image, thermal_image_ptr->image, rm1, rm2, cv::INTER_LINEAR);

			// RESIZE to rgb size
			cv::resize(thermal_image_ptr->image, thermal_image_ptr->image, rgb_image_ptr->image.size());
			
			// PAINT (flir water mark)
			if(remove_watermark_) cv::rectangle(thermal_image_ptr->image, cv::Point(460,10), cv::Point(560,70), cv::Scalar(0,0,0), cv::FILLED);

			// Binarize
			if (binarize_thermal_) cv::threshold(thermal_image_ptr->image, thermal_image_ptr->image, binary_threshold_, 255, 0);

			/// @brief Align thermal and visual sources
			// Detect visual and thermal aruco markers. If detection, align by centroid displacement, otherwise let hardcoded alignment.
			bool alignment_possible = find_alignment_rgb_thermal(rgb_image_ptr->image, thermal_image_ptr->image, translation_thermal_);
			cv::warpAffine(thermal_image_ptr->image, thermal_image_ptr->image, *translation_thermal_, thermal_image_ptr->image.size());

			/* Point Cloud */
			/// @brief Containers for original & filtered point cloud data
			pcl::PCLPointCloud2* original_pcl = new pcl::PCLPointCloud2;
			pcl::PCLPointCloud2ConstPtr original_pcl_const_ptr(original_pcl);
			pcl::PCLPointCloud2 filtered_pcl;

			/// @brief Get last pcl message data and convert it to pcl native
			pcl_conversions::toPCL(pcl_msg, *original_pcl);
			
			/// @brief Downsample the point cloud
			pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
			grid.setInputCloud(original_pcl_const_ptr);
			grid.setLeafSize(leaf_size, leaf_size, leaf_size);
			grid.filter(filtered_pcl);

			/// @brief Convert PCLPointCloud2 to pcl::PCLPointCloud<PointXYZ>
			pcl::fromPCLPointCloud2(filtered_pcl, *last_filtered_pcl_);
			
			/// @brief Crop pointcloud
			pcl::CropBox<pcl::PointXYZI> crop;
			if (bag_path_.find("20s-07") != std::string::npos || bag_path_.find("14-07") != std::string::npos)
			{
				crop.setInputCloud(last_filtered_pcl_);
				crop.setMin(Eigen::Vector4f(0, -1.0, -2.0, 1.0));
				crop.setMax(Eigen::Vector4f(max_depth_, 1.0, 2.0, 1.0));
				crop.filter(*last_filtered_pcl_);
			}
			else
			{
				crop.setInputCloud(last_filtered_pcl_);
				crop.setMin(Eigen::Vector4f(-2.0, 0, -2.0, 1.0));
				crop.setMax(Eigen::Vector4f(2.0, max_depth_, 2.0, 1.0));
				crop.filter(*last_filtered_pcl_);
			}

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
			image_geometry::PinholeCameraModel camera_geometry;
			camera_geometry.fromCameraInfo(info_rgb_);
			float depth;
			float u, v;

			// Image of depths
			cv::Mat pcl_image = cv::Mat::zeros(cv::Size(info_rgb_.width, info_rgb_.height), CV_8UC1);
			for (pcl::PointXYZI point : last_filtered_pcl_->points)
			{
				if (bag_path_.find("20s-07") != std::string::npos || bag_path_.find("14-07") != std::string::npos)
				{
					points3D.x = point.x; // xcam = zlaser
					points3D.y = point.y; // ycam = xlaser
					points3D.z = point.z; // zcam = ylaser
					depth = points3D.z; 
				}
				else
				{
					points3D.x = point.y; // xcam = zlaser
					points3D.y = point.z; // ycam = xlaser
					points3D.z = point.x; // zcam = ylaser
					depth = points3D.z; 
				}

				// Saturate depth
				if(depth >= max_depth_) depth = max_depth_;

				// Project 3D to 2D
				points2D = camera_geometry.project3dToPixel(points3D);	

				// Hardcoded Offset to apply to 14-07 bags
				if(bag_path_.find("20s-07") != std::string::npos || bag_path_.find("14-07") != std::string::npos)
				{
					points2D.x += 75;
					points2D.y -= 45;
				}

				// If point is intense (reflected by aruco) then save and paint that point
				if (points2D.x > 0 && points2D.x < info_rgb_.width && points2D.y > 0 && points2D.y < info_rgb_.height && point.intensity > pcl_intensity_threshold_)
				{
					// Draw pointcloud 2D image
					pcl_image.at<uchar>((int)points2D.y, (int)points2D.x) = 255 * (1.0 - depth / max_depth_);
				}
			}
			// Dilate image lidar points
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(30,30));
			cv::dilate(pcl_image, pcl_image, kernel);

			// Final resize
			cv::resize(pcl_image, pcl_image, desired_size_);

			/// @brief Generate datasets for all configuration of inputs
			// all inputs
			cv::Mat concatenated_input = generate_dataset_sample(rgb_image_ptr->image, thermal_image_ptr->image, pcl_image, datasets_dirs_["complete"]);
			// visual only
			generate_dataset_sample(rgb_image_ptr->image, black_image_, black_image_, datasets_dirs_["visual"]);
			// thermal only
			generate_dataset_sample(black_image_, thermal_image_ptr->image, black_image_, datasets_dirs_["thermal"]);
			// pcl only
			generate_dataset_sample(black_image_, black_image_, pcl_image, datasets_dirs_["pcl"]);

			/// @brief Publish concatenated input
			/// Define header
			std_msgs::Header header = last_rgb_.header;
			header.seq = num_frames_;

			/// Publish
			publish_concatenated_input(concatenated_input, header);

			std::cout << "Saved image samples idx " << num_frames_<< "\n";
			
			num_frames_++;
		}
		
		/// @brief Returns centroid and area of detected aruco
		/// @param x corners in clockwise order
		/// @param y corners in clockwise order
		/// @return array of doubles. position 0 and 1 are centroid x and y. position 2 is area
		std::array<double,3> aruco_centroid_and_area(std::array<double,4> xs, std::array<double,4> ys)
		{
			// Initialize area
			double area = 0.0;

			// Calculate value of shoelace formula
			int j = 3;
			for (int i = 0; i < 4; i++)
			{
				area += (xs[j] + xs[i]) * (ys[j] - ys[i]);
				j = i;  // j is previous vertex to i
			}
			// Ensemble output
			std::array<double,3> centroid_and_area;
			centroid_and_area[0] = 0.5 * (*std::max_element(xs.begin(), xs.end()) + *std::min_element(xs.begin(), xs.end()));
			centroid_and_area[1] = 0.5 * (*std::max_element(ys.begin(), ys.end()) + *std::min_element(ys.begin(), ys.end()));
			centroid_and_area[2] = abs(area / 2.0);
			return centroid_and_area;
		}

		bool find_alignment_rgb_thermal(cv::Mat visual_image, cv::Mat thermal_image, cv::Ptr<cv::Mat> translation_thermal)
		{
			std::vector<int> marker_ids_visual, marker_ids_thermal;
			std::vector<std::vector<cv::Point2f>> marker_corners_visual, rejected_candidates_visual, marker_corners_thermal, rejected_candidates_thermal;
			cv::aruco::detectMarkers(visual_image, aruco_dict_, marker_corners_visual, marker_ids_visual, aruco_params_visual_, rejected_candidates_visual);
			cv::aruco::detectMarkers(thermal_image, aruco_dict_, marker_corners_thermal, marker_ids_thermal, aruco_params_thermal_, rejected_candidates_thermal);

			int num_detections_visual = marker_ids_visual.size();
			int num_detections_thermal = marker_ids_thermal.size();
			std::array<double,2> best_centroid_visual, best_centroid_thermal;
			bool visual_aruco_detected = false;
			bool thermal_aruco_detected = false;
			if (num_detections_visual > 0 && num_detections_thermal > 0)
			{
				double max_area = -1000;
				/// @brief Extract the best visual marker detection by excluding false positives
				for(std::vector<cv::Point2f> detection_corners : marker_corners_visual)
				{
					std::array<double,4> xs, ys;
					for(int i = 0; i < 4; i++)
					{
						xs[i] = detection_corners[i].x; 
						ys[i] = detection_corners[i].y;
					}	

					std::array<double,3> centroid_and_area = aruco_centroid_and_area(xs, ys);
					if (num_detections_visual == 1 && centroid_and_area[2] > area_aruco_threshold_)
					{
						best_centroid_visual[0] = centroid_and_area[0];
						best_centroid_visual[1] = centroid_and_area[1];
						if(!visual_aruco_detected) visual_aruco_detected = true;
						break;
					}
					if (centroid_and_area[2] > area_aruco_threshold_ && centroid_and_area[2] > max_area)
					{
						max_area = centroid_and_area[2];
						best_centroid_visual[0] = centroid_and_area[0];
						best_centroid_visual[1] = centroid_and_area[1];
						if(!visual_aruco_detected) visual_aruco_detected = true;
					}
				}

				max_area = -1000;
				/// @brief Extract the best thermal marker detection by excluding false positives
				for(std::vector<cv::Point2f> detection_corners : marker_corners_thermal)
				{
					// Save corners
					std::array<double,4> xs, ys;
					for(int i = 0; i < 4; i++)
					{
						xs[i] = detection_corners[i].x; 
						ys[i] = detection_corners[i].y;
					}	
					// Find centroid and area of aruco
					std::array<double,3> centroid_and_area = aruco_centroid_and_area(xs, ys);
					if (num_detections_thermal == 1 && centroid_and_area[2] > area_aruco_threshold_)
					{
						best_centroid_thermal[0] = centroid_and_area[0];
						best_centroid_thermal[1] = centroid_and_area[1];
						if(!thermal_aruco_detected) thermal_aruco_detected = true;
						break;
					}
					if (centroid_and_area[2] > area_aruco_threshold_ && centroid_and_area[2] > max_area)
					{
						max_area = centroid_and_area[2];
						best_centroid_thermal[0] = centroid_and_area[0];
						best_centroid_thermal[1] = centroid_and_area[1];
						if(!thermal_aruco_detected) thermal_aruco_detected = true;
					}
				}
			}
			bool alignment_possible = thermal_aruco_detected && visual_aruco_detected;
			// If both visual and thermal aruco were detected, then assign a translation
			if(alignment_possible)
			{
				ROS_INFO("Automatic alignment between thermal and visual sources executed");
				translation_thermal_->at<float>(0, 2) = (best_centroid_visual[0] - best_centroid_thermal[0]);
				translation_thermal_->at<float>(1, 2) = (best_centroid_visual[1] - best_centroid_thermal[1]);
			}
			else
			{
				translation_thermal_->at<float>(0, 2) = tx_thermal_;
				translation_thermal_->at<float>(1, 2) = ty_thermal_;
			}
			return alignment_possible;
		}


		cv::Mat generate_dataset_sample(cv::Mat visual_channel, cv::Mat thermal_channel, cv::Mat pcl_channel, std::string dataset_folder_path)
		{
			cv::Mat output_image;
			std::vector<cv::Mat> channels;
			if (cv::countNonZero(visual_channel) < 1) channels.push_back(black_image_);
			else 
			{
				channels.push_back(visual_channel);
				cv::imwrite(dataset_folder_path + "im-visual-" + std::to_string(num_frames_) + "_" + hash_dir_ + im_format_dataset_, visual_channel);
			}
			if (cv::countNonZero(thermal_channel) < 1) channels.push_back(black_image_);
			else
			{
				channels.push_back(thermal_channel);
				cv::imwrite(dataset_folder_path + "im-thermal-" + std::to_string(num_frames_) + "_" + hash_dir_ + im_format_dataset_, thermal_channel);
			} 
			if (cv::countNonZero(pcl_channel) < 1) channels.push_back(black_image_);
			else 
			{
				channels.push_back(pcl_channel);
				cv::imwrite(dataset_folder_path + "im-pcl-" + std::to_string(num_frames_) + "_" + hash_dir_ + im_format_dataset_, pcl_channel);
			}

			/// @brief Concatenate
			try
			{
				cv::merge(channels, output_image);
			}
			catch (cv::Exception &e)
			{
				ROS_ERROR("Failed to merge output image. cv exception: %s", e.what());
				return black_image_;
			}
			
			/// @brief Save images
			cv::imwrite(dataset_folder_path + "im-concat-" + std::to_string(num_frames_) + "_" + hash_dir_ + im_format_dataset_, output_image);

			return output_image;
		}

		void publish_concatenated_input(cv::Mat concat_input, std_msgs::Header header)
		{
			sensor_msgs::Image image_msg;
			cv_bridge::CvImage image_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, concat_input);
			image_bridge.toImageMsg(image_msg);
			concatenated_input_pub_.publish(image_msg);
		}

		void load_cameras_info()
		{
			ROS_INFO("Loading Camera info...");
			sensor_msgs::CameraInfoConstPtr info_rgb_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(rgb_info_topic_);
			sensor_msgs::CameraInfoConstPtr info_thermal_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(thermal_info_topic_);
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

			// If old bags put hardcoded calibration parameters
			if (bag_path_.find("bags/old") != std::string::npos || bag_path_.find("bags/15-03") != std::string::npos)
			{
				// Thermal
				info_thermal_.D = {-8.7955719162052803e-03, 2.7957338512854757e-01, 2.9514273519729906e-03, -7.8091815268012512e-03, -1.0969845111284882e+00};
				info_thermal_.K = {5.8651197564377128e+02, 0., 3.0317247522782532e+02, 0.,7.3675903031957341e+02,2.5406537636242152e+02,0.,0.,1.};	
				info_thermal_.R = {1,0,0,0,1,0,0,0,1};
				info_thermal_.P = {5.8651197564377128e+02, 0., 3.0317247522782532e+02, 0.0, 
					0.,7.3675903031957341e+02,2.5406537636242152e+02, 0.0,
					0.,0.,1.,0.};
			}

			// Fill R and P missing values for visual source for all bags
			info_rgb_.R = {1,0,0,0,1,0,0,0,1};
			info_rgb_.P = {1743.4035352713363, 0.0, 760.3723854064434, 0.0,
					0.0, 1739.4423246973906, 595.5405415362117, 0.0,
					0.0, 0.0, 1.0, 0.0};

			K_rgb_ = cv::Mat(3, 3, CV_64F, &info_rgb_.K[0]);
			D_rgb_ = cv::Mat(5, 1, CV_64F, &info_rgb_.D[0]);
			R_rgb_ = cv::Mat(3, 3, CV_64F, &info_rgb_.R[0]);

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
		std::string hash_dir_;
		std::string im_format_dataset_;
		std::string dataset_main_dir_;
		std::string bag_namespace_;
		std::string bag_path_;
		std::map<std::string, std::string> datasets_dirs_;
		std::string pcl_data_topic_;
		std::string thermal_data_topic_;
		std::string thermal_info_topic_;
		std::string rgb_data_topic_;
		std::string rgb_info_topic_;
		int image_width_, image_height_;
		cv::Size desired_size_;
		bool rgb_to_gray_;
		int binary_threshold_;
		double area_aruco_threshold_;
		int pcl_intensity_threshold_;
		bool sync_sensors_;
		bool remove_watermark_;
		bool binarize_thermal_;
		float max_depth_;
		float tx_thermal_, ty_thermal_; // Translation to align thermal with visual image
		cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
		cv::Ptr<cv::aruco::DetectorParameters> aruco_params_visual_, aruco_params_thermal_;
		cv::Mat K_thermal_, D_thermal_, R_thermal_;
		cv::Mat K_rgb_, D_rgb_, R_rgb_;
		cv::Ptr<cv::Mat> translation_thermal_; // The translation matrix
		ros::Publisher concatenated_input_pub_;
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

	// Main cycle
	ros::Rate rate(10);	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
