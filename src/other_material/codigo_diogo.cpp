/**
  Code developed by Diogo Silva

  **/

/*#####################
  ##     Includes    ##
  #####################*/

#include <ros/ros.h> ///Ros
#include <opencv2/opencv.hpp>///OpenCv
#include "camera/camera.h"///Camera
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <stdlib.h>

/*####################
  ##     Defines    ##
  ####################*/

#define VISUAL_CAMERA_IMG "image_raw/compressed" ///Name of the topic where the image of the termal camera are published
#define TERMAL_CAMERA_IMG "usb_cam/image_raw/compressed" ///Name of the topic where the image of the termal camera are published
#define NODE_NAME "Inter" ///Node Name



int main(int argc, char *argv[])
{
  /*##################################
    ##     Ros parameters initi     ##
    ##################################*/
    ros::init(argc, argv, NODE_NAME); // Node name
    ros::NodeHandle  n; // Node
    ros::NodeHandle nh("~");
    ros::Rate rate(10); // Publication rate

    std::cout<<NODE_NAME << "Node starting\n" ;

    /*#############################
      ##     Class instances     ##
      #############################*/
    std::cout<<"Class instances ." ;

    camera *visualCamera = new camera();
    std::cout<<"." ;
    camera *termalCamera = new camera();
    std::cout<<". done\n" ;

    /*############################
      ##     Ros Subscriber     ##
      ############################*/
    std::cout<<"Ros Subscriber ." ;
    ros::Subscriber visualCamera_img_sub = n.subscribe<sensor_msgs::CompressedImage>(VISUAL_CAMERA_IMG, 10, &camera::cb_camImg,visualCamera);
    std::cout<<"." ;
    ros::Subscriber termalCamera_img_sub = n.subscribe<sensor_msgs::CompressedImage>(TERMAL_CAMERA_IMG, 10, &camera::cb_camImg,termalCamera);
    std::cout<<". done\n" ;

    /*#######################
      ##     Variables     ##
      #######################*/

    //Parameters variables
    std::cout<<"Variables init .";

    int max_Iterations_Visual;//max number of iterations
    float min_Accuracy_Visual;// min accuracy
    int boardsize_Width_Visual;
    int boardsize_Height_Visual;
    int blob_MinTreshold_Visual;
    int blob_MaxTreshold_Visual;
    int numCircl_Visual;
    bool filterByArea_Visual;
    int minArea_Visual; //minArea may be adjusted to suit for your experiment
    int maxArea_Visual; //maxArea may be adjusted to suit for your experiment
    bool filterByCircularity_Visual;
    float minCircularity_Visual;
    bool filterByConvexity_Visual;
    float minConvexity_Visual;
    bool filterByInertia_Visual;
    float minInertiaRatio_Visual;
    float circDist_Visual;
    int nImg;
    int nImgV;

    nh.param<int>("max_Iterations_Visual", max_Iterations_Visual, 30);
    nh.param<float>("min_Accuracy_Visual", min_Accuracy_Visual, 0.1);
    nh.param<int>("boardsize_Width_Visual", boardsize_Width_Visual, 4);
    nh.param<int>("boardsize_Height_Visual", boardsize_Height_Visual, 7);
    nh.param<int>("blob_MinTreshold_Visual", blob_MinTreshold_Visual, 8);
    nh.param<int>("blob_MaxTreshold_Visual", blob_MaxTreshold_Visual, 255);
    nh.param<int>("numCircl_Visual", numCircl_Visual, 4*8);
    nh.param<bool>("filterByArea_Visual", filterByArea_Visual, true);
    nh.param<int>("minArea_Visual", minArea_Visual, 64);
    nh.param<int>("MaxArea_Visual", maxArea_Visual, 2500);
    nh.param<bool>("filterByCircularity_Visual", filterByCircularity_Visual, true);
    nh.param<float>("minCircularity_Visual", minCircularity_Visual, 0.1);
    nh.param<bool>("filterByConvexity_Visual", filterByConvexity_Visual, true);
    nh.param<float>("minConvexity_Visual", minConvexity_Visual, 0.87);
    nh.param<bool>("filterByConvexity_Visual", filterByInertia_Visual, true);
    nh.param<float>("minConvexity_Visual", minInertiaRatio_Visual, 0.87);
    nh.param<float>("minConvexity_Visual", circDist_Visual, 72.0);
    nh.param<int>("nImg", nImg, 1000);
    nh.param<int>("nImg", nImgV, 200);

    std::cout<<"." ;

    int max_Iterations_Termal;//max number of iterations
    float min_Accuracy_Termal;// min accuracy
    int boardsize_Width_Termal;
    int boardsize_Height_Termal;
    int blob_MinTreshold_Termal;
    int blob_MaxTreshold_Termal;
    int numCircl_Termal;
    bool filterByArea_Termal;
    int minArea_Termal; //minArea may be adjusted to suit for your experiment
    int maxArea_Termal; //maxArea may be adjusted to suit for your experiment
    bool filterByCircularity_Termal;
    float minCircularity_Termal;
    bool filterByConvexity_Termal;
    float minConvexity_Termal;
    bool filterByInertia_Termal;
    float minInertiaRatio_Termal;
    float circDist_Termal;

    nh.param<int>("max_Iterations_Visual", max_Iterations_Termal, 30);
    nh.param<float>("min_Accuracy_Visual", min_Accuracy_Termal, 0.1);
    nh.param<int>("boardsize_Width_Visual", boardsize_Width_Termal, 4);
    nh.param<int>("boardsize_Height_Visual", boardsize_Height_Termal, 7);
    nh.param<int>("blob_MinTreshold_Visual", blob_MinTreshold_Termal, 8);
    nh.param<int>("blob_MaxTreshold_Visual", blob_MaxTreshold_Termal, 255);
    nh.param<int>("numCircl_Visual", numCircl_Termal, 4*7);
    nh.param<bool>("filterByArea_Visual", filterByArea_Termal, true);
    nh.param<int>("minArea_Visual", minArea_Termal, 64);
    nh.param<int>("MaxArea_Visual", maxArea_Termal, 250);
    nh.param<bool>("filterByCircularity_Visual", filterByCircularity_Termal, true);
    nh.param<float>("minCircularity_Visual", minCircularity_Termal, 0.1);
    nh.param<bool>("filterByConvexity_Visual", filterByConvexity_Termal, true);
    nh.param<float>("minConvexity_Visual", minConvexity_Termal, 0.87);
    nh.param<bool>("filterByConvexity_Visual", filterByInertia_Termal, true);
    nh.param<float>("minConvexity_Visual", minInertiaRatio_Termal, 0.87);
    nh.param<float>("minConvexity_Visual", circDist_Termal, 72.0);

    std::cout<<"." ;

    // code variables
    int contVisual = 0;
    int contTermal = 0;

    std::vector<std::string> visualImgs;
    std::vector<std::string> termalImgs;

    std::cout<<".done\n";


    std::cout<<"Grabbing img's\n";

    //Grabbing img's from a bag
    while(ros::ok()){
      ros::spinOnce();

      if(visualCamera->getImageReady()==1 && contVisual < nImg){

        cv::imwrite("/home/diogo/Desktop/calib/Visual_" + std::to_string(contVisual)+".png", visualCamera->getImage()->image);
        contVisual++;
        std::cout<<"."<<std::flush;

      }

      if(termalCamera->getImageReady()==1 && contTermal < nImg){

        cv::imwrite("/home/diogo/Desktop/calib/Termal_" + std::to_string(contTermal)+".png", termalCamera->getImage()->image);
        contTermal++;
        std::cout<<"."<< std::flush;
      }
      if(contVisual==nImg && contTermal==nImg){
        std::cout<<"Grabbing img's done\n";
        break;
      }
      rate.sleep();
    }

    std::cout<<"BlobDetector param setup .";



    // the points on the chessboard
    std::vector<cv::Point2f> circleCentersVisual;
    std::vector<cv::Point2f> circleCentersTermal;

    std::vector<std::vector<cv::Point3f>> objectCornersVisual;
    std::vector<std::vector<cv::Point2f>> imgCenterVisual;
    std::vector<std::vector<cv::Point3f>> objectCornersTermal;
    std::vector<std::vector<cv::Point2f>> imgCenterTermal;

    //#####Setup SimpleBlobDetector parameters visual.####

    cv::SimpleBlobDetector::Params blobParamsVisual;

    // Change thresholds
    blobParamsVisual.minThreshold = blob_MinTreshold_Visual;
    blobParamsVisual.maxThreshold = blob_MaxTreshold_Visual;

    // Filter by Area.
    blobParamsVisual.filterByArea = filterByArea_Visual;
    blobParamsVisual.minArea = minArea_Visual;//minArea may be adjusted to suit for your experiment
    blobParamsVisual.maxArea = maxArea_Visual;//maxArea may be adjusted to suit for your experiment

    //Filter by Circularity
    blobParamsVisual.filterByCircularity = filterByCircularity_Visual ;
    blobParamsVisual.minCircularity = minCircularity_Visual;

    //Filter by Convexity
    blobParamsVisual.filterByConvexity = filterByConvexity_Visual;
    blobParamsVisual.minConvexity = minConvexity_Visual;

    //Filter by Inertia
    blobParamsVisual.filterByInertia = filterByInertia_Visual;
    blobParamsVisual.minInertiaRatio = minInertiaRatio_Visual;
    std::cout<<"." ;


    //#####Setup SimpleBlobDetector parameters termal.####

    cv::SimpleBlobDetector::Params blobParamsTermal;

    // Change thresholds
    blobParamsTermal.minThreshold = blob_MinTreshold_Termal;
    blobParamsTermal.maxThreshold = blob_MaxTreshold_Termal;

    // Filter by Area.
    blobParamsTermal.filterByArea = filterByArea_Termal;
    blobParamsTermal.minArea = minArea_Termal;//minArea may be adjusted to suit for your experiment
    blobParamsTermal.maxArea = maxArea_Termal;//maxArea may be adjusted to suit for your experiment

    //Filter by Circularity
    blobParamsTermal.filterByCircularity = filterByCircularity_Termal ;
    blobParamsTermal.minCircularity = minCircularity_Termal;

    //Filter by Convexity
    blobParamsTermal.filterByConvexity = filterByConvexity_Termal;
    blobParamsTermal.minConvexity = minConvexity_Termal;

    //Filter by Inertia
    blobParamsTermal.filterByInertia = filterByInertia_Termal;
    blobParamsTermal.minInertiaRatio = minInertiaRatio_Termal;


    std::cout<<". done\n";

    std::cout<<"Create BlobDetector .";
    cv::Ptr<cv::SimpleBlobDetector> blobDetectorVisual = cv::SimpleBlobDetector::create(blobParamsVisual);
    std::cout<<"." ;
    cv::Ptr<cv::SimpleBlobDetector> blobDetectorTermal = cv::SimpleBlobDetector::create(blobParamsTermal);

    std::cout<<". done\n";

    std::cout<<"Define circle positions .";
    std::vector<cv::Point3f> mat;

    float bias =0;
    bool flag = true;
    int j=0;
    for(int i{0}; i<7; i++){
      if(i%2 !=0 && flag){
        bias=0.05/2;
        std::cout<<"i = "<<i<<" \n";

        flag = false;
      }

      for( j=0; j<4; j++){
         mat.push_back(cv::Point3f(0.05*i/2,0.05*j+bias,0));
     }

     bias=0;
     flag = true;

     }
    std::cout<<". done\n";

    std::cout<<"Chose img's\n";
    cv::Mat im_with_keypoints_visual,im_with_keypoints_Termal;
    cv::Mat grayimageVisual, grayimageTermal,im_with_keypoints_gray_visual,im_with_keypoints_gray_Termal;
    int foundV =0;
    int foundT =0;
    bool flagV=false;
    bool flagT=false;
    std::cout<<"Chose mode for visual\n";
    char liness[256];
    if (fgets(liness, sizeof liness, stdin) == NULL) {
        printf("Input error.\n");
        exit(1);
    }
    char respVa = liness[0];
    if(respVa=='s' || respVa=='S'){
      flagV=true;
    }
    std::cout<<"Chose mode for Termica\n";
    if (fgets(liness, sizeof liness, stdin) == NULL) {
        printf("Input error.\n");
        exit(1);
    }
    char respTa = liness[0];
    if(respTa=='s' || respTa=='S'){
      flagT=true;
    }
    for (int i=0; i<nImg;i++) {
      cv::Mat imageVisual, imageTermal;
      //std::cout<<"Load "<<"Visual_" + std::to_string(i)+".png"<<"\n";
      //std::cout<<"Load "<<"Termal_" + std::to_string(i)+".png"<<"\n";
      imageVisual = cv::imread("/home/diogo/Desktop/calib/Visual_" + std::to_string(i)+".png",  cv::IMREAD_COLOR );
      imageTermal = cv::imread("/home/diogo/Desktop/calib/Termal_" + std::to_string(i)+".png",  cv::IMREAD_COLOR );
      // std::cout<<"Load Img's done\n";

      //std::cout<<"Convert to gray\n";
      cv::cvtColor(imageVisual,grayimageVisual,cv::COLOR_BGR2GRAY);
      cv::cvtColor(imageTermal,grayimageTermal,cv::COLOR_BGR2GRAY);
      //std::cout<<"Convert to gray done\n";

      std::vector<cv::KeyPoint> keyPointsVisual;
      std::vector<cv::KeyPoint> keyPointsTermal;

      //std::cout<<"BlobDetector init\n";
      blobDetectorVisual->detect(grayimageVisual,keyPointsVisual);
      blobDetectorTermal->detect(grayimageTermal,keyPointsTermal);

      //std::cout<<"BlobDetector init done\n";

      //std::cout<<"drawKeypoints init\n";
      cv::drawKeypoints(imageVisual,keyPointsVisual,im_with_keypoints_visual,CV_RGB(0,255,0),cv::DrawMatchesFlags().DRAW_RICH_KEYPOINTS);
      cv::drawKeypoints(imageTermal,keyPointsTermal,im_with_keypoints_Termal,CV_RGB(0,255,0),cv::DrawMatchesFlags().DRAW_RICH_KEYPOINTS);
      //std::cout<<"drawKeypoints done\n";

      //std::cout<<"Convert to gray init\n";
      cv::cvtColor(im_with_keypoints_visual,im_with_keypoints_gray_visual,cv::COLOR_BGR2GRAY);
      cv::cvtColor(im_with_keypoints_Termal,im_with_keypoints_gray_Termal,cv::COLOR_BGR2GRAY);

     //std::cout<<"Convert to gray done\n";

      //std::cout<<"findCirclesGrid init\n";
      bool retV = cv::findCirclesGrid(im_with_keypoints_visual,cv::Size(boardsize_Width_Visual,boardsize_Height_Visual),circleCentersVisual, cv::CALIB_CB_ASYMMETRIC_GRID);
      bool retT = cv::findCirclesGrid(im_with_keypoints_Termal,cv::Size(boardsize_Width_Termal,boardsize_Height_Termal),circleCentersTermal, cv::CALIB_CB_ASYMMETRIC_GRID);

      //std::cout<<"findCirclesGrid done\n";
      bool termalVal = false, visualVal = false;
      if(retV && foundV<nImgV){

//        std::cout<<"Add objectCorners \n";
//        objectCornersVisual.push_back(mat);
//        std::cout<<"Add objectCorners done\n";

        //std::cout<<"cornerSubPix init\n";
        cv::cornerSubPix(im_with_keypoints_gray_visual,circleCentersVisual,cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));

        //std::cout<<"cornerSubPix done\n";

        //std::cout<<"DrawChessboardCorners init\n";
        cv::drawChessboardCorners(im_with_keypoints_visual,cv::Size(boardsize_Width_Visual,boardsize_Height_Visual),circleCentersVisual,retV);
        //std::cout<<"DrawChessboardCorners done\n";

        visualVal = true;
      }

      if(retT && foundT<nImgV){


        //std::cout<<"cornerSubPix Termal init\n";
        cv::cornerSubPix(im_with_keypoints_gray_Termal,circleCentersTermal,cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));
        //std::cout<<"cornerSubPix Termal done\n";



        //std::cout<<"DrawChessboardCorners init\n";
        cv::drawChessboardCorners(im_with_keypoints_Termal,cv::Size(boardsize_Width_Termal,boardsize_Height_Termal),circleCentersTermal,retT);
        //std::cout<<"DrawChessboardCorners done\n";

        termalVal = true;
      }

      if(foundV>nImgV-1 && foundT>nImgV-1)
        break;

      //std::cout<<"img init\n";
      std::string file_rgV = "/home/diogo/Desktop/calib/Visual_" + std::to_string(i)+".png";
      std::string file_rgT = "/home/diogo/Desktop/calib/Termal_" + std::to_string(i)+".png";


      if(visualVal){
        char respV;
        if(flagV){
          cv::resize(im_with_keypoints_visual,im_with_keypoints_visual,cv::Size(im_with_keypoints_visual.size().width*0.5,im_with_keypoints_visual.size().height*0.5));
          cv::imshow("imgV",im_with_keypoints_visual);
          cv::waitKey(100);

          std::cout<<"Utilizar esta imagem visual?s/n\n";
          char line[256];
          if (fgets(line, sizeof line, stdin) == NULL) {
              printf("Input error.\n");
              exit(1);
          }
          respV = line[0];
          if(respV=='s' || respV=='S'){
            foundV++;
            visualImgs.push_back(file_rgV);
          }
        }
        else {
          foundV++;

          visualImgs.push_back(file_rgV);
        }
      }
      if(flagT){
        cv::imshow("imgt",im_with_keypoints_Termal);
        cv::waitKey(100);
        if(termalVal){
          char respT;
          std::cout<<"Utilizar esta imagem termica?s/n\n";
          char line[256];
          if (fgets(line, sizeof line, stdin) == NULL) {
              printf("Input error.\n");
              exit(1);
          }
          respT = line[0];
          if(respT=='s' || respT=='S'){

            foundT++;
            std::cout<<"IMG "<<foundT<<"/"<<nImgV<<" Disponiveis"<<i<<"/"<<nImg<<"\n";
            termalImgs.push_back(file_rgT);
          }
        }
      }
      else{
        foundT++;
        termalImgs.push_back(file_rgT);
      }

    }
  if (foundV < nImgV || foundT<nImgV)
    return -1;
    foundV=0;
    foundT=0;


    for (int i=0; i<visualImgs.size();i++) {//visualImgs termalImgs
      cv::Mat imageVisual, imageTermal;
      std::cout<<"inicio\n";
      imageVisual = cv::imread(visualImgs[i],  cv::IMREAD_COLOR );
      imageTermal = cv::imread(termalImgs[i],  cv::IMREAD_COLOR );

      cv::cvtColor(imageVisual,grayimageVisual,cv::COLOR_BGR2GRAY);
      cv::cvtColor(imageTermal,grayimageTermal,cv::COLOR_BGR2GRAY);

      std::vector<cv::KeyPoint> keyPointsVisual;
      std::vector<cv::KeyPoint> keyPointsTermal;

      blobDetectorVisual->detect(grayimageVisual,keyPointsVisual);
      blobDetectorTermal->detect(grayimageTermal,keyPointsTermal);

      cv::drawKeypoints(imageVisual,keyPointsVisual,im_with_keypoints_visual,CV_RGB(0,255,0),cv::DrawMatchesFlags().DRAW_RICH_KEYPOINTS);
      cv::drawKeypoints(imageTermal,keyPointsTermal,im_with_keypoints_Termal,CV_RGB(0,255,0),cv::DrawMatchesFlags().DRAW_RICH_KEYPOINTS);

      cv::cvtColor(im_with_keypoints_visual,im_with_keypoints_gray_visual,cv::COLOR_BGR2GRAY);
      cv::cvtColor(im_with_keypoints_Termal,im_with_keypoints_gray_Termal,cv::COLOR_BGR2GRAY);


      bool retV = cv::findCirclesGrid(im_with_keypoints_visual,cv::Size(boardsize_Width_Visual,boardsize_Height_Visual),circleCentersVisual, cv::CALIB_CB_ASYMMETRIC_GRID);
      bool retT = cv::findCirclesGrid(im_with_keypoints_Termal,cv::Size(boardsize_Width_Termal,boardsize_Height_Termal),circleCentersTermal, cv::CALIB_CB_ASYMMETRIC_GRID);

      if(retV && foundV<nImgV){

        //std::cout<<"Add objectCorners \n";
        objectCornersVisual.push_back(mat);
        //std::cout<<"Add objectCorners done\n";

        cv::cornerSubPix(im_with_keypoints_gray_visual,circleCentersVisual,cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));


        imgCenterVisual.push_back(circleCentersVisual);

        cv::drawChessboardCorners(im_with_keypoints_visual,cv::Size(boardsize_Width_Visual,boardsize_Height_Visual),circleCentersVisual,retV);
        foundV++;
      }

      if(retT && foundT<nImgV){


        //std::cout<<"cornerSubPix Termal init\n";
        cv::cornerSubPix(im_with_keypoints_gray_Termal,circleCentersTermal,cv::Size(11,11),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));
        //std::cout<<"cornerSubPix Termal done\n";
        objectCornersTermal.push_back(mat);
       // std::cout<<"Add circleCenters Termal\n";
        imgCenterTermal.push_back(circleCentersTermal);
       // std::cout<<"Add circleCenters Termal done\n";

       // std::cout<<"DrawChessboardCorners init\n";
        cv::drawChessboardCorners(im_with_keypoints_Termal,cv::Size(boardsize_Width_Termal,boardsize_Height_Termal),circleCentersTermal,retT);
       // std::cout<<"DrawChessboardCorners done\n";

        foundT++;
      }

      if(foundV>nImgV-1 && foundT>nImgV-1 )
        break;

      //std::cout<<"img init\n";
      std::string file_rgV = "/home/diogo/Desktop/calib/Visual_" + std::to_string(i)+".png";
      std::string file_rgT = "/home/diogo/Desktop/calib/Termal_" + std::to_string(i)+".png";

      cv::resize(im_with_keypoints_visual,im_with_keypoints_visual,cv::Size(im_with_keypoints_visual.size().width*0.5,im_with_keypoints_visual.size().height*0.5));
      cv::imshow("imgV",im_with_keypoints_visual);
      cv::waitKey(100);



      cv::imshow("imgt",im_with_keypoints_Termal);
      cv::waitKey(100);

     // std::cout<<"asdsakldçjasdaks,d init\n";


    }




    //std::cout<<"saltou\n";

    std::vector<cv::Mat> rvecsV, tvecsV;
    cv::Mat K_cameraMatrixV;
    cv::Mat LensDistortion_coefV;
    //std::cout<<"img size "<<imgCenterVisual.size()<<"\n";
    //std::cout<<"gray size "<<grayimageVisual.size()<<"\n";


    cv::calibrateCamera(objectCornersVisual,imgCenterVisual,cv::Size(grayimageVisual.rows,grayimageVisual.cols),K_cameraMatrixV,LensDistortion_coefV,rvecsV,tvecsV);
    std::cout << "ImgSize" << grayimageVisual.size()<<"\n";
    std::cout << "Intrinsics" << K_cameraMatrixV<<"\n";
    std::cout << "Distortion" << LensDistortion_coefV<<"\n";
    cv::FileStorage fs_KD("/home/diogo/Desktop/calib/IntrinsicsDistortionV.xml", cv::FileStorage::WRITE);
        if (fs_KD.isOpened())
        {

          fs_KD << "ImgSize" << grayimageVisual.size();
            fs_KD << "Intrinsics" << K_cameraMatrixV;
            fs_KD << "Distortion" << LensDistortion_coefV;
            fs_KD.release();
        }

        std::cout << "init Undistor\n";

        cv::Mat mapx, mapy;
        cv::initUndistortRectifyMap(
                            K_cameraMatrixV,  // computed camera matrix
                            LensDistortion_coefV,    // computed distortion matrix
                            cv::Mat(),     // optional rectification (none)
                            cv::Mat(),     // camera matrix to generate undistorted
                            grayimageVisual.size(),          //            image.size(),  // size of undistorted
                            CV_32FC1,      // type of output map
                            mapx, mapy);
        std::cout << "init Undistor done\n";

      cv::Mat imgCalib,imgO;
      for (int i=0; i<visualImgs.size();i++){
        std::cout << "remap \n";
        imgO=cv::imread(visualImgs[i],  cv::IMREAD_COLOR );
        cv::remap(imgO, imgCalib, mapx, mapy, cv::INTER_LINEAR);
        std::cout << "remap done\n";

        cv::resize(imgO,imgO,cv::Size(imgO.size().width*0.5,imgO.size().height*0.5));
        cv::resize(imgCalib,imgCalib,cv::Size(imgCalib.size().width*0.5,imgCalib.size().height*0.5));

        cv::imshow("imgtaa",imgO);
        cv::imshow("imgsast",imgCalib);
        cv::waitKey(0);
      }

      std::vector<cv::Mat> rvecsT, tvecsT;
      cv::Mat K_cameraMatrixT;
      cv::Mat LensDistortion_coefT;


      cv::calibrateCamera(objectCornersTermal,imgCenterTermal,cv::Size(grayimageTermal.rows,grayimageTermal.cols),K_cameraMatrixT,LensDistortion_coefT,rvecsT,tvecsT);
      std::cout << "ImgSize" << grayimageTermal.size()<<"\n";
      std::cout << "Intrinsics" << K_cameraMatrixT<<"\n";
      std::cout << "Distortion" << LensDistortion_coefT<<"\n";
      cv::FileStorage fs_KDT("/home/diogo/Desktop/calib/IntrinsicsDistortionT.xml", cv::FileStorage::WRITE);
          if (fs_KDT.isOpened())
          {

            fs_KDT << "ImgSize" << grayimageTermal.size();
              fs_KDT << "Intrinsics" << K_cameraMatrixT;
              fs_KDT << "Distortion" << LensDistortion_coefT;
              fs_KDT.release();
          }

          std::cout << "init Undistor\n";

          cv::Mat mapxT, mapyT;
          cv::initUndistortRectifyMap(
                              K_cameraMatrixT,  // computed camera matrix
                              LensDistortion_coefT,    // computed distortion matrix
                              cv::Mat(),     // optional rectification (none)
                              cv::Mat(),     // camera matrix to generate undistorted
                              grayimageTermal.size(),          //            image.size(),  // size of undistorted
                              CV_32FC1,      // type of output map
                              mapxT, mapyT);
          std::cout << "init Undistor done\n";

        cv::Mat imgCalibT,imgOT;
        for (int i=0; i<termalImgs.size();i++){
          std::cout << "remap \n";
          imgOT=cv::imread(termalImgs[i],  cv::IMREAD_COLOR );
          cv::remap(imgOT, imgCalibT, mapxT, mapyT, cv::INTER_LINEAR);
          std::cout << "remap done\n";

          cv::resize(imgOT,imgOT,cv::Size(imgOT.size().width*0.5,imgOT.size().height*0.5));
          cv::resize(imgCalibT,imgCalibT,cv::Size(imgCalibT.size().width,imgCalibT.size().height));

          cv::imshow("imgtaa",imgOT);
          cv::imshow("imgsast",imgCalibT);
          cv::waitKey(0);

        }

  return 0;
}
