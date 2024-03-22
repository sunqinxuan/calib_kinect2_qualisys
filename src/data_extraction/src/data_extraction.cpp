/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2019-06-21 09:45
#
# Filename: data_extraction.cpp
#
# Description: 
#
************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <time.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
//#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <cv_bridge/cv_bridge.h>

class DataExtraction
{
public:
  enum Mode
  {
    COLLECT = 0,
    CALIBRATE,
	HD,
	QHD,
	SD
  };

private:

  const size_t queueSize;

  ros::NodeHandle nh;
  //ros::AsyncSpinner spinner;

  std::string folder_name;
  std::ofstream fp_rgb,fp_dep,fp_gt,fp_tf;
  bool received_cam_info_sd;
  bool received_cam_info_hd;

public:
  DataExtraction(const std::string &folder): queueSize(5), nh("~")//, spinner(0)
  {
	folder_name=folder;
	std::cout<<"folder_name="<<folder_name.c_str()<<std::endl;

	std::string mkdir_rgb="mkdir -p "+folder_name+"/rgb";
	std::string mkdir_dep="mkdir -p "+folder_name+"/depth";
	system(mkdir_rgb.c_str());
	system(mkdir_dep.c_str());

	std::string mkdir_rgb_hd="mkdir -p "+folder_name+"/rgb_hd";
	std::string mkdir_dep_hd="mkdir -p "+folder_name+"/depth_hd";
	system(mkdir_rgb_hd.c_str());
	system(mkdir_dep_hd.c_str());
  }

  void setFolderName(const std::string &name) { folder_name=name; }

  ~DataExtraction()
  {
  }

  void run()
  {
    start();
    stop();
  }

private:
  void start()
  {
	received_cam_info_hd=false;
	received_cam_info_sd=false;

	std::string topicCam_hd = "/kinect2/hd/camera_info";
	std::string topicRGB_hd = "/kinect2/hd/image_color";
	std::string topicDep_hd = "/kinect2/hd/image_depth_rect";

	std::string topicCam_sd = "/kinect2/sd/camera_info";
	std::string topicRGB_sd = "/kinect2/sd/image_color_rect";
	std::string topicDep_sd = "/kinect2/sd/image_depth";
	std::string topicGT  = "/qualisys/vi/pose";
//	std::string topicWM  = "/qualisys/tag/pose";
//	std::string topicTF  = "/tf";

	ros::Subscriber sub_cam_hd   = nh.subscribe(topicCam_hd, queueSize, &DataExtraction::callback_cam_hd, this);
	ros::Subscriber sub_rgb_hd   = nh.subscribe(topicRGB_hd, queueSize, &DataExtraction::callback_rgb_hd, this);
	ros::Subscriber sub_depth_hd = nh.subscribe(topicDep_hd, queueSize, &DataExtraction::callback_depth_hd, this);

	ros::Subscriber sub_cam_sd   = nh.subscribe(topicCam_sd, queueSize, &DataExtraction::callback_cam_sd, this);
	ros::Subscriber sub_rgb_sd   = nh.subscribe(topicRGB_sd, queueSize, &DataExtraction::callback_rgb_sd, this);
	ros::Subscriber sub_depth_sd = nh.subscribe(topicDep_sd, queueSize, &DataExtraction::callback_depth_sd, this);

	ros::Subscriber sub_gt    = nh.subscribe(topicGT, queueSize, &DataExtraction::callback_gt, this);
//	ros::Subscriber sub_wm    = nh.subscribe(topicWM.c_str(), queueSize, &DataExtraction::callback_wm, this);
//	ros::Subscriber sub_tf    = nh.subscribe(topicTF.c_str(), queueSize, &DataExtraction::callback_tf, this);

    //spinner.start();
	ros::spin();

  }

  void stop()
  {
    //spinner.stop();
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

	void callback_cam_hd(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
	{
		if(received_cam_info_hd) return;
		std::ofstream fp;
		std::string file_name=folder_name+"/cam_info_hd.txt";
		fp.open(file_name.c_str(),std::ios::out);
		fp<<camera_info->height<<std::endl;
		fp<<camera_info->width<<std::endl;
		for(int i=0;i<9;i++) fp<<camera_info->K[i]<<" ";
		fp<<std::endl;
		fp.close();
		received_cam_info_hd=true;
	}

	void callback_cam_sd(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
	{
		if(received_cam_info_sd) return;
		std::ofstream fp;
		std::string file_name=folder_name+"/cam_info.txt";
		fp.open(file_name.c_str(),std::ios::out);
		fp<<camera_info->height<<std::endl;
		fp<<camera_info->width<<std::endl;
		for(int i=0;i<9;i++) fp<<camera_info->K[i]<<" ";
		fp<<std::endl;
		fp.close();
		received_cam_info_sd=true;
	}

	void callback_rgb_sd(const sensor_msgs::Image::ConstPtr& imageColor)
	{
		std::string file_name=folder_name+"/rgb.txt";
		fp_rgb.open(file_name.c_str(),std::ios::app);

		cv::Mat color;
		readImage(imageColor,color);

		std::ostringstream name;
//		name.precision(10);
		name.setf(std::ios::fixed);

		double sec=imageColor->header.stamp.sec;
		double nsec=imageColor->header.stamp.nsec;
		double time_stamp_color=sec+nsec*1e-9;
		fp_rgb<<std::fixed<<time_stamp_color<<" rgb/"<<std::fixed<<time_stamp_color<<".png"<<std::endl;
		name<<folder_name<<"/rgb/"<<time_stamp_color<<".png";
		cv::imwrite(name.str().c_str(),color);

		fp_rgb.close();
	}

	void callback_depth_sd(const sensor_msgs::Image::ConstPtr imageDepth) 
	{
		std::string file_name=folder_name+"/depth.txt";
		fp_dep.open(file_name.c_str(),std::ios::app);

		cv::Mat depth;
		readImage(imageDepth,depth);

		std::ostringstream name;
//		name.precision(10);
		name.setf(std::ios::fixed);

		double sec=imageDepth->header.stamp.sec;
		double nsec=imageDepth->header.stamp.nsec;
		double time_stamp_depth=sec+nsec*1e-9;
		fp_dep<<std::fixed<<time_stamp_depth<<" depth/"<<std::fixed<<time_stamp_depth<<".png"<<std::endl;
		name<<folder_name<<"/depth/"<<time_stamp_depth<<".png";
		cv::imwrite(name.str().c_str(),depth);

		fp_dep.close();

//		// fake groundtruth file;
//		file_name=folder_name+"/groundtruth00.txt";
//		fp_gt.open(file_name.c_str(),std::ios::app);
//		fp_gt<<std::fixed<<time_stamp_depth<<" "
//			 <<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<1<<" "<<std::endl;
//		fp_gt.close();
	}

	void callback_rgb_hd(const sensor_msgs::Image::ConstPtr& imageColor)
	{
		std::string file_name=folder_name+"/rgb_hd.txt";
		fp_rgb.open(file_name.c_str(),std::ios::app);

		cv::Mat color;
		readImage(imageColor,color);

		std::ostringstream name;
//		name.precision(10);
		name.setf(std::ios::fixed);

		double sec=imageColor->header.stamp.sec;
		double nsec=imageColor->header.stamp.nsec;
		double time_stamp_color=sec+nsec*1e-9;
		fp_rgb<<std::fixed<<time_stamp_color<<" rgb_hd/"<<std::fixed<<time_stamp_color<<".png"<<std::endl;
		name<<folder_name<<"/rgb_hd/"<<time_stamp_color<<".png";
		cv::imwrite(name.str().c_str(),color);

		fp_rgb.close();
	}

	void callback_depth_hd(const sensor_msgs::Image::ConstPtr imageDepth) 
	{
		std::string file_name=folder_name+"/depth_hd.txt";
		fp_dep.open(file_name.c_str(),std::ios::app);

		cv::Mat depth;
		readImage(imageDepth,depth);

		std::ostringstream name;
//		name.precision(10);
		name.setf(std::ios::fixed);

		double sec=imageDepth->header.stamp.sec;
		double nsec=imageDepth->header.stamp.nsec;
		double time_stamp_depth=sec+nsec*1e-9;
		fp_dep<<std::fixed<<time_stamp_depth<<" depth_hd/"<<std::fixed<<time_stamp_depth<<".png"<<std::endl;
		name<<folder_name<<"/depth_hd/"<<time_stamp_depth<<".png";
		cv::imwrite(name.str().c_str(),depth);

		fp_dep.close();
	}

	void callback_gt(const geometry_msgs::PoseStamped::ConstPtr groundtruth)
	{
		std::string file_name=folder_name+"/groundtruth.txt";
		fp_gt.open(file_name.c_str(),std::ios::app);

		double sec=groundtruth->header.stamp.sec;
		double nsec=groundtruth->header.stamp.nsec;
		double time_stamp_gt=sec+nsec*1e-9;
		fp_gt<<std::fixed<<time_stamp_gt<<" "
			 <<groundtruth->pose.position.x<<" "<<groundtruth->pose.position.y<<" "<<groundtruth->pose.position.z<<" "
			 <<groundtruth->pose.orientation.x<<" "<<groundtruth->pose.orientation.y<<" "
			 <<groundtruth->pose.orientation.z<<" "<<groundtruth->pose.orientation.w<<" "
			 <<std::endl;
		
		fp_gt.close();
	}

	void callback_wm(const geometry_msgs::PoseStamped::ConstPtr markers)
	{
		std::string file_name=folder_name+"/markers.txt";
		fp_gt.open(file_name.c_str(),std::ios::app);

		double sec=markers->header.stamp.sec;
		double nsec=markers->header.stamp.nsec;
		double time_stamp_gt=sec+nsec*1e-9;
		fp_gt<<std::fixed<<time_stamp_gt<<" "
			 <<markers->pose.position.x<<" "<<markers->pose.position.y<<" "<<markers->pose.position.z<<" "
			 <<markers->pose.orientation.x<<" "<<markers->pose.orientation.y<<" "
			 <<markers->pose.orientation.z<<" "<<markers->pose.orientation.w<<" "
			 <<std::endl;
		
		fp_gt.close();
	}

//rosmsg show tf2_msgs/TFMessage 
//==============================
//geometry_msgs/TransformStamped[] transforms
//  std_msgs/Header header
//    uint32 seq
//    time stamp
//    string frame_id
//  string child_frame_id
//  geometry_msgs/Transform transform
//    geometry_msgs/Vector3 translation
//      float64 x
//      float64 y
//      float64 z
//    geometry_msgs/Quaternion rotation
//      float64 x
//      float64 y
//      float64 z
//      float64 w
	void callback_tf(const tf2_msgs::TFMessage::ConstPtr tf)
	{
		std::string file_name=folder_name+"/calibration.txt";
		fp_tf.open(file_name.c_str(),std::ios::app);

		double sec=tf->transforms[0].header.stamp.sec;
		double nsec=tf->transforms[0].header.stamp.nsec;
		double time_stamp_tf=sec+nsec*1e-9;
		fp_tf<<std::fixed<<time_stamp_tf<<" "
			 <<tf->transforms[0].transform.translation.x<<" "
			 <<tf->transforms[0].transform.translation.y<<" "
			 <<tf->transforms[0].transform.translation.z<<" "
			 <<tf->transforms[0].transform.rotation.x<<" "<<tf->transforms[0].transform.rotation.y<<" "
			 <<tf->transforms[0].transform.rotation.z<<" "<<tf->transforms[0].transform.rotation.w<<" "
			 <<std::endl;
		
		fp_tf.close();
	}


}; // class DataExtraction;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_extraction", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  std::string folder_name;

    time_t tt = time(NULL);
    struct tm* t= localtime(&tt);
	std::ostringstream ostr;
	ostr<<t->tm_year + 1900<<"-"<<t->tm_mon + 1<<"-" <<t->tm_mday<<"-"
		<<t->tm_hour<<"-" <<t->tm_min<<"-" <<t->tm_sec;
    folder_name="/home/sun/dataset/"+ostr.str();

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);
	if(param == "-O") folder_name="/home/sun/dataset/"+std::string(argv[i+1]);
  }

  DataExtraction data_extraction(folder_name);

  std::cout<<"awaiting rosbag play ..."<<std::endl;
  data_extraction.run();

  ros::shutdown();
  return 0;
}

