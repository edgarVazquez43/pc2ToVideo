#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

void PointCloud2Msg_ToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
  // #################################
  //      Transform using PCL 
  // #################################
  
  //std::cout << "ObjectDetectorNode.-> Transforming from PointCloud2 ros message to cv::Mat type" << std::endl;
  //std::cout << "ObjectDetectorNode.-> Width= " << pc_msg.width << "  height= " << pc_msg.height << std::endl;
  pcl::PointCloud<pcl::PointXYZRGBA> pc_pcl;
  pcl::fromROSMsg(pc_msg, pc_pcl);  //Transform from PointCloud2 msg to pointCloud (from pcl) type

  if(!pc_pcl.isOrganized())
    {
      std::cout << "ObjectDetectorNode.->Point cloud is not organized!! O M G!!!!" << std::endl;
      return;
    }
  //std::cout << "ObjectDetectorNode.-> Pcl_w= " << pc_pcl.width << "  pcl_h= " << pc_pcl.height << std::endl;
  bgr_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_8UC3);
  pc_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_32FC3);
	
  // pcl::PointXYZRGBA p_ = pc_pcl.at(320, 240);
  // std::cout<<"ObjectDetectorNode: Middle point: "
  // 	   << p_.x << " "
  // 	   << p_.y << " "
  // 	   << p_.z << " "
  // 	   << (int)p_.r << " "
  // 	   << (int)p_.g << " "
  // 	   << (int)p_.b << std::endl;

  for (int h=0; h<bgr_dest.rows; h++)
    for (int w=0; w<bgr_dest.cols; w++)
      {
  	pcl::PointXYZRGBA p = pc_pcl.at(w, h);

  	bgr_dest.data[h*bgr_dest.step + w*3] = (unsigned char)p.b;
  	bgr_dest.data[h*bgr_dest.step + w*3 + 1] = (unsigned char)p.g;
  	bgr_dest.data[h*bgr_dest.step + w*3 + 2] = (unsigned char)p.r;
  	pc_dest.at<cv::Vec3f>(h,w)[0] = std::isnan(p.x) ? 0.0 : p.x;
  	pc_dest.at<cv::Vec3f>(h,w)[1] = std::isnan(p.y) ? 0.0 : p.y;
  	pc_dest.at<cv::Vec3f>(h,w)[2] = std::isnan(p.z) ? 0.0 : p.z;
      }
 

  
  // #################################
  // Transform using SR. MARC proccess
  // #################################

  
  // bgr_dest = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_8UC3);
  // pc_dest  = cv::Mat::zeros(pc_msg.height, pc_msg.width, CV_32FC3);
  // for(int i=0; i < bgr_dest.cols; i++)
  //   for(int j=0; j < bgr_dest.rows; j++)
  //     {
  // 	float* x = (float*)&pc_msg.data[(j*pc_msg.width + i)*32];
  // 	float* y = (float*)&pc_msg.data[(j*pc_msg.width + i)*32 + 4];
  // 	float* z = (float*)&pc_msg.data[(j*pc_msg.width + i)*32 + 8];
  // 	pc_dest.at<cv::Vec3f>(j, i)[0] = *x;
  // 	pc_dest.at<cv::Vec3f>(j, i)[1] = *y;
  // 	pc_dest.at<cv::Vec3f>(j, i)[2] = *z;
  // 	bgr_dest.data[j*bgr_dest.step + i*3]     = pc_msg.data[(j*pc_msg.width + i)*32 + 12];
  // 	bgr_dest.data[j*bgr_dest.step + i*3 + 1] = pc_msg.data[(j*pc_msg.width + i)*32 + 13];
  // 	bgr_dest.data[j*bgr_dest.step + i*3 + 2] = pc_msg.data[(j*pc_msg.width + i)*32 + 14];
  //     }
}


void PointCloud2Msg_ToCvMat(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{

  // ############################
  //       Transform using PCL
  // ############################

  pcl::PointCloud<pcl::PointXYZRGB> pc_pcl;
  pcl::fromROSMsg(*pc_msg, pc_pcl);  //Transform from PointCloud2 msg to pointCloud (from pcl) type

  if(!pc_pcl.isOrganized())
    {
      std::cout << "ObjectDetectorNode.->Point cloud is not organized!! O M G!!!!" << std::endl;
      return;
    }

  // std::cout << "ObjectDetectorNode.-> Pcl_w= " << pc_pcl.width
  // << "  pcl_h= " << pc_pcl.height << std::endl;
  
  bgr_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_8UC3);
  pc_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_32FC3);
	
  // pcl::PointXYZRGB p_ = pc_pcl.at(320, 240);
  // std::cout<<"ObjectDetectorNode: Middle point: "
  // 	   << p_.x << " "
  // 	   << p_.y << " "
  // 	   << p_.z << " "
  // 	   << (int)p_.r << " "
  // 	   << (int)p_.g << " "
  // 	   << (int)p_.b << std::endl;
  
  for (int h=0; h<bgr_dest.rows; h++)
    for (int w=0; w<bgr_dest.cols; w++)
      {
  	pcl::PointXYZRGB p = pc_pcl.at(w,h);
  	bgr_dest.data[h*bgr_dest.step + w*3] = (unsigned char)p.b;
  	bgr_dest.data[h*bgr_dest.step + w*3 + 1] = (unsigned char)p.g;
  	bgr_dest.data[h*bgr_dest.step + w*3 + 2] = (unsigned char)p.r;
  	pc_dest.at<cv::Vec3f>(h,w)[0] = std::isnan(p.x) ? 0.0 : p.x;
  	pc_dest.at<cv::Vec3f>(h,w)[1] = std::isnan(p.y) ? 0.0 : p.y;
  	pc_dest.at<cv::Vec3f>(h,w)[2] = std::isnan(p.z) ? 0.0 : p.z;
      }

  
  // #################################
  // Transform using SR. MARC proccess
  // #################################
  
  // bgr_dest = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_8UC3);
  // pc_dest  = cv::Mat::zeros(pc_msg->height, pc_msg->width, CV_32FC3);
  // for(int i=0; i < bgr_dest.cols; i++)
  //   for(int j=0; j < bgr_dest.rows; j++)
  //     {
  // 	float* x = (float*)&pc_msg->data[(j*pc_msg->width + i)*32];
  // 	float* y = (float*)&pc_msg->data[(j*pc_msg->width + i)*32 + 4];
  // 	float* z = (float*)&pc_msg->data[(j*pc_msg->width + i)*32 + 8];
  // 	pc_dest.at<cv::Vec3f>(j, i)[0] = *x;
  // 	pc_dest.at<cv::Vec3f>(j, i)[1] = *y;
  // 	pc_dest.at<cv::Vec3f>(j, i)[2] = *z;
  // 	bgr_dest.data[j*bgr_dest.step + i*3]     = pc_msg->data[(j*pc_msg->width + i)*32 + 12];
  // 	bgr_dest.data[j*bgr_dest.step + i*3 + 1] = pc_msg->data[(j*pc_msg->width + i)*32 + 13];
  // 	bgr_dest.data[j*bgr_dest.step + i*3 + 2] = pc_msg->data[(j*pc_msg->width + i)*32 + 14];
  //     }
}

std::string get_bag_name(std::string sensor_source, int user_number, int action_number)
{
  std::string dir_dataset   = "/media/edd/Dataset_drive/";
 
  return( dir_dataset + sensor_source + "/user_" + std::to_string(user_number) +
	  "/action" + std::to_string(action_number) + ".bag" ); 
}


std::string get_video_name(std::string sensor_source, int user_number, int action_number)
{
  std::string dir_dataset   = "/media/edd/Dataset_drive/";
  // std::cout << dir_dataset + sensor_source + "/user_" + std::to_string(user_number)
  //  + "/action" + std::to_string(action_number) + ".bag";
  
  return( dir_dataset + sensor_source + "/user_" + std::to_string(user_number) +
	  "/action" + std::to_string(action_number) + ".avi" ); 
}


void print_message()
{
  std::cout << "The library is right imported...." << std::endl;
}
