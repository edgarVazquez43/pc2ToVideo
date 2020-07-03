#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include "rosbag_player/tools.h"

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"


namespace fs = boost::filesystem;

std::vector<std::string> lis_dir(std::string dir_name)
{
  std::vector<std::string> content_dir;
  // boost::progress_timer t( std::clog );

  fs::path full_path = fs::system_complete( fs::path( dir_name ));
  // std::cout << "\nusage:   simple_ls [path]" << std::endl;

  unsigned long file_count  = 0;
  unsigned long dir_count   = 0;
  unsigned long other_count = 0;
  unsigned long err_count   = 0;

  if ( !fs::exists( full_path ) )
  {
    std::cout << "\nNot found: " << full_path.filename() << std::endl;
    return content_dir;
  }

  if ( fs::is_directory( full_path ) )
  {
    // std::cout << "\nIn directory: "
    //           << full_path.filename() << "\n\n";
    fs::directory_iterator end_iter;
    for ( fs::directory_iterator dir_itr( full_path );
          dir_itr != end_iter;
          ++dir_itr )
    {
      try
      {
        if ( fs::is_directory( dir_itr->status() ) )
        {
          ++dir_count;
          // std::cout << dir_itr->path().filename() << " [directory]\n";
	  content_dir.push_back(std::string(dir_itr->path().string()));
	}
        else if ( fs::is_regular_file( dir_itr->status() ) )
        {
          ++file_count;
          // std::cout << dir_itr->path().filename() << "\n";
	  content_dir.push_back(std::string(dir_itr->path().string()));
	}
        else
        {
          ++other_count;
          std::cout << dir_itr->path().filename() << " [other]\n";
        }

      }
      catch ( const std::exception & ex )
      {
        ++err_count;
        std::cout << dir_itr->path().filename() << " " << ex.what() << std::endl;
      }
    }
    // std::cout << "\n" << file_count << " files\n"
    //           << dir_count << " directories\n"
    //           << other_count << " others\n"
    //           << err_count << " errors\n";
  }
  else // must be a file
  {
    std::cout << "\nFound: " << full_path.filename() << "\n";    
  }
  return content_dir;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_reader");
  ros::NodeHandle nh;
  std::cout << "Rosbag_reader_node    [initialization]" << std::endl;

  // cv variables declaration
  cv::Mat imgBGR;
  cv::Mat imgDepth;
  cv::VideoWriter outputVideo;
  cv::Size        size(640, 480);
  int             codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  double          fps   = 30;
  
  
  /// Publishers
  ros::Publisher pc_publisher = nh.advertise<sensor_msgs::PointCloud2>("pc", 1000);

  //Open Rosbag
  rosbag::Bag bag;

  //Set the ros loop in Hz
  ros::Rate loop(30);

  //Vector of source_data folders name
  std::vector<std::string> data_source;
  data_source.push_back("/media/edd/Dataset_drive/kinect_rosbag");
  data_source.push_back("/media/edd/Dataset_drive/xtion_rosbag");
  data_source.push_back("/media/edd/Dataset_drive/hsrb_rosbag");

  for(int data_source_index = 0; data_source_index < data_source.size(); data_source_index++)
  {
    std::cout << data_source[data_source_index] << std::endl;
  
    std::vector<std::string> content_dir = lis_dir(data_source[data_source_index]);
    if(data_source_index == 0)
      content_dir.erase(content_dir.begin(), content_dir.begin()+14);
	
    for(int user_index = 0; user_index < content_dir.size(); user_index++)
    {
      std::cout << content_dir[user_index] << std::endl;
      std::vector<std::string> videos_by_user = lis_dir(content_dir[user_index]);
       
	 
      
      for(int video_index = 0; video_index < videos_by_user.size(); video_index++)
      {
	// std::cout << "Open bag: " << std::endl;
	// std::cout << "  " << videos_by_user[video_index] << std::endl;

	/////////////////////////////////////////////////////////////
	//////      LOOP FOR READ AND TRANSFOR EACH ROS-BAG       ///

	std::string bag_name = videos_by_user[video_index];
	std::cout << "Try to open bag: [" << bag_name << "]" << std::endl;
	bag.open(bag_name);
	std::cout << "  - Open ros bag: [" << bag_name << "]   SUCCESS" << std::endl;

	//Open the video file
	std::string video_name = videos_by_user[video_index];
	video_name.replace(video_name.end()-4, video_name.end(), ".avi");
	outputVideo.open(video_name, codec, fps, size, true);
	std::cout << "  - Open video: [" << video_name << "]   SUCCESS" << std::endl;

      	//Loop for each ROS-BAG
      	for(rosbag::MessageInstance const m: rosbag::View(bag))
      	{
      	    if(ros::ok() && cv::waitKey(10) != 27)
      	    {
      		sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
      		if (i != nullptr)
      		{
      		    PointCloud2Msg_ToCvMat(i, imgBGR, imgDepth);
      		    //pc_publisher.publish(i);
      		    cv::imshow("RGB image", imgBGR);
      		    outputVideo.write(imgBGR);
      		    cv::waitKey(5);
      	        }
      
      		ros::spinOnce();
      		loop.sleep();
      	    }
      	    else{
      	      return 0;
      	    } 
	}
	// end-for

      	bag.close();
    

      }

    }
    
  }
  
  
 
  
  return 0;
}
