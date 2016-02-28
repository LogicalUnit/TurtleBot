/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <boost/thread/once.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sound_play/sound_play.h>
#include "Teleop.hpp"
#include "std_msgs/String.h"


void output_range(SensorArray& sensors)
{
	std::cout<<"Outputting Range"<<std::endl;
	
 	while(true)
 	{
 		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
 		std::cout<<"Range: "<<sensors.getRangeMillimeters()<<std::endl;;
 	}

}

void ellipse(TurtlebotTeleop& bot, SensorArray& sensors)
{
	std::cout<<"Starting Ellipse"<<std::endl;
		
	bot.setLinearVelocity(0.07);
	bot.setAngularVelocity(0);
	
	while(true)
	{
		unsigned int grayScaleCentroid = sensors.getGrayCentroid();
		
 		std::cout<<"Gray Centroid: "<<grayScaleCentroid<<std::endl;
 		
		if (grayScaleCentroid < 300)
		{
			std::cout<<"Turning Left"<<std::endl;
			bot.increaseAngular(0.005);
		}	
		else if (grayScaleCentroid > 340)
		{
			std::cout<<"Turning Right"<<std::endl;
			bot.decreaseAngular(0.005);
		}
		else
		{			
			bot.setAngularVelocity(0);
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));		
	}
}

void ekf_stats(SensorArray& sensors)
{


	while(true)
	{
	
		double x = sensors.getX();
		double y = sensors.getY();
		double orient = sensors.getOrientation();
		
		std::cout<<"x: "<<x<<" y: "<<y<<" orient: "<<orient<<std::endl;
		
/*		double theta = sensors.getTheta();
  	std::cout<<"Theta: "<<theta<<std::endl;
*/
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
	
}

void turn(TurtlebotTeleop& bot, SensorArray& sensors)
{
	double initial_angle = sensors.getOrientation();
	bot.setAngularVelocity(0.2);
	
	while(sensors.getOrientation() < initial_angle + 2*3.141/4)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
	
	bot.stop();

}

void colored_obstacles(TurtlebotTeleop& bot, SensorArray& sensors)
{
	while(true)
	{
		unsigned int redCentroid = sensors.getRedCentroid();
		unsigned int blueCentroid = sensors.getBlueCentroid();
		unsigned int greenCentroid = sensors.getGreenCentroid();
//		unsigned short range = sensors.getRangeMillimeters();
		
		std::cout<<"Red Center: "<<redCentroid<<" Blue Center: "<<blueCentroid<<" Green Center: "<<greenCentroid<<std::endl;
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

void print_range(SensorArray& sensors)
{
	while(true)
	{
		unsigned short range = sensors.getRangeMillimeters();
		
		std::cout<<"Range: "<<range<<std::endl;
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_teleop");
  
  static boost::once_flag cv_thread_flag = BOOST_ONCE_INIT;
  boost::call_once(cv_thread_flag, &cv::startWindowThread);
  
  cv::namedWindow("Kinect reading");
//  cv::namedWindow("Kinect red");
//	cv::namedWindow("Kinect blue");
//  cv::namedWindow("Kinect green");
  cv::namedWindow("Kinect grayscale");
//	cv::namedWindow("Kinect depth");
  
  TurtlebotTeleop bot;
	SensorArray sensors;  
  ros::NodeHandle n;
  
  
//	  ros::Subscriber colour_sub = n.subscribe("/camera/rgb/image_color", 1, &SensorArray::rgbcamera_callback, &sensors);
//usb_cam/image_raw
//	  	  ros::Subscriber colour_sub = n.subscribe("/usb_cam/image_raw", 1, &SensorArray::rgbcamera_callback, &sensors);
		ros::Subscriber grey_sub = n.subscribe("/usb_cam/image_raw", 1, &SensorArray::grayscalecamera_callback, &sensors);
//		ros::Subscriber depth_sub = n.subscribe("/camera/depth/image_raw", 1, &SensorArray::depthcamera_callback, &sensors);
//			ros::Subscriber imu_sub = n.subscribe("imu/data", 100, &SensorArray::imu_callback, &sensors);
//		ros::Subscriber odom_sub = n.subscribe("odom", 100, &SensorArray::odom_callback, &sensors);
//			ros::Subscriber ekf_sub = n.subscribe("/robot_pose_ekf/odom", 10, &SensorArray::ekf_callback, &sensors);
				
		boost::thread ellipse_thread(boost::bind(ellipse, boost::ref(bot), boost::ref(sensors)));
//  	boost::thread range_thread(boost::bind(output_range, boost::ref(sensors)));
//			boost::thread ekf_thread(boost::bind(ekf_stats, boost::ref(sensors)));
 //   	boost::thread turn_thread(boost::bind(turn, boost::ref(bot), boost::ref(sensors)));
 //			boost::thread colors_thread(boost::bind(colored_obstacles, boost::ref(bot), boost::ref(sensors)));
//			boost::thread range_thread(boost::bind(print_range, boost::ref(sensors)));

  
  std::cout<<"Spinning ROS"<<std::endl;
  ros::spin();
  std::cout<<"Finishing ROS"<<std::endl;

  return 0;
}
