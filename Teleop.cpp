#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include <math.h>

#include "colour_matching.hpp"
#include "Teleop.hpp"

#include <iostream>

TurtlebotTeleop::TurtlebotTeleop():
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0),
  publish_sleep_(PUBLISH_SLEEP)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  publish_thread_ = boost::thread(boost::bind(&TurtlebotTeleop::publishLoop, this));  
}

TurtlebotTeleop::~TurtlebotTeleop()
{
//	publish_thread_.interrupt();
//	publish_thread_.join();	
}

SensorArray::SensorArray()
{
	grayCentroid_ =	redCentroid_ = blueCentroid_ = greenCentroid_ = 0;
	distance_ = theta_ = 0;
	x_ = y_ = orientation_ = 0;
	range_ = 0;
	bump_ = 0;
	
	current_time_ = previous_time_ = ros::Time::now();
}

SensorArray::~SensorArray()
{

}

void SensorArray::turtlesensor_callback(const turtlebot_node::TurtlebotSensorState& msg) 
{
	bump_ = msg.bumps_wheeldrops;
	distance_ += fabs(msg.distance);
}


void SensorArray::odom_callback(const nav_msgs::Odometry& msg)
{
	//do something
}

void SensorArray::imu_callback(const sensor_msgs::Imu& msg)
{
	static bool done_once = false;	

	if(!done_once)
	{
		//we do this once because initially previous_time_ and current_time_ are identical
		previous_time_ = ros::Time::now();
		done_once = true;
		return;
	}
	
	current_time_ = ros::Time::now();
		
	double vth = msg.angular_velocity.z;	
	double dt = (current_time_ - previous_time_).toSec();	
	
	double delta_th = dt * vth;
	
//	theta_ += fabs(delta_th); 
	theta_ = vth;
	
//		std::cout<<"Gyro: angular_velocity.z: "<<vth<<" dt: "<<dt<<" delta_th: "<<delta_th<<" theta: "<<theta_<<std::endl;
	
	previous_time_ = current_time_;	
}

void SensorArray::rgbcamera_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	
	cv::Mat rgbImg = cvPtr->image;
	
	cv::Rect roi(0, 240, 640, 240);
	cv::Mat croppedImg = rgbImg(roi);

	imshow("Kinect reading", croppedImg);
 	cv::waitKey(20);

	cv::Mat croppedRed = redFilter(croppedImg);	
	std::vector<unsigned char> scanline1 = getLine(croppedRed, croppedRed.rows / 2);	
	redCentroid_ = findBestCentroid(scanline1);
//	std::cout<<"Red Centroid: "<<redCentroid_<<std::endl;
	imshow("Kinect red", croppedRed);
	cv::waitKey(20);
	
	cv::Mat croppedGreen = greenFilter(croppedImg);
	std::vector<unsigned char> scanline2 = getLine(croppedGreen, croppedGreen.rows / 2);
	greenCentroid_ = findBestCentroid(scanline2);
//	std::cout<<"Green Centroid: "<<greenCentroid_<<std::endl;
	imshow("Kinect green", croppedGreen);
	cv::waitKey(20);
	
	cv::Mat croppedBlue = blueFilter(croppedImg);
	std::vector<unsigned char> scanline3 = getLine(croppedBlue, croppedBlue.rows / 2);
	blueCentroid_ = findBestCentroid(scanline3);
//	std::cout<<"Blue Centroid: "<<blueCentroid_<<std::endl;
	imshow("Kinect blue", croppedBlue);
	cv::waitKey(20);

}

void SensorArray::grayscalecamera_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	
	cv::Mat rgbImg = cvPtr->image;
	
	cv::Rect roi(0, 240, 640, 240);
	cv::Mat croppedImg = rgbImg(roi);

	imshow("Kinect reading", croppedImg);
 	cv::waitKey(20);
 
 cv::Mat croppedGray = grayFilter(croppedImg);
 std::vector<unsigned char> scanline = getGrayscaleLine(croppedGray, croppedGray.rows / 2); 
 grayCentroid_ = findBestCentroid(scanline);
 //std::cout<<"Gray Centroid: "<<grayCentroid_<<std::endl; 
 imshow("Kinect grayscale", croppedGray);
 cv::waitKey(20);		
}

void SensorArray::depthcamera_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); //16UC1
	
	cv::Mat depthImg = cvPtr->image;
	
	cv::Rect roi(0, 240, 640, 240);
	cv::Mat croppedImg = depthImg(roi);
	
	imshow("Kinect depth", croppedImg);
 	cv::waitKey(20);
 	
	unsigned short result = croppedImg.at<unsigned short>(croppedImg.rows / 2, croppedImg.cols / 2); //range of centre pixel
	
	range_ = result;
 }
 
void SensorArray::ekf_callback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
	//std::cout<<"ekf callback"<<std::endl;
	x_ = msg->pose.pose.position.x;
	y_ = msg->pose.pose.position.y;
	orientation_ = msg->pose.pose.orientation.z;
}

void TurtlebotTeleop::publishLoop()
{
	std::cout<<"Starting publish loop"<<std::endl;
	
	while(ros::ok())
	{
  	boost::this_thread::sleep(boost::posix_time::milliseconds(publish_sleep_));				
		boost::mutex::scoped_lock lock(publish_mutex_); //don't publish while we're updating
	  publish(angular_, linear_);	
	}

	std::cout<<"Finished publish loop"<<std::endl;
	return;
}

void TurtlebotTeleop::publish(double angular, double linear)  
{	
		static int i;
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;
//		std::cout<<"Publishing: "<<++i<<" linear: "<<linear<<" angular: "<<angular<<std::endl;
    vel_pub_.publish(vel);    

	  return;
}

void TurtlebotTeleop::setVelocity(double angular, double linear)
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	angular_ = angular;
	linear_ = linear;	
	return;
}

void TurtlebotTeleop::increaseLinear(double delta)
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	linear_ += delta;	
	return;
}

void TurtlebotTeleop::decreaseLinear(double delta)
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	linear_ -= delta;
	return;
}

void TurtlebotTeleop::increaseAngular(double delta)
{
		boost::mutex::scoped_lock lock(publish_mutex_);
		angular_ += delta;
		return;
}

void TurtlebotTeleop::decreaseAngular(double delta)
{
		boost::mutex::scoped_lock lock(publish_mutex_);
		angular_ -= delta;
		return;
}

void TurtlebotTeleop::stop()
{
		boost::mutex::scoped_lock lock(publish_mutex_);
		linear_ = 0;
		angular_ = 0;
		return;
}

void TurtlebotTeleop::setPublishSleep(int milliseconds)
{
	publish_sleep_ = milliseconds;
	return;
}

void TurtlebotTeleop::setLinearVelocity(double linear)
{
		boost::mutex::scoped_lock lock(publish_mutex_);
		linear_ = linear;
		return;
}

void TurtlebotTeleop::setAngularVelocity(double angular)
{
//		std::cout<<"Setting angular velocity"<<std::endl;
		boost::mutex::scoped_lock lock(publish_mutex_);
		angular_ = angular;
		return;
}
