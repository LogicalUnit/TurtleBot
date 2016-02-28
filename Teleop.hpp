#ifndef TURTLEBOT_TELEOP_H
#define TURTLEBOT_TELEOP_H

#include <ros/ros.h>
#include <turtlebot_node/TurtlebotSensorState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"


class TurtlebotTeleop
{
public:
  TurtlebotTeleop();
  ~TurtlebotTeleop();
  
  //default values
 	const static float LINEAR_INCREMENT = 0.1; 	// metres per second
	const static float ANGULAR_INCREMENT = 0.1; // radians per second
	const static int PUBLISH_SLEEP = 100; 			// milliseconds between publish updates
	
	//control the turtlebot
  void setVelocity(double angular, double linear); 
  void setLinearVelocity(double linear);
  void setAngularVelocity(double angular);
  void increaseLinear(double delta = LINEAR_INCREMENT);
  void decreaseLinear(double delta = LINEAR_INCREMENT);
  void increaseAngular(double delta = ANGULAR_INCREMENT);
  void decreaseAngular(double delta = ANGULAR_INCREMENT);
  void stop();
  
  //state of the turtlebot
  double getLinear() const { return linear_; }			//linear velocity, metres per second
  double getAngular() const { return angular_; }		//angular velocity, radians per second
	
	//manage internal thread
  void setPublishSleep(int milliseconds = PUBLISH_SLEEP); //time between publish updates. You probably don't need to change this
  int getPublishSleep() const { return publish_sleep_; }
 

private:
		
  double linear_, angular_;
  double l_scale_, a_scale_;
  int publish_sleep_; 	//milliseconds
  ros::NodeHandle nh_,ph_;
  ros::Publisher vel_pub_;
  boost::mutex publish_mutex_;
  boost::thread publish_thread_;
  
  void publishLoop();
  void publish(double angular, double linear);
};

class SensorArray
{
public:
	SensorArray();
	~SensorArray();

	//callback functions	
	void turtlesensor_callback(const turtlebot_node::TurtlebotSensorState& msg);
  void imu_callback(const sensor_msgs::Imu& msg);
  void odom_callback(const nav_msgs::Odometry& msg);
  void rgbcamera_callback(const sensor_msgs::ImageConstPtr& msg);
  void grayscalecamera_callback(const sensor_msgs::ImageConstPtr& msg);
  void depthcamera_callback(const sensor_msgs::ImageConstPtr& msg);
  void ekf_callback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);
  
  //accessor functions
  double getDistance() const { return distance_; }
  double getTheta() const { return theta_; }
	int getBump() const { return bump_; }
	double getX() const { return x_; }
	double getY() const { return y_; }
	double getOrientation() const { return orientation_; }
	
  unsigned short getRangeMillimeters() const { return range_; }

	//returns the index of the pixel of the best centroid
  unsigned int getGrayCentroid() const { return grayCentroid_; } 
  unsigned int getGreenCentroid() const { return greenCentroid_; }
  unsigned int getBlueCentroid() const { return blueCentroid_; }
  unsigned int getRedCentroid() const { return redCentroid_; }
   
  void resetDistance() { distance_ = 0; }
  void resetTheta() { theta_ = 0; }
  
  
private:

	double distance_, theta_;
	double x_, y_, orientation_;
	int bump_;
	unsigned int grayCentroid_, redCentroid_, blueCentroid_, greenCentroid_;
	unsigned short range_;
 	ros::Time current_time_, previous_time_;
};

#endif
