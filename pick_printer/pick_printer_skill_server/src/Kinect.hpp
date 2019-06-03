#ifndef KINECT_HPP
#define KINECT_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <string>

using namespace cv;
using namespace std;

class Kinect{

private:
	Mat img,img_depth;
	sensor_msgs::PointCloud2 cloud;
public:
	void set_image(const sensor_msgs::ImageConstPtr& msg);
	void set_image_depth(const sensor_msgs::ImageConstPtr& msg);
	void set_point(const sensor_msgs::PointCloud2Ptr& msg);
	void show_image();
	void show_image_depth();
	std::string type2str();
	Mat get_image(){return img;};
	Mat get_image_depth(){return img_depth;};
	sensor_msgs::PointCloud2 get_cloud(){return cloud;};
    bool cloud_filled=false;
};



#endif
