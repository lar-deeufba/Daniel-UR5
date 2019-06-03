#include "Kinect.hpp"


std::string Kinect::type2str() {
  int type= img.type();
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


void Kinect::set_image(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImageConstPtr cv_ptr;
	try
        {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
		img = cv_ptr->image;
	}
    	catch (cv_bridge::Exception& e)
        {
        	ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	}
}

void Kinect::set_image_depth(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
        try
        {
        	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
		img_depth=cv_ptr->image;
        }
    	catch (cv_bridge::Exception& e)
        {
        	ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	}
	
}


void Kinect::set_point(const sensor_msgs::PointCloud2Ptr& msg){
	cloud.header=msg->header;
	cloud.height=msg->height;
	cloud.width=msg->width;
	cloud.is_bigendian=msg->is_bigendian;
	cloud.point_step=msg->point_step;
	cloud.row_step=msg->row_step;
	cloud.is_dense=msg->is_dense;
	int sizef=msg->fields.size();
	int sized=msg->data.size();

	cloud.fields.resize(sizef);
	cloud.data.resize(sized);	

	for (int i=0; i < sizef; i++){
		cloud.fields[i].name = msg->fields[i].name;
		cloud.fields[i].offset= msg->fields[i].offset;
		cloud.fields[i].datatype= msg->fields[i].datatype;
		cloud.fields[i].count= msg->fields[i].count;
	}
		
	for (int i=0; i < sized; i++)
		cloud.data[i] = msg->data[i];
    cloud_filled=true;
}


void Kinect::show_image(){

	if(img.data)                              // Check for invalid input
    		{
			imshow( "Kinect RGB Image",img );  
			waitKey(5);
		}

}

void Kinect::show_image_depth(){

	if(img.data)                              // Check for invalid input
    		{
			imshow( "Kinect Depth Image",img_depth );  
			waitKey(5);
		}

}
