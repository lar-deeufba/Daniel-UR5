#ifndef OBJDETECTION_HPP
#define OBJDETECTION_HPP

#include <sensor_msgs/Image.h>
#include "Kinect.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <zbar.h>
#include <iostream>
#include <vector>
#include <apriltags_swatbotics/TagDetector.h>
#include <apriltags_swatbotics/DebugImage.h>
#include <array>

using namespace std;
using namespace zbar;
using namespace cv;
using namespace cv::xfeatures2d;

#define DEFAULT_TAG_FAMILY "Tag16h5"


typedef struct TagTestOptions {
  TagTestOptions() :
      show_debug_info(false),
      show_timing(false),
      show_results(false),
      be_verbose(false),
      no_images(true),
      generate_output_files(false),
      params(),
      family_str(DEFAULT_TAG_FAMILY),
      error_fraction(1){
  }
  bool show_debug_info;
  bool show_timing;
  bool show_results;
  bool be_verbose;
  bool no_images;
  bool generate_output_files;
  TagDetectorParams params;
  std::string family_str;
  double error_fraction;
} TagTestOptions;

class ObjDetec{
    private:
        typedef struct{
            string type;
            string data;
            vector <Point> location;
            } decodedObject;
        static void decode(Mat &im, vector<decodedObject>&decodedObjects);
        static Point display(Mat &im, vector<decodedObject>&decodedObjects, string code, bool show);
        //static void print_usage(const char* tool_name, FILE* output);
        static TagTestOptions parse_options(int argc, char** argv);
        void static pointsToPose(geometry_msgs::Point p1,geometry_msgs::Point p2);
    public: 
        static float ox,oy,oz;
        static geometry_msgs::Point pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v);
        static Point objDetec(Kinect cam,string image);
        static Point qrDetect(Kinect cam,string code,bool show);
        static Point aprilDetect(Kinect cam, unsigned int id, bool show);
        static void multAprilDetect(Kinect cam, unsigned int id[], Point output[], int size, bool show);

};



#endif
