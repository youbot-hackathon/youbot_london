#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "ColorBlobDetector.hpp"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <cvblob.h>
#include <sstream>
#include <ros/console.h>

#include <imachines2/cubelist.h>
#include <imachines2/cube.h>
#include <imachines2/frames.h>
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
static const int DEPTH_DIST = 13;
static const double PIXEL_CM = 54.0;

using namespace std;
using namespace cvb;

class ImageAnalyzer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher cubes_pub;
  ros::Subscriber frames_sub;
  ColorBlobDetector* detector;
  CvFont font;
  int msg_no;
  int frames_to_read;

public:
  ImageAnalyzer() :
      it_(nh_)
  {

    frames_sub = nh_.subscribe("/frame_control", 1000, &ImageAnalyzer::controllerCb, this);
    cubes_pub = nh_.advertise<imachines2::cubelist>("cubelist", 1000);
    detector = NULL;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN | CV_FONT_ITALIC, 1, 1, 0, 1);
    cv::namedWindow(WINDOW);
    msg_no = 0;
  }

  ~ImageAnalyzer()
  {
    cv::destroyWindow(WINDOW);
  }

  string getColorName(Color color)
  {
    if (color == RED)
      return "red";
    if (color == GREEN)
      return "green";
    if (color == BLUE)
      return "blue";
    return "unknown";
  }

  /**
   double getDistanceFromSize(int size)
   {
   return -0.2 * size + 39;
   }

   double getHorizontalDistance(int pixdiff, int depthdist)
   {
   double alpha = 320.0 / 22.5 * pixdiff;
   return tan(alpha / 180 * M_PI) * depthdist;

   }

   double getVerticalDistance(int pixdiff, int depthdist)
   {
   double alpha = 240.0 / 22.5 * pixdiff;
   return tan(alpha / 180 * M_PI) * depthdist;

   }

   double getAvgSize(std::vector<Cube> &cubes)
   {
   int sum = 0;
   for (std::vector<Cube>::iterator it = cubes.begin(); it != cubes.end(); it++)
   {
   int size = (*it).getSize();
   sum += size;
   }
   return sum / (double)cubes.size();
   }
   */

  void controllerCb(const imachines2::frames &msg)
  {
    frames_to_read = msg.frames;
    image_sub_ = it_.subscribe("/image_raw", 1, &ImageAnalyzer::imageCb, this);

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    IplImage* img = new IplImage(cv_ptr->image);

    if (detector == NULL)
      detector = new ColorBlobDetector(cvGetSize(img));

    detector->setImage(img);
    vector<Cube> cubes = detector->getCubes();
    cout << "Detected " << cubes.size() << " cubes." << endl;

    for (vector<Cube>::iterator it = cubes.begin(); it != cubes.end(); it++)
    {
      cout << (*it).toString() << endl;
      cvPutText(img, getColorName((*it).getColor()).c_str(), (*it).getCenter(), &font, CV_RGB(255,255,255));
      cvRectangle(img, (*it).getMinXy(), (*it).getMaxXy(), CV_RGB(255,255,255), 3, 8, 0);
    }

    cv::imshow(WINDOW, img);
    cv::waitKey(3);
    delete img;

    std::vector<imachines2::cube> msg_cubes;

    //Conversion to righthand-coordinates, 0-point in lens, x in view direction
    //Conversion into rosmessage
    for (vector<Cube>::iterator it = cubes.begin(); it != cubes.end(); it++)
    {

      imachines2::cube newCube;
      //avg. dist to Cam, assuming all cubes on same layer
      newCube.centerx = DEPTH_DIST / 100.0;

      int horDistCenter = (*it).getCenter().x - 320;
      double distY = abs(horDistCenter) / PIXEL_CM;
      //if hordistcenter<0, it's left from the pics center -> pos on the y-axis
      if (horDistCenter > 0)
        distY *= -1;
      newCube.centery = distY / 100.0;

      int verDistCenter = (*it).getCenter().y - 240;
      double distZ = abs(verDistCenter) / PIXEL_CM;

      if (verDistCenter > 0)
        distZ *= -1;
      newCube.centerz = distZ / 100.0;
      newCube.color = getColorName((*it).getColor());
      msg_cubes.push_back(newCube);
    }

    imachines2::cubelist cubelist_msg;
    cubelist_msg.header.frame_id = "cubelist";
    cubelist_msg.header.seq = msg_no++;
    cubelist_msg.header.stamp = ros::Time::now();
    cubelist_msg.cubes = msg_cubes;
    cout << "published!!" << endl;
    cubes_pub.publish(cubelist_msg);
    frames_to_read--;
    if (frames_to_read <= 0)
      image_sub_.shutdown();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_Analyzer");
  ImageAnalyzer ic;
  ros::spin();
  return 0;
}
