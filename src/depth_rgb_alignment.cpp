
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace rs::core;
using namespace rs::utils;
using namespace cv;


int main(int argc, char **argv)
{
ros::init(argc, argc, "projection");
ros::NodeHandle nodeHandler;
ros::Rate loop_rate(10);

Projector projector;
ros::Subscriber sub_rgb = nodeHandler.subscribe("/camera/rgb/image_rect_color", 10, &Projector::rgb_callback, &projector)
ros::Subscriber sub_depth = nodeHandler.subscribe("/camera/depth_registered/sw_registered/image_rect_raw", 10, &Projector::depth_callback, &projector);
ros::Subscriber sub_depth = nodeHandler.subscribe("/text_detect", 10, &Projector::text_det_callback, &projector);


ros::spin();
return 0;

}