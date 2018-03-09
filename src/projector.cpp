#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <stdlib.h>     /* atoi */
#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>

// Namespace
using namespace std;
using namespace rs::core;
using namespace rs::utils;
using namespace cv;
using namespace Eigen;

const double s_factor = 1000;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

/* The Projector class is to receive RGB image stream and Depth image. 
Subscribing to another node which is written in Python for the 
text detection and recognition. 
*/
class Projector
{
private:
	Mat image; 
	Mat d_image;
	vector<vector<int>> bboxes; // [[x1,y1,x2,y2,x3,y3,x4,y4,confidence]...]
	vector<std::string> text_rec; // [['String1']['String2']...]
	MatrixXd Proj_mat(3,4); // Project matrix 
	pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB; 

public:
void rgb_callback(const sensor_msgs::ImageConstPtr& msg);
void depth_callback(const sensor_msgs::ImageConstPtr& msg);
void get_projection_mat(const sensor_msgs::CameraInfoConstPtr& msg);
void text_det_callback(const my_pkg::VectorStringPtr& msg);
void pixel_to_camera_coordinate();
void compute_normals();

};

void Projector::rgb_callback(const sensor_msgs::ImageConstPtr& msg){
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	image = cv_ptr->image.clone();
}

void Projector::depth_callback(const sensor_msgs::ImageConstPtr& msg){
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	d_image = cv_ptr->image.clone();
}


void get_projection_mat(const sensor_msgs::CameraInfoConstPtr& msg){
	Proj_mat = msg->P;
}

//ToDo
void Projector::text_det_callback(const my_pkg::VectorStringPtr& msg){
	// msg is a vector of string, containing all the text detected region of one image, 
	// each string contains the bounding box coordinate, followed by the text recognized. 
	vector<string> text_det;
	text_det.clear();
	text_det = msg->vec;

	for(int i = 0; i < text_det.size(); i++){
		stringstream ss(text_det[i]);
		string substr;
		for(int j = 0 ; j < 9; j++){
			string substr;
			getline(ss, substr, ','); // Skip ',' only
			bboxes[i][j] = atoi(substr.c_str()); // Convert string to integer
		}
		getline(ss, substr, ','); // Get the text recognition result
		text_rec.push_back(substr); // Push to the string vector
	}

}


void Projector::pixel_to_camera_coordinate(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 
	num_of_boxes = bboxes.size();
	uint8_t r, g, b;
	Vec3b intensity;
	float factor = 1; // Need to verify
	// Loop over all the bounding boxes
	for (int i = 0; i < num_of_boxes; i++){
		for (int u_i = bboxes[i][0]; u_i < bboxes[i][4]; u_i++){
			for (int v_i = bboxes[i][1]; v_i < bboxes[i][5]; v_i++)
			{
				intensity = image.at<Vec3b>(v_i, u_i);
				b = intensity.val[0];
				g = intensity.val[1];
				r = intensity.val[2];
				// Follow the example on http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
				pcl::PointXYZRGB point;
				point.x = (u_i - Proj_mat(0,3))/Proj_mat(0,0)*d_image(u_i,v_i);
				point.y = (v_i - Proj_mat(0,3))/Proj_mat(0,0)*d_image(u_i,v_i);
				point.z = d_image(u_i,v_i)*s_factor;
				uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              			static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				point.rgb = *reinterpret_cast<float*>(&rgb);
				point_cloud_ptr->points.push_back (point);
			}
		}
	}
	// RGB viewer
	viewer = rgbVis(point_cloud_ptr);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	// Copyt the XYZRGB point cloud
	CloudXYZRGB = *point_cloud_ptr;
}

void Projector::compute_normals(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(CloudXYZRGB));
	// Create the normal estimation object, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB>, pcl::Normal> normal_est;
	normal_est.setInputCloud(point_cloud_ptr);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
  	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  	normal_est.setSearchMethod (tree);
  	
  	// Output datasets
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
  	normal_est.setRadiusSearch (0.03);

  	// Compute the features
  	normal_est.compute (*cloud_normals);
}