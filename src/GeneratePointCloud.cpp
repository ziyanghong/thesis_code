// C++ library
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <stdlib.h> // atoi header file
using namespace std;

// OpenCV library
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Define point cloud data type
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

// Realsense ZR300 Intrinsic parameter
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

vector<vector<int>> GetTextDetection(const string &text_detection_path);

// Main 
int main( int argc, char** argv )
{
    string rgb_path = "/data/rgb.png";
    string depth_path = "/data/depth.png";
    string text_detection_txt = "/frame.txt";

    cv::Mat rgb, depth;
    rgb = cv::imread( rgb_s );
    // Read the raw data with -1.
    depth = cv::imread( depth_s, -1 );

    // Point cloud
    // Use smart pointer here, initialize an empty point cloud
    PointCloud::Ptr cloud ( new PointCloud );
    // Tranverse the depth image
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // Take the depth value at pixel location (m,n)
            ushort d = depth.ptr<ushort>(m)[n];
            // If zero, skip it.
            if (d == 0)
                continue;
            PointT p;

            // Compute the 3d coordinate.
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            
            // BGR channel
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // Add point p to the point cloud.
            cloud->points.push_back( p );
        }
    // Save the point cloud.
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "./pointcloud.pcd", *cloud );
    // Delete it and exit.
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}

vector<vector<int> > GetTextDetection(const string &text_detection_txt)
{
    vector<vector<int> > bboxes;
    ifstream text_dtection;
    cout<<"text_detection_txt.c_str() is "<<text_detection_txt.c_str()<<endl;
    text_dtection.open(text_detection_txt.c_str());
    string s;
    while(getline(text_dtection,s)){
        vector<int> box;
        stringstream ss;
        ss<<s;
        for(int j = 0; j < 8; j++)
        {
            string substr;
            getline(ss, substr, ',');
            box.push_back(atoi(substr.c_str()));
            cout<<box[j]<<",";
        }
        cout<<endl;
        bboxes.push_back(box);
        box.erase(box.begin(), box.end()); // Make sure clear the vector every iteration.
    }
    return bboxes;
}