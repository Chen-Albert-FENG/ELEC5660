#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <string>

using namespace aruco;
using namespace std;

aruco::CameraParameters CamParam;

int main(int argc, char **argv)
{
    string cam_cal;
    cam_cal = "/home/albert/ELEC5660/proj2phase1/tag_ws/src/tag_detector/config/camera.yml";

    CamParam.readFromXMLFile(cam_cal);

}