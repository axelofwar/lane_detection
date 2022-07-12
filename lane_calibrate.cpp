#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "popt_pp.h"
#include <sys/stat.h>

char newImage;

using namespace cv;
using namespace std;

//FRAME ACQUISITION AND RESIZING VARIABLES
Mat resized;
Mat calImage;
Mat calibImage;

Mat img, gray;
Size im_size;

int image_width = 640;
int image_height = 480;

bool imageShow = true;

vector< vector< Point3f> > objpoints;  
vector< vector< Point2f> > imgpoints;
vector<Point3f> objp;
vector<Point2f> corner_pts;
vector<String> images;

//namedWindow("myFrame");


bool doesExist (const std::string& name) 
{
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0); 
}

void setup_calibration(int board_width, int board_height, int num_imgs, float square_size, char* imgs_directory, char* imgs_filename, char* extension) 
{
  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;
  int A = 0;

//Iterate through lane calibration images & populate image glob
  for (int k = 1; k <= num_imgs; k++) {
    char img_file[100];
    
    imgs_directory = "/home/tyler/catkin_ws/src/laneDetection/src/ChessboardImages/";
    if(k<(num_imgs/2+1))
    {imgs_filename = "left";}
    else
    {
        imgs_filename = "right";
        A++;
    }

    extension = "jpg";

    if(imgs_filename == "left")
    sprintf(img_file, "%s%s%d.%s", imgs_directory, imgs_filename, k, extension);

    if(imgs_filename == "right")
    {
        sprintf(img_file, "%s%s%d.%s", imgs_directory, imgs_filename, A, extension);
    }

//FOR SOME REASON THE RIGHT IMAGES AREN'T POPULATING EVEN THO THE ADDRESS IS CORRECT AND REGULAR IMSHOW OF DIRECT READ WORKS

    ROS_INFO_STREAM("Image File: "<<img_file); //output file directory to confirm

    if(!doesExist(img_file))
      {continue;}
    img = imread(img_file);

    //imshow("Chessboard Image",img);
    cvtColor(img, gray, CV_BGR2GRAY);

    bool found = false;

    // Find chessboard corners for calibration 
    found = findChessboardCorners(img, board_size, corner_pts, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    if (found)
    {
      ROS_INFO_STREAM("Corners Found!");

      cornerSubPix(gray, corner_pts, cv::Size(5, 5), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      
      drawChessboardCorners(gray, board_size, corner_pts, found);
    }
    
    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    if (found) {
      ROS_INFO_STREAM("Image Num: "<<k << ". Found corners!");
      imgpoints.push_back(corner_pts);
      objpoints.push_back(obj);
    }
  }
}

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs) 
{
  vector< Point2f > imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  vector< float > perViewErrors;
  perViewErrors.resize(objectPoints.size());

  for (i = 0; i < (int)objectPoints.size(); ++i) {
    projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                  distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n; 
  }
  return std::sqrt(totalErr/totalPoints);
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_calibrator");
  ros::NodeHandle _nh;

  int calibCount = 1;

  string calCount = to_string(calibCount);
  
  int board_width, board_height, num_imgs;
  float square_size;
  char* imgs_directory;
  char* imgs_filename;
  char* out_file;
  char* extension;


  string path = "/home/tyler/catkin_ws/src/laneDetection/include/laneDetection/ChessboardImages/";

// Chessboard Intrinsics
board_width = 9;
board_height = 6;
glob(path, images);
num_imgs = images.size();
square_size = .02423;
out_file = "Calibration Statistics";

namedWindow("Orig Img");
namedWindow("Undistort Img");


  ros::Rate loop_rate(1);

  while (ros::ok())
  {
      waitKey(3);
      ros::spinOnce();

      setup_calibration(board_width, board_height, num_imgs, square_size, imgs_directory, imgs_filename, extension);

      ROS_INFO_STREAM("Starting Calibration");

      Mat K, D;
      vector<Mat> rvecs, tvecs;
      int flag = 0;
      flag |= CALIB_FIX_K4;
      flag |= CALIB_FIX_K5;

      if (objpoints.empty())
      {
          ROS_INFO_STREAM("ObjPoints empty!");
      }

      if (imgpoints.empty())
      {
          ROS_INFO_STREAM("ImgPoints empty!");
      }

      calibrateCamera(objpoints, imgpoints, img.size(), K, D, rvecs, tvecs, flag);

      cout << "Calibration error: " << computeReprojectionErrors(objpoints, imgpoints, rvecs, tvecs, K, D) << endl;

      //K = cameraMatrix - 
      //D = distCoeffs

      FileStorage fs(out_file, FileStorage::WRITE); // output the file instrinscs to a calibration file 
      fs << "K" << K;
      fs << "D" << D;
      fs << "board_width" << board_width;
      fs << "square_size" << square_size;
      ROS_INFO_STREAM("Done Calibration\n");

      Mat imageUndistorted;
  
      undistort(img,imageUndistorted,K,D);
      if(img.empty())
      {
        ROS_INFO_STREAM("Image Empty");
      }

      if(imageUndistorted.empty())
      {
        ROS_INFO_STREAM("Undistorted Empty");
      }

      ROS_INFO_STREAM("Undistorted Size"<<imageUndistorted.size());

      imshow("Orig Img",img);
      imshow("Undistort Img",imageUndistorted);

      /*
      Mat view,rview,map1,map2;

      if(img.empty())
      {
        ROS_INFO_STREAM("Image is Empty!");
      }

      Mat temp = imread(path+"left1.jpg");

      Mat newCamMat;

      undistort(temp,view,K,D,newCamMat);
      if (img.empty())
      {
        ROS_INFO_STREAM("Image is Empty!");
      }
      imshow("Original Img",img);
      ROS_INFO_STREAM("Original Img Shown");


      initUndistortRectifyMap(K,D,Mat(),getOptimalNewCameraMatrix(K,D,img.size(),1,img.size(),0),img.size(),CV_16SC2,map1,map2);
      if(map1.empty())
      {
        ROS_INFO_STREAM("Map 1 Empty");
      }

      if(map2.empty())
      {
        ROS_INFO_STREAM("Map 2 Empty");
      }

      remap(view,rview,map1,map2,CV_INTER_LINEAR);

      if(rview.empty())
      {
        ROS_INFO_STREAM("Rview Empty");
      }

      imshow("Image View",rview);
      ROS_INFO_STREAM("Undistorted Img Shown");
      */
      pause();
      imageShow = false;
      return 0;
  }
  return 0;
}