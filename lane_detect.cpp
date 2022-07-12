#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
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

char newImage;

using namespace cv;

int imageCount = 4; //which image to use for detection test

//MATH VARIABLES
int min_area = 30; //Connected Components minimum allowable area
int image_width = 640;
int image_height = 480;
int maskSize = 2; //Low pass mask size
int morph_size = 3; //small ellipse size
int morph_size2 = 5; //big ellipse size
int connectivity = 4; //4 or 8 connected for CC analysis

//FRAME ACQUISITION AND RESIZING VARIABLES
Mat frame;
Mat resized;
Mat cropped_image;

//GRAY SCALE AND WHITE HIGHLIGHT VARIABLES
Mat channel[3];
Mat red, green, blue;

//THRESHOLDING AND BINARIZATION VARIABLES
Mat gray_image;
Mat binary_img, binary_img1;
Mat lowPassImg;
Mat mean_img;

//DILATION AND EROSION VARIABLES
Mat dilation, dilation2, erosion, erosion2;
Mat dil, dil1, ero, ero1;

Mat smallEllipse = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size / 3 + 1, 2 * morph_size / 3 + 1), Point(morph_size / 3, morph_size / 3));
Mat bigEllipse = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size2 + 1, 2 * morph_size2 + 1), Point(morph_size2, morph_size2));
Mat smallCross = getStructuringElement(MORPH_CROSS, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
Mat bigCross = getStructuringElement(MORPH_CROSS, Size(2 * morph_size2 + 1, 2 * morph_size2 + 1), Point(morph_size2, morph_size2));
Mat smallRect = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));

//CONNECTED COMPONENET VARIABLES
Mat stats, labels, centroids, surfSup;

//HISTOGRAM EQUALIZATION VARIABLES
//Mat hist,dst;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_image_processor");
  ros::NodeHandle nh;

  //namedWindow("myFrame");
  //namedWindow("myFrame1");
  //namedWindow("myFrame2");

  std::string imCount = std::to_string(imageCount);

  //std::string image_path = "/home/tyler/catkin_ws/src/laneDetection/include/laneDetection/road/road"+imCount+".png";
  std::string image_path = "/home/tyler/catkin_ws/src/laneDetection/src/Color_Set/Img23.png";
  frame = imread(image_path);
  if (frame.empty())
  {
    ROS_INFO_STREAM("NO IMAGE");
  }

  else 
  {
    imshow("myFrame",frame);
    newImage = true;
  }

  // resize the image to 640 by 480
  resize(frame, resized, Size(image_width, image_height), INTER_LINEAR);
  cropped_image = resized(Range(140, 480), Range(0, 640));

  //imshow("myFrame",resized);
  imshow("myFrame",cropped_image);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    if (newImage)
    {
      if (frame.empty())
      {
        ROS_INFO_STREAM("Empty Frame Message");
        //********************
        // not sure why this frame message is empty but the callback msg isnt and i've set it equal to the global variable
      }
      else
      {

        split(cropped_image, channel);

        blue = channel[0]; // we determine white lines to be more visible in the BLUE plane
        green = channel[1];
        red = channel[2];

        gray_image = (2 * blue) - red; // weight the blue spectrum more and extract the red which contains most noise
        //gray_image.convertTo(mean_img, CV_32F);

        //imshow("mean img",mean_img);

        //IMPLEMENT LOW PASS FILTERING FOR ILLUMINATION CORRECTION
        lowPassImg = gray_image.clone();

        ROS_INFO_STREAM("Image Size: " << gray_image.size());
        
        Scalar intensity1 = 0; //intensity initialization
        for (int i=0; i<gray_image.rows-maskSize; i++)
        {
          for(int j=0; j<gray_image.cols-maskSize; j++)
          {
            Scalar intensity2;
            for (int p=0; p<maskSize; p++)
            {
              for (int q=0; q<maskSize; q++)
              {
                //Exectute median filter process on pixels
                intensity1 = gray_image.at<uchar>(i+p,j+q);
                intensity2.val[0]+=intensity1.val[0];
              }
            }
            lowPassImg.at<uchar>(i+(maskSize-1)/2,j+(maskSize-1)/2)=intensity2.val[0]/(maskSize*maskSize);
          }
        }

        //imshow("myFrame",gray_image);
        imshow("Low Pass Img",lowPassImg);
        imshow("Gray Image", gray_image);

        // HERE I WILL TRY HISTOGRAM EQUALIZATION 
        //cvtColor(lowPassImg,hist,COLOR_BGR2GRAY);

        //equalizeHist(lowPassImg,dst);

        //imshow("Low Pass Image",lowPassImg);
        //imshow("Equalized Image",dst);

        // BINARIZE THE IMAGE
        adaptiveThreshold(gray_image, binary_img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 3, 15);
        adaptiveThreshold(lowPassImg, binary_img1, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 3, 5);
        // adaptiveThreshold(gray_image,binary_img1,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,3,15);

        //imshow("myFrame3",binary_img1);
        //imshow("gray threshed",binary_img);

        //Execution 1 = no low pass filter
        //Execution 2 = low pass filter
        dilate(binary_img, dilation, smallCross, Point(-1, -1), 1); // kernel, iterations 
        dilate(binary_img1, dil, smallCross, Point(-1, -1), 1);

        erode(dilation, erosion, smallEllipse, Point(-1, -1), 1);
        erode(dil, ero, smallEllipse, Point(-1, -1), 1);

        dilate(erosion, dilation2, bigCross, Point(-1, -1), 1);
        dilate(ero, dil1, bigCross, Point(-1, -1), 1);

        erode(dilation2, erosion2, smallRect, Point(-1, -1), 1);
        erode(dil1, ero1, smallRect, Point(-1, -1), 1);

        // Write outputs to compare efficacy
        // imwrite("dilation1.png",dilation);
        // imwrite("erosion1.png",erosion);
        // imwrite("dilation2.png",dilation2);
        // imwrite("erosion2.png",erosion2);

        imshow("myFrame2", erosion2);
        imshow("Pre - CC", ero1);

        // WE WANT TO MOVE FORWARD W/ ERO2 OR SOMETHING SIMILAR

        // NOW THAT WE HAVE DONE SOME BINARIZATION AND ADAPTIVE THRESHOLDING 
        // WE WILL DO CC ANALYSIS

        //int nLabels = connectedComponentsWithStats(erosion2, labels, stats, centroids, connectivity, CV_32S); NO LOW PASS OPTION
        int nLabels = connectedComponentsWithStats(ero1, labels, stats, centroids, connectivity, CV_32S);

        surfSup = stats.col(4) > min_area;

        ROS_INFO_STREAM("SurfSup: " << surfSup);

        // ROS_INFO_STREAM("Show statistics and Centroids: " <<std::endl << "Stats: " <<std::endl << "(Left, Top, Width, Height, Area" << std::endl << stats <<std::endl <<std::endl);
        // ROS_INFO_STREAM("Centroids: " <<std::endl <<"(x,y)" <<std::endl << centroids <<std::endl <<std::endl);

        // IMROVED METHOD for CC Area Removal

        //Mat r(erosion.size(), CV_8UC1, Scalar(0));
        Mat r(ero1.size(), CV_8UC1, Scalar(0));
        Mat mask(labels.size(), CV_8UC1, Scalar(0));
        int tmp_label;

        for (int i = 1; i < labels.rows; i++)
        {
          for (int j = 1; j < labels.cols; j++)
          {

            tmp_label = labels.at<int>(i, j);

            mask.at<char>(i, j) = (char)surfSup.at<char>(tmp_label, 0);
          }
        }

        //erosion.copyTo(r, mask); NO LOW PASS OPTION
        ero1.copyTo(r, mask);


        Mat inverted_binary_mask;
        bitwise_not(mask, inverted_binary_mask);

        //imshow("myFrame1",mask);

        Mat outImage; //, outImage2, outImage3, outImage4, outImage5;
        //outImage = erosion2 - inverted_binary_mask; NO LOW PASS OPTION
        outImage = ero1 - inverted_binary_mask;

        //HERE WE WILL ATTEMPT TO DILATE AND ERODE TO GET LANE LINES LEFT
        //dilate(outImage,outImage2,bigEllipse,Point(-1,-1),1);
        //dilate(outImage2,outImage3,smallRect,Point(-1,-1),1);

        //erode(outImage3,outImage4,smallRect,Point(-1,-1),1);

        //imshow("myFrame", outImage2);
        //imshow("myFrame1", outImage3);
        //imshow("myFrame1", outImage4);

        imshow("myFrame2",outImage);
        //imwrite("src/laneDetection/src/Low_Pass_Outputs/Low_Pass_Output_img4.png",outImage);

        if (nLabels == 0)
        {
          ROS_INFO_STREAM("No Labels Found");
        }

        else
        {
          // ROS_INFO_STREAM("nLabels: " <<nLabels);
          // ROS_INFO_STREAM("Labels: " <<labels);
          ROS_INFO_STREAM("stats.size()=" << stats.size());
        }
      }
      newImage = false;
    }
    
    waitKey(3);
    ros::spinOnce();
  }
  return 0;
}