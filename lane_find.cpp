#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

using namespace cv;

cv::Mat frame;
char newImage;

Mat channel[3];
Mat red; //(250,750,CV_8UC1,cv::Scalar(0));
Mat green; // (250,750,CV_8UC1,cv::Scalar(0));
Mat blue; // (250,750,CV_8UC1,cv::Scalar(0));

Mat gray_image, greyMat;
Mat mean_img, lowPassImg, lowPassImg1; // (250,750,CV_32F,cv::Scalar(0));
Mat binary_img, binary_img1, binary_img2;


int min_area = 8; //Connected Components minimum allowable area
int image_width = 640;
int image_height = 480;
int maskSize = 2; //Low pass mask size
int morph_size = 3; //small ellipse size
int morph_size2 = 5; //big ellipse size
int connectivity = 4; //4 or 8 connected for CC analysis

//DILATION AND EROSION VARIABLES
Mat dilation, dilation2, erosion, erosion2;
Mat dil, dil1, dil2, ero, ero1, ero2, ero3, ero4;

Mat smallEllipse = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size / 3 + 1, 2 * morph_size / 3 + 1), Point(morph_size / 3, morph_size / 3));
Mat bigEllipse = getStructuringElement(MORPH_ELLIPSE, Size(2 * morph_size2 + 1, 2 * morph_size2 + 1), Point(morph_size2, morph_size2));
Mat smallCross = getStructuringElement(MORPH_CROSS, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
Mat bigCross = getStructuringElement(MORPH_CROSS, Size(2 * morph_size2 + 1, 2 * morph_size2 + 1), Point(morph_size2, morph_size2));
Mat smallRect = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));

//CONNECTED COMPONENET VARIABLES
Mat stats, labels, centroids, surfSup;
Mat stats1, labels1, centroids1, surfSup1;

Mat blur_img;
Mat blurred_img;
Mat sigma;

ros::Time begin;
ros::Time end;

int totalImages = 0;
int imgcount = 0;


void camera_callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    frame = cv_ptr->image;
    
    if(frame.empty())
    {
      ROS_INFO_STREAM("Empty Frame Callback Message");
    }
    //else
    //cv::imshow("myFrame1",frame2);
    //waitKey(30);
    
    newImage=true;

    //return;

    //HERE FRAME NOW HOLDS THE MOST RECENT IMAGE 
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    return;
  }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ros_image_processor");
    ros::NodeHandle nh_;

    newImage=false;

    //ROS_INFO_STREAM("Frame created");

    image_transport::ImageTransport it(nh_);

    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw",1,camera_callback);
    
    image_transport::Publisher pub = it.advertise("/camera/image",1);

    //image_transport::Subscriber sub = it.subscribe("camera/rgb/image_color",1,camera_callback);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
      if (newImage)
      {
        if (frame.empty())
        {
          ROS_INFO_STREAM("Empty Frame Message");
          //********************
        }
        else
        {
          //NEED TO PUT COLOR PRE-PROCESSING HERE IN ORDER TO REMOVE WHITE LANE
          // ******************************************************************
          begin = ros::Time::now(); //start frame timer

          split(frame,channel);
          blue = channel[0]; //we determine white lines to be more visible in the BLUE plane
          green = channel[1];
          red = channel[2];

          gray_image = (2 * blue) - red;   //weight the blue spectrum more and extract the red which contains most noise

          /*
          totalImages = totalImages + 1;
          ROS_INFO_STREAM("TOTAL: " << totalImages);
          ROS_INFO_STREAM("Total Incremented");
          if (totalImages == 20)
          {
            totalImages = 0;
            //imwrite("src/laneDetection/src/NewRoadImage" + imageCount + ".png", gray_image);

            std::string iteration = "/home/tyler/catkin_ws/src/laneDetection/src/Road_Test_Imgs/Img" + std::to_string(imgcount);
            std::string colorPath = "/home/tyler/catkin_ws/src/laneDetection/src/Color_Set/Img" + std::to_string(imgcount);
            std::string grayPath = "/home/tyler/catkin_ws/src/laneDetection/src/Gray_Set/Img" + std::to_string(imgcount);
            std::string pathFinal = iteration+".png";
            std::string grayP = grayPath + ".png";
            std::string colorP = colorPath + ".png";

            //ROS_INFO_STREAM("Path: " <<pathFinal);
            imwrite(pathFinal,gray_image);
            imwrite(grayP,gray_image);
            imwrite(colorP,frame);
            //imwrite("New_image.png",gray_image);
            ROS_INFO_STREAM("Image Captured");
            imgcount = imgcount + 1;
          }
          */

          cvtColor(frame,greyMat,COLOR_BGR2GRAY);

          lowPassImg = greyMat.clone();
          lowPassImg1 = gray_image.clone();

        //ROS_INFO_STREAM("Image Size: " << gray_image.size()); //confirm image data not empty

        //EXECUTE LOW PASS FILTER ON BOTH BLUE-RED AND CVGRAY VERSIONS
        
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

        Scalar intensity3 = 0; //intensity initialization
        for (int i=0; i<greyMat.rows-maskSize; i++)
        {
          for(int j=0; j<greyMat.cols-maskSize; j++)
          {
            Scalar intensity4;
            for (int p=0; p<maskSize; p++)
            {
              for (int q=0; q<maskSize; q++)
              {
                //Exectute median filter process on pixels
                intensity3 = greyMat.at<uchar>(i+p,j+q);
                intensity4.val[0]+=intensity3.val[0];
              }
            }
            lowPassImg1.at<uchar>(i+(maskSize-1)/2,j+(maskSize-1)/2)=intensity4.val[0]/(maskSize*maskSize);
          }
        }

        //imshow("Gray Image",gray_image);
        //imshow("Low Pass Image",lowPassImg);
        //imshow("CV Low Pass Image",lowPassImg1);

        // BINARIZE THE IMAGE
        adaptiveThreshold(gray_image, binary_img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 3, 15);

        adaptiveThreshold(lowPassImg, binary_img1, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 3, 4);

        adaptiveThreshold(lowPassImg1, binary_img2, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 3, 5);

        // adaptiveThreshold(gray_image,binary_img1,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,3,15); //secondary thresh method

        //imshow("Binary Image 1",binary_img1);
        //imshow("Binary Image 2",binary_img2);

        //Execution 1 = no low pass filter
        //Execution 2 = low pass filter
        //dilate(binary_img, dilation, smallCross, Point(-1, -1), 1); // kernel, iterations 
        dilate(binary_img1, dil, smallCross, Point(-1, -1), 1); //wih blue - red weighted & low pass
        dilate(binary_img2, dil1, smallCross, Point(-1, -1), 1); //with openCV grayscale

        //erode(dilation, erosion, smallEllipse, Point(-1, -1), 1); //without lowPass filter
        erode(dil, ero, smallEllipse, Point(-1, -1), 1); //with blue - red weighted grayscale & low pass
        erode(dil1, ero3, smallEllipse, Point(-1, -1), 1); //with openCV grayscale

        //dilate(erosion, dilation2, bigCross, Point(-1, -1), 1); //without lowPass filter
        dilate(ero, dil1, bigCross, Point(-1, -1), 1); //with blue - red weighted grayscale & low pass
        dilate(ero3, dil2, bigCross, Point(-1, -1), 1); //with openCV grayscale

        //erode(dilation2, erosion2, smallRect, Point(-1, -1), 1); //without lowPas filter
        erode(dil1, ero1, smallRect, Point(-1, -1), 1); //with blue - red gray scale & low pass
        erode(dil2, ero4, smallRect, Point(-1, -1), 1); //with openCV grayscale

        imshow("Erosion w/ my gray",ero1);
        imshow("Erosion w/ CV gray",ero4);

        int nLabels = connectedComponentsWithStats(ero1, labels, stats, centroids, connectivity, CV_32S);
        int nLabels1 = connectedComponentsWithStats(ero4, labels1, stats1, centroids1, connectivity, CV_32S);

        surfSup = stats.col(4) > min_area;
        surfSup1 = stats1.col(4) > min_area;

        //ROS_INFO_STREAM("SurfSup: " << surfSup);
        //ROS_INFO_STREAM("SurfSup1: " <<surfSup1);

        // ROS_INFO_STREAM("Show statistics and Centroids: " <<std::endl << "Stats: " <<std::endl << "(Left, Top, Width, Height, Area" << std::endl << stats <<std::endl <<std::endl);
        // ROS_INFO_STREAM("Centroids: " <<std::endl <<"(x,y)" <<std::endl << centroids <<std::endl <<std::endl);

        // IMROVED METHOD for CC Area Removal
        // CC REMOVAL FOR BLUE-RED
        //Mat r(erosion.size(), CV_8UC1, Scalar(0)); //method before low pass filter
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

        // CC REMOVAL FOR BGR2GRAY
        Mat e(ero4.size(), CV_8UC1, Scalar(0));
        Mat mask1(labels1.size(), CV_8UC1, Scalar(0));
        int tmp_label1;

        for (int i = 1; i < labels1.rows; i++)
        {
          for (int j = 1; j < labels1.cols; j++)
          {

            tmp_label1 = labels1.at<int>(i, j);

            mask1.at<char>(i, j) = (char)surfSup1.at<char>(tmp_label1, 0);
          }
        }

        //Produce Masked output to remove components
        //erosion.copyTo(r, mask); NO LOW PASS OPTION
        ero1.copyTo(r, mask);
        ero4.copyTo(e, mask1);

        Mat inverted_binary_mask, inverted_binary_mask1;
        bitwise_not(mask, inverted_binary_mask);
        bitwise_not(mask1, inverted_binary_mask1);

        //imshow("myFrame1",mask);

        Mat outImage, outImage1, outImage2, outImage3, outImage4;// outImage5;
        //outImage = erosion2 - inverted_binary_mask; NO LOW PASS OPTION
        outImage = ero1 - inverted_binary_mask;
        outImage1 = ero4 - inverted_binary_mask;

        outImage2 = ero1 - inverted_binary_mask1;
        outImage3 = ero4 - inverted_binary_mask1;

        //outImage2 = ero1 - mask;
        //outImage3 = inverted_binary_mask;
        //outImage4 = mask;

        //imshow("Sub Inverted Mask",outImage);
        imshow("Sub Inverted Mask CV",outImage1);
        //imshow("Sub Inverted Mask1",outImage2);
        imshow("Sub Inverted Mask1 CV",outImage3);
        imshow("Natural Image",frame);
        //imshow("Sub Mask",outImage2);
        //imshow("Inverted Mask",outImage3);
        //imshow("Mask",outImage4);

        end = ros::Time::now();
        int start = begin.toSec();
        int finish = end.toSec();

        int duration = finish - start;
        ros::Duration Duration = end - begin;
        //ROS_INFO_STREAM("Time Duration INT: " << duration << " s");
        //ROS_INFO_STREAM("Time Duration DURATION NSEC: " << Duration.toNSec() << " s");
        ROS_INFO_STREAM("Time Duration: " << Duration.toSec() << "s");
        }

        newImage=false;
      }

      waitKey(3);
      ros::spinOnce();
 
      //ros::Duration duration = begin - end;
    }

return 0;
}