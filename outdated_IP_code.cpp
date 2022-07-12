/*===========================================================================/
			 COMPETITION CODE!!

			 color_cloud.cpp

			 Written by: Philip Renn
			 Last Edited by Phil: May 27, 2017
			 Edited by Ted Chase (for ground plane geometry? 2017-18 term II)
			 
/===========================================================================*/



/*===========================================================================/
			INCLUDES
/===========================================================================*/

// Plugins, Nodelets, ROS, Standard I/O, String
//-------------------------------------------
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>  //standard input output - used for subscribe/publishing
#include <string>
//-------------------------------------------


// Standard Messages
//-------------------------------------------
#include <std_msgs/String.h>  //include if topic the is being subscribed is a string
#include <std_msgs/Float64.h>  // include file for other format of std_msg
#include <std_msgs/UInt8.h>
#include <std_msgs/Header.h>
//-------------------------------------------


// Sensor Messages, PointCloud, Image, IMU
//-------------------------------------------
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
//-------------------------------------------


// Math
//-------------------------------------------
#include <math.h>
#include <cmath>
#include <vector>
//-------------------------------------------


// Point Cloud Library (PCL)
//-------------------------------------------
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//-------------------------------------------


// OpenCV & Edge Detection
//-------------------------------------------
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
//#include "linefinder.h"
#include "opencv-2-cookbook-src-master/Chapter 07/linefinder.h"
#include <iostream>
//-------------------------------------------


/*===========================================================================/
			GLOBAL VARIABLES
/===========================================================================*/

// Declare NaN values for "don't care" pixels
//-------------------------------------------
const float NaN = std::numeric_limits<float>::quiet_NaN();  // elegant way to set a constant NaN variable
//const float NaN = sqrt(-1); // This way will work but the method above is more elegant
//-------------------------------------------


//===========================POINT CLOUD VARIABLES===========================//
sensor_msgs::PointCloud2* cloud_in (new sensor_msgs::PointCloud2);  // full point cloud input
//---------------------------------------------------------------------------//


//=============CHECKERBOARD FILTERED DOWNSAMPLED POINT CLOUD=================//
// Variable for the downsampled cloud using "iteration" method
sensor_msgs::PointCloud2Ptr it_cloud = boost::make_shared<sensor_msgs::PointCloud2>();
//---------------------------------------------------------------------------//


//===========================GROUND PLANE VARIABLES==========================//
// Declare x,y,z values to store float values for the coordinates of the pointcloud
float x; float y; float z;


//===========================IMAGE VARIABLES=================================//
sensor_msgs::Image *in_image (new sensor_msgs::Image);
sensor_msgs::Image *ground_image (new sensor_msgs::Image);  // image of the ground plane
sensor_msgs::Image *metric_image (new sensor_msgs::Image);  // ground image w/ metric adjustment
cv_bridge::CvImage out_img;
//---------------------------------------------------------------------------//


//=======================TIME DURATION VARIABLES=============================//
// Callback Function
ros::Time begin_callback;   // beginning time of callback
ros::Duration callback_duration; // duration of callback

// Ground Plane Extraction
ros::Time begin_ground;   // beginning time of ground plane extraction
ros::Duration ground_duration; // duration of ground plane extraction
ros::Time begin_lane;   // beginning time of lane detection
ros::Duration lane_duration; // duration of lane detection
//---------------------------------------------------------------------------//


//==============================FLAG VARIABLE================================//
bool newData;  // flag to identify new data
//---------------------------------------------------------------------------//


//==============================OPEN_CV VARIABLES================================//
// Image Processing Matrix Variables
cv::Mat bgr (246,750,CV_8UC1,cv::Scalar(0,0,255));
cv::Mat out1 (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat temp (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat gray_out (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat canny_out (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat gray_out1 (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat gray_outb (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat channel[3];

// Define R,G,B matrix variables of the same size as input image to store each layer independently
cv::Mat red (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat green (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat blue (246,750,CV_8UC1,cv::Scalar(0));
// Matrix variable to store Layer Manipulated images
cv::Mat blue_green (246,750,CV_8UC1,cv::Scalar(0));
cv::Mat blue_red (246,750,CV_8UC1,cv::Scalar(0));
// Output Image to lane_detect topic
cv::Mat bgr_out (246,750,CV_8UC1,cv::Scalar(0));

// "Actual" color layers displayed as true color (row x height x 3)
// cv::Mat red3 cv::Scalar(0,0,255);
// cv::Mat green3 cv::Scalar(0,0,255);
// cv::Mat blue3 cv::Scalar(0,0,255);

// Variable to store hough lines transformations
cv::Mat result(246,750,CV_8U,cv::Scalar(0));

// Low Pass Variables
cv::Mat mean_img (246,750,CV_32F,cv::Scalar(0));
cv::Mat mu (246,750,CV_32F,cv::Scalar(0));
cv::Mat mu2 (246,750,CV_32F,cv::Scalar(0));
cv::Mat sigma (246,750,CV_32F,cv::Scalar(0));
cv::Mat mean_thresh (246,750,CV_32F,cv::Scalar(0));
int alpha = 2;
int beta = 3;

// int thresh_mem = 220;  // Variable to store the previous threshold value

// Matricies for storing the difference between intensities of color layers
cv::Mat gb_diff (246,750,CV_8UC1,cv::Scalar(0));; // = green != blue;
cv::Mat gr_diff (246,750,CV_8UC1,cv::Scalar(0));; // = green != red;
cv::Mat gray_red_diff (246,750,CV_8UC1,cv::Scalar(0));; // Difference between grayscale image and red layer

//----------------------END OPEN_CV VARIABLES--------------------------------------------//


//===============INTITIALIZE VARIABLBES FOR COMPARISION MATRIX ==============//
// NOTE: Use with 1024x544 resolution image -- if resolution changes, adjust values respectively

sensor_msgs::LaserScan scan;
sensor_msgs::LaserScan flan;
float shit= 10;
float camera_height = 1.25;  // height of the camera from the ground
float row_start_angle = 0.21817;// 6615;  // angle of the top row -- 12.5 degrees -> radians
float inc_angle_row = 0.00144;// 37466;  // calculated angle increment value per row -- 45(vert FOV)/544(rows) 0.0827 degrees -> radians
float delta = 0;
float height_change = 0;
float offset = .14;
float hyp[544]; // Hypotenuse ('Z'/depth) values
float z_percent = .8;//.955;
//--------------END COMPARISON MATRIX VARIABLES-------------------------------//


/*===========================================================================/

			PROGRAM BEGIN

/===========================================================================*/

namespace color_cloud_nodelet
{
	  class point_cloud : public nodelet::Nodelet  //created nodelet class called "point_cloud" which inherits the standard base class
	  {  //start point_cloud class

		  virtual void onInit()  //initializing nodelet
		  {

				// *************************CONSTRUCT COMPARISON MATRIX**************************/
				// Create initial threshold values for comparison matrix
				// float row_start_angle = 0.21817;// 6615;  // 12.5 degrees -> radians
				// float inc_angle_row = 0.00144;// 37466;  // 45(vert FOV)/544(rows) 0.0827 degrees -> radians

				for (int i = 0; i < 544; i++)
				{
					  hyp[i] = camera_height/sin(row_start_angle + (inc_angle_row * i));// - offset;
				}

				//******************************************************************************/

				ros::NodeHandle nh;

				// SUBSCRIBERS
				sub_imu = nh.subscribe("/sparton/imu/data", 10, &point_cloud::sparton_imu_callback, this);
				sub_ground = nh.subscribe("/multisense/organized_image_points2_color", 10, &point_cloud::ground_plane_cb, this);
				// PUBLISHERS
				//pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 30);	// Full Cloud
				pub_ground_img = nh.advertise<sensor_msgs::Image>("ground_image", 10);	// Ground Plane Image
				pub_metric_img = nh.advertise<sensor_msgs::Image>("metric_image", 10);	// Metric Image
				pub_lane_lines = nh.advertise<sensor_msgs::Image>("lane_detect", 10);	// Black & White Image
				pub_laser = nh.advertise<sensor_msgs::LaserScan>("cam_scan", 10); 	// Black & White Image


				sub_metric = nh.subscribe("metric_image", 10, &point_cloud::lane_detect_callback, this);

				// Set up Publish Duration Variables, to measure time it takes to publish message
				ros::Time begin_pub;   // Beginning time of publishing
				ros::Duration pub_duration; // Amount of time it takes to publish

				//********************************[BEGIN]***************************************/
				ros::Rate r(100); // 100 hz
				while (ros::ok())
				{
					begin_pub = ros::Time::now(); // set begin_pub = now

				// If there is new point cloud data, publish messages
					if(newData)
					{
						//pub_cloud.publish(*cloud_in);  // Full image
						//pub_it_im.publish(*in_image);  // Input image
						pub_ground_img.publish(*ground_image); // Ground plane image
						pub_metric_img.publish(*metric_image);  // Metric Ground plane image
						pub_lane_lines.publish(out_img.toImageMsg());  // Metric Ground plane image
						pub_laser.publish(scan);
						newData = false;
						pub_duration = ros::Time::now() - begin_pub; // calculate total time to publish message

						ROS_INFO_STREAM(" \n\t----------------Timing Table------------------- \n"
						<< "\t Callback Function:\t\t" << callback_duration << "s"
						<< "\n\t Publish Duration:\t\t" << pub_duration << "s"
						<< "\n\t Ground Plane Extraction:\t" << ground_duration << "s"
						<< "\n\t Lane Detection:\t\t" << lane_duration << "s\n");
					}
					else
					{
						ROS_INFO_STREAM("\n\t SWaiting for Incoming Data...");
						//ROS_INFO_STREAM("\n\t Is Jaus bullshit?"  << jaus);
					}

					pub_duration = ros::Time::now() - begin_pub; // calculate total time to publish message
					ros::spinOnce();
					r.sleep();

				} // END While loop
				//*********************************[END]****************************************/

		  } // END virtual void onint function
	  	//-------------------------end oninit; return to class color_cloud definition-------------------------------


	  	//******************************************************************************
	  	//---------------------Sparton IMU Data callback function------------------------
	  	//******************************************************************************
	  	/*
			This callback recieves the data comming from the Sparton IMU. This allows us to adjust the 
			camera angle and height. Very beneficial in balance mode.  
	  	*/
	  	void sparton_imu_callback(const sensor_msgs::Imu::ConstPtr& sparton_imu)
	  	{
	  		delta = sparton_imu->orientation.y;  // radians of robot tilt
	  		double arc_length = delta * camera_height;  // calculate arc length of robot tilt
	  		height_change = camera_height - (camera_height * cos(delta));
	  	}
	  	//------------------------------------------------------------------------------

	  	//******************************************************************************
	  	//---------------Ground Plane extraction callback function----------------------
	  	//******************************************************************************
	  	/*
			This function receives a point cloud from the camera. Then, the image from the point cloud data is extracted.
			An image of the ground plane is created by only retaining the pixels whose z-values are above the expected (hypotenuse)
			value. Then, a metrically scaled image is constructed. Each pixel will correspond to a distance value of 9mm. This is 
			acheived by diving the x and y coordinates of each pixel by 0.009. The reslut of this operation will be a column and row
			coordinate, respectfully.
	  	*/
	  	void ground_plane_cb(const sensor_msgs::PointCloud2::ConstPtr& input)
	  	{
			// Set up Callback Duration Variables for callback function processing time
			begin_callback = ros::Time::now(); // set begin_convert = now

			// Assign the full point cloud input to a new variable
			*cloud_in = *input;

			// Adjust depth comparison array -- height change and delta values 
			for (int i = 0; i < 544; i++)
			{
		    		if(i<300)
		        		hyp[i] = (camera_height-(height_change))/sin((row_start_angle) + (inc_angle_row * i));
		    		else
		        		hyp[i] = (camera_height-(height_change))/sin((row_start_angle) + (inc_angle_row * i));
			}
			//-----------------------------------------------------------------------------------
			// *************************GROUND PLANE EXTRACTION*****************************

			// Set up Iteration Duration Variables, to measure time it takes for iteration
			begin_ground = ros::Time::now(); // set begin_it = now

			// Declare variables for iterated cloud & ground plane image -- must be reset for each frame
			int it = 0; // Iterator for ground plane image
			float pos = 0; // position in pointcloud

			// Get Field Offset values
			int x_index = cloud_in->fields[0].offset;
			int y_index = cloud_in->fields[1].offset;
			int z_index = cloud_in->fields[2].offset;
			int rgb_index = cloud_in->fields[3].offset;

			// Extract image from Full Point Cloud
			pcl::toROSMsg(*cloud_in, *in_image);

			// Initialize Structure Elements for ground plane image to be the same as the input image from the pointcloud
			ground_image->header = in_image->header;
			ground_image->header.stamp = in_image->header.stamp;
			ground_image->header.frame_id = in_image->header.frame_id;
			ground_image->height = in_image->height;
			ground_image->width = in_image->width;
			ground_image->encoding = in_image->encoding;
			ground_image->is_bigendian = in_image->is_bigendian;
			ground_image->step = in_image->step;
			ground_image->data.resize(in_image->width*in_image->height*3);

			// ************************* Actual Extraction *************************
			for (int k = 0; k < cloud_in->height; k++)
			{
		    		for (int l = 0; l < cloud_in->width; l++)
				{
		        		// Calculate the index position of the next point in the point cloud using row and column
		        		pos = k*cloud_in->row_step + l*cloud_in->point_step;

		        		// Extract z-value
		        		memcpy(&z,&cloud_in->data[pos+z_index],sizeof(float));

		        		// Check if the z-value is greater than the comparison matrix threshold value
		        		if (z > z_percent*(hyp[k]))
					{
		            			// Fill Ground Plane Image RGB values with RGB values from iterated image
		            			ground_image->data[it] = in_image->data[it];
		            			ground_image->data[it+1] = in_image->data[it+1];
		            			ground_image->data[it+2] = in_image->data[it+2];

		            			// Point step iterator for ground_image data
		            			it = it + 3;
		        		}
		        		else  // If z-value is NOT greater than expected distance value, assign the RGB values of the pixel in question with NaN
					{
		            			// Fill Ground Image RGB values with NaN's for this pixel
		            			ground_image->data[it] = NaN;
		            			ground_image->data[it+1] = NaN;
		            			ground_image->data[it+2] = NaN;

		            			// Point step iterator for ground_image data
		            			it = it + 3;
		        		} // END if/else*/
		    		} // END column for loop
			} // END row for loop

			// Calculate total time to extract the ground plane
			ground_duration = ros::Time::now() - begin_ground;

			/* Create a scaled image based on the metric measurements from the stereo vision camera*/
			// Initialize variables for metric image loop
			float im_pos = 0; // row/col position of ground plane image
			int row = 0; // Calculated row value for metric image
			int col = 0; // Calculated column value for metric image
			int row_col = 0; // row/col position of metric image

			// Initialize Structure Elements for metric_image
			metric_image->header = ground_image->header;
			metric_image->header.stamp = ground_image->header.stamp;
			metric_image->header.frame_id = ground_image->header.frame_id; // "segway/multisense/left_camera_optical_frame";
			// New height value of metric image to account for the amount of rows between camera and where the ground in the camera's frame
			// metric_image->height = 626;  // 5.632/0.009 = 625.7 (Z-VALUE)
			metric_image->height = 246; //330; //412 //438 // 3.((3.9435-1.5879)/0.009)*2 = 546 (Y-VALUE)
			metric_image->width = 750;
			metric_image->encoding = ground_image->encoding;
			metric_image->is_bigendian = ground_image->is_bigendian;
			metric_image->step = 2250;
			// Resize metric image to set the amount of bytes contained in each image & clear
			metric_image->data.resize(metric_image->width*metric_image->height*3);

			// Initialize metric image RGB values to black [255]
			for (int init_met = 0; init_met < (metric_image->width*metric_image->height)*3; init_met++)
			{
		    		metric_image->data[init_met] = NaN;
		    		metric_image->data[init_met+1] = NaN;
		    		metric_image->data[init_met+2] = NaN;
			}

			// *************************CONSTRUCT METRIC IMAGE*************************

			for (int m = 0; m < cloud_in->height; m++)
			{
		    		for (int n = 0; n < cloud_in->width; n++)
		    		{
		        		// Calculate position of point cloud to extract x & y coordinates
		        		pos = m * cloud_in->row_step + n * cloud_in->point_step;

		        		// Calculate the position of the ground plane image
		        		im_pos = m * ground_image->step + n * 3;

		        		// Extract the x,y,z coordinates from point cloud
		        		memcpy(&x, &cloud_in->data[pos + x_index], sizeof(float));
		        		memcpy(&y, &cloud_in->data[pos + y_index], sizeof(float));
		        		memcpy(&z, &cloud_in->data[pos + z_index], sizeof(float));
					// Divide x and y coordinates by 9mm to determine row and column values
		        		//col = floor(x / 0.009) + (375); // Add 375 columns to correct for positive and negative values (original)
		        		col = floor(x / 0.0129) + (375);
		       			// row = floor(y / 0.009) + (177); // Add 177 (273) rows to correct for positive and negative values (original)
		        		row = 245 - (((1.25-(y /cos(.6109)))/tan(.6109)-.7963)/.0197); // Add 177 (273) rows to correct for positive and negative values (original) ... .7963 = 1.25m/tan(57.5deg)
		       			// row = floor(y / 0.0197) + (123); // Add 177 (273) rows to correct for positive and negative values

		        		// Calculate new row and column position for metric image using row and col variables
		        		row_col = (row * metric_image->step) + (col * 3);

		        		// Assign RGB values to the metric image row and column coordinates if they are within the boundaries of metric image

					if (row >= 0 && row < 246)
		        		{
		            			if (col >= 0 && col < 750)
		            			{
		                			metric_image->data[row_col] = ground_image->data[im_pos];
		                			metric_image->data[row_col+1] = ground_image->data[im_pos+1];
		                			metric_image->data[row_col+2] = ground_image->data[im_pos+2];
		            			} // end column check
		        		} // end row check
				} // END column for loop
			} // END row for loop

			// Check for data in metric image, if no data, return
			newData = true;

			//calculate total time to convert to XYZRGB =-=-=-=--=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-==-=-=-=-=-=-==-==-=-=-=-=-=-==-=-=-
			callback_duration = ros::Time::now() - begin_callback;

	  	} // end ground_plane_cb function
	  	//************************back to colorcloud class definition*********************************************************************/

	  	//******************************************************************************/
	  	//---------------lane detect callback function---------------------------------
	  	//******************************************************************************/
	  	void lane_detect_callback(const sensor_msgs::ImageConstPtr& msg)
	  	{
			begin_lane = ros::Time::now();

		cv_bridge::CvImagePtr cv_ptr; // Pointer of OpenCV data type

		namespace enc = sensor_msgs::image_encodings;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // Convert to open_cv type
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		} //end try

		// "the big if"
		// corrects image size mismatch
		if (cv_ptr->image.rows >= 246 && cv_ptr->image.cols >= 750)
		{
			std::vector<cv::Vec2f> lines;
			int houghVote = 200;

		 	// Set threshold value
			int threshold = 230;
			int thresh_count = 0;  // Number of "white" pixels that are thresholded
			int thresh_offset = 10;  // Maximum deviation value between layers

			// Assign CV image to a cv::Mat variable
			bgr = cv_ptr->image;

			    // Separate the 3-layer image in to 3 separate layers
			cv::split(bgr,channel);

			    // Assign the separated layers to their respective color matix 
			    /*
			Shouldnt b=channel[0]
			r=channel[2]
			    */
			red = channel[2];
			green = channel[1];
			blue = channel[0];

			    // cv::GaussianBlur( green, green, cv::Size(3,3), 0, 0 );  // Perform gaussian function for filtering

			    // Color Layer Manipulation
			    // blue_green = 2*blue - green;
		   	// blue_red = 2*blue - red;
		   	gray_out = 2*blue - red;
		   	
		   	//cv::imshow("B-R DIFFERENCE", gray_out);  // New rgb image after layer manipulation
			//cv::GaussianBlur( gray_out, gray_out, cv::Size(5,5), 0, 0 );  // Perform Gaussian function for
					// Gaussian blur the layer manipulated image to increase homogeneity of background
			// cv::GaussianBlur( blue_green, gray_out, cv::Size(3,3), 0, 0 );  // Perform Gaussian function for filtering (mask: 5x5)
			// cv::GaussianBlur( blue_red, gray_out, cv::Size(3,3), 0, 0 );  // Perform Gaussian function for filtering (mask: 5x5)

			    // LOW FILTER - ESTIMATED MEAN & STD
			    gray_out.convertTo(mean_img, CV_32F);  // Convert to dtat type which will allow matrix math
			    cv::blur(mean_img, mu, cv::Size(1,25));  // Take the mean of image using a row-mask {1-by-#_of_col}
			    cv::blur(mean_img.mul(mean_img), mu2, cv::Size(1,25));  // Square the mean of image
			    cv::sqrt(mu2 - mu.mul(mu), sigma);  // Calculate standard deviation
			    cv::addWeighted(mu,alpha,sigma,beta,0.0,mean_thresh);  // mean_thresh = alpha*mu + beta*sigma


			    gray_out1 =  mean_thresh - mean_img;
			    
				/*	The following comment was sent by Phil on 6-4-18 during the competition		     
				 If the values of mean_thresh are negative then the pixels are not kept
				 gray_out1 is used to store the result of a filter that will **ELIMINATE** the values below the mean threshold of the   image. This happens on line 539 by subtracting the mean values of the image, mean_img (row-mask = 1row-by-25col) from a  calculated mean threshold, mean_thresh = alpha*mu + beta*sigma. This value, gray_out1, may have negative values which  will become zeros, but its the positive values we care about.  gray_out1 is turned into a binary image with all of the  "positive values" or pixels who were above the threshold becoming 1, leaving the lower values to 0. These positive   values are the pixel values that were deemed higher that the mean and therefore are most likely to contain white pixels (lane lines). gray_img1 is then multiplied by the original gray_out as a way to initially filter out the pixels with a lower chance of containing white (line 550).
				*/			    
			    
			    // Threshold the positive values
			    cv::threshold( gray_out1, gray_out1, 0, 1,CV_THRESH_BINARY);   // input, output, threshold, value to replace pixel, threshold type
			    // Convert back to u_int8 type to make simple functions quicker
			    gray_out1.convertTo(gray_out1, CV_8U);
				
			    //cv::normalize(gray_out1,gray_out1,0,255, 32);

			    // cv::imshow("LOW FILTER THRESH", gray_out1);

			    // Restore original values of grayscale image
			    gray_out = gray_out.mul(gray_out1);
				
			    // cv::imshow("LOW FILTER", gray_out);

			// BRIGHTNESS ADAPTIVE FILTER
			    // Determine global threshold depending on the intensity value that contains the least amount of
			    // white pixel values.


			    // BRIGHT & SUNNY
			    /*  None of the following was done for final runs.
			    2) CHANGE thresh_count IN LINE 568 (the 2nd if statement for BRIGHT & SUNNY) TO ~600-1000! 
				As for the threshold within that if statement Sunny/Bright, I would leave it around ~245-250.
				PARTY CLOUDY: change threshold = i; to threshold = i + 30;

				The theory behind using "300" as the thresh_count is because I found that this was a close estimate of the minimum amount of white pixels when 2 lane lines are present (or a solid single line). So once the for loop finds this many "white pixels" in an image it will set the threshold to what is reasonable for each brightness condition since sunny days will return higher values resulting in more false positives/noise, so increasing the threshold to 250 would hopefully filter out most noise. But first, to allow the threshold to be set, the second if statement must be changed from 3000 to a lower value as mentioned above.
				*/
			    
			    for (int i = 200; i < 255; i++)
			{
			    	gray_outb = gray_out>i;
				    thresh_count = cv::countNonZero(gray_outb);
				if(thresh_count > 300)
				{
					    threshold = i-20;
					    thresh_offset = 7;
					    if(thresh_count > 3000)
					{
						threshold = 250;
						    thresh_offset = 2;
					    }
			  		}
				}

				// PARTLY CLOUDY
				if (threshold == 230)
			{
				  	for (int i = 150; i < 200; i++)
				{
			  			gray_outb = gray_out>i;
			  			thresh_count = cv::countNonZero(gray_outb);
			    		if(thresh_count > 300)
					{
			      			threshold = i;
			      			thresh_offset = 10;
			    		}
			  		}
				}

				// OVERCAST CLOUDY
				if (threshold == 230)
			{
			  		for (int i = 100; i < 150; i++)
				{
			    		gray_outb = gray_out>i;
			    		thresh_count = cv::countNonZero(gray_outb);
			    		if(thresh_count > 150)
					{
			      			threshold = i+30;
			      			thresh_offset = 5;
			    		}
			  		}
				}

				// DARK
				if (threshold == 230)
			{
			  		for (int i = 50; i < 100; i++)
				{
				  		gray_outb = gray_out>i;
				  		thresh_count = cv::countNonZero(gray_outb);
				    	if(thresh_count > 200)
					{
				      		threshold = i;
				      		thresh_offset = 5;
				    	}
				}
			}

			ROS_INFO_STREAM("WHITE PIXELS:\t" << thresh_count);
			ROS_INFO_STREAM("THRESHOLD:\t" << threshold);

//threshold = 200;
			// Threshold the grayscale image using calculated threshold from adaptive filter
			cv::threshold( gray_out, bgr_out, (threshold), 255,CV_THRESH_TOZERO);
			
			   // input, output, threshold, value to replace pixel, threshold type
			//cv::imshow("GRAYSCALE IMAGE", gray_out);  // New thresholded image


			cv::Canny( bgr_out, canny_out, 60, 250, 3);   // lowThreshold, lowThreshold*ratio, kernel_size
			temp = canny_out;
			cv::medianBlur(canny_out,canny_out,3);

			if (houghVote < 1 || lines.size() >2)
			{
				houghVote = 200;
			}
			else
			{
				houghVote += 25;
			}

			//while(lines.size() < 5 && houghVote > 0)
			//{
				cv::HoughLines(canny_out,lines,1,(3.14/180)/4,houghVote);
				//  houghVote -= 5;
			//}

			bgr_out.copyTo(result);

			// cv::imshow("CANNY EDGE DETECTION", gray_out1);  // B&W image in BGR format

			// Draw the lines
			std::vector<cv::Vec2f>::const_iterator it = lines.begin();
			cv::Mat hough(canny_out.size(),CV_8U,cv::Scalar(0));
			while (it!=lines.end())
			{
				float rho = (*it)[0];
				float theta = (*it)[1];

				//if ( theta > 0.22 && theta < 2.95 )
				//{
					cv::Point pt1(rho/cos(theta),0);
					cv::Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);

					// Draw white line
					cv::line(result, pt1, pt2, cv::Scalar(255),8);
					cv::line(hough, pt1, pt2, cv::Scalar(255),8);
				//}
				++it;
			}

			// Create LineFinder instance
			LineFinder ld;

			// Set probabilistic Hough parameters
			ld.setLineLengthAndGap(30,20);  // min_length, gap
			ld.setMinVote(4);

			// Detect lines
			std::vector<cv::Vec4i> li = ld.findLines(result); //canny_out
			cv::Mat houghP(canny_out.size(),CV_8U,cv::Scalar(0));
			ld.drawDetectedLines(houghP);

			cv::dilate(houghP,houghP,cv::Mat(),cv::Point(-1,-1),2,1,1);

			// cv::imshow("HOUGH LINES", houghP);  // B&W image in BGR format

			cv::cvtColor(houghP, out1, CV_GRAY2BGR);  // Convert grayscale image to RGB format
			//cv::cvtColor(canny_out, out1, CV_GRAY2BGR);  // Convert grayscale image to RGB format

			out_img.header = metric_image->header;
			out_img.encoding = sensor_msgs::image_encodings::RGB8; // Specify format of output image
			//cv::cvtColor(temp, out1, CV_GRAY2BGR);  // Convert grayscale image to RGB format
			out_img.image = out1;  // Edge detection image in RGB format

			lane_duration = ros::Time::now() - begin_lane;
			// cv::waitKey(3);

		}//------------ end "big if"-----------


		//********************************************************************************//
		//**********************LLLLLLLLLLLAAAAAAAAAAAZZZZZZZZZZZZZEEEEEEEEEEERRRRRRRRR***//
		//********************************************************************************//

		float mmppc=.0129, mmppr=.0197;
		int pcol=375, prow=245;
		double laser_res = .5, laser_offset=.7963;
		float min_las_ang = 1000, max_las_ang = -1000;
		std::vector<float> x(0),y(0),r(0),theta(0), deg(0),vfh_deg(0);//, scan((max_las_ang-min_las_ang)/laser_res + 1,6);

		/*----------Locate White Pixels (x,y)---------*/
		// Find the x and y distances of the
		// white pixels with respect to (p_row,p_col)
		for(int row=0; row<out1.rows; row++)
		{
			for(int col=0; col<out1.cols; col++)
			{
				if(out1.at<cv::Vec3b>(row,col)[0]>200)
				{
					//x.push_back((col-pcol)*mmpp);
					//y.push_back((prow-row)*mmpp+laser_offset);
					x.push_back(((prow-row)*mmppr) + laser_offset);
					y.push_back((pcol-col)*mmppc);
				}
			}
		}
		//ROS_INFO_STREAM("laser_offset: " << laser_offset);
		//DEBUG
		//search for min and max y position
		if((!y.empty()) && (!x.empty()))
		{
			float ymin = y.at(0), xmin = x.at(0);
			int y0=0;
			int ind = 0;

			for(int i = 0; i < y.size(); i++)
			{
				if(y.at(i)<ymin);
				{
					ymin=y.at(i);
				}

				if(x.at(i)<xmin)
				{
					xmin=x.at(i);
					ind=i;
				}
				if(y.at(i)==0)
					y0++;
			}

			//ROS_INFO_STREAM("xmin " << xmin << " xind " << x.at(ind) << " y0 count " <<y0);
		}


		/*------------------Cart to polar---------------*/

		/*
		std::vector<float>::iterator yit = y.begin();
		std::vector<float>::iterator xit = x.begin();

		bool endflag = false;

		do{
			if ( (xit == x.end()) || (yit == y.end()) )
				endflag = true;

			//theta.push_back(180*atan2(*yit,*xit)/M_PI -90); // shift to I and IV quadrants
			theta.push_back(180*atan2(*yit,*xit)/M_PI);
			// -- keep on -180 to 180 --
			//if(theta.back() < -180)
				//theta.back() = theta.back() +360;
			r.push_back(sqrt(pow(*xit,2)+pow(*yit,2)));

			if(!endflag)
			{
				//haven't reached end of either x or y; iterate again
				yit++;
				xit++;
			}
		}while(!endflag);
		*/

		//old code for above; has boudary issue
		std::vector<float>::iterator yit = y.begin();
		for (std::vector<float>::iterator it = x.begin(); it != x.end(); it++)
		{
			//theta.push_back(180*atan2(*yit,*it)/M_PI -90); // shift to I and IV quadrants
			theta.push_back(180*atan2(*yit,*it)/M_PI);
			// keep on -180 to 180
			//if(theta.back() < -180)
				//theta.back() = theta.back() +360;
			r.push_back(sqrt(pow(*it,2)+pow(*yit,2)));

			yit++;
		}


		//DEBUG
		//find and print min theta and x, y found there
		if(!theta.empty())
		{
			float tmin = theta.at(0);
			int ind = 0;

			for(int i = 0; i<theta.size(); i++)
			{
				if(theta.at(i)<tmin)
				{
					tmin = theta.at(i);
					ind = i;
				}
			}

			//ROS_INFO_STREAM("tmin " << tmin <<" theta(i) " << theta.at(ind)<< " x(i) " << x.at(ind)<< " y(i) "<<y.at(ind));
		}


		/*--------degrees/(degrees/index)=index-------*/
		for (int i=0; i<theta.size();i++)
		{
			deg.push_back(floor(theta.at(i)/laser_res));
			//vfh_deg.push_back(floor(theta.at(i)/laser_res)+80);
			//if(vfh_deg.back() < (float)(0) || vfh_deg.back() > (float)(161))
				//vfh_deg.back()=400; // flag
		}

		/* // old; may have boundary issue
		for (std::vector<float>::iterator tit = theta.begin(); tit != theta.end(); ++tit)
		{
			deg.push_back(floor(*tit/laser_res));
			//if(deg.back() < (float)(min_las_ang/laser_res) || deg.back() > (float)(max_las_ang/laser_res))
				//deg.back()=400; // flag
		}
		*/

		//DEBUG
		/*if(!deg.empty())
		{
			ROS_INFO_STREAM("first deg " << deg.at(0));
			ROS_INFO_STREAM("last deg " << *deg.end());
		}*/

		/*---Delete indices and ranges outside of angle range--*
		
		for (uint i = 0; i < vfh_deg.size(); i++)
		{
			if(vfh_deg.at(i) == 400)
			{
				vfh_deg.erase(vfh_deg.begin()+i);
				vfh_r.erase(vfh_r.begin()+i);
				// i-- is needed because the size of the array has decreased
				// therefore, all indices head of the element deleted have
				// been shifted down by one
				i--;
			}
		}
		
*/
		/*-------------max and min theta angle----------------*/
		// step through theta angle, and find max and min, respectively
		// find min and max theta angle
		bool min = false, max = false;

		for(int i=0; i<theta.size(); i++)
		{
			if(theta.at(i)<min_las_ang)
			{
				min_las_ang=theta.at(i);
				min = true;
			}
		}

		for(int i=0; i<theta.size(); i++)
		{
			if(theta.at(i)>max_las_ang)
			{
				max_las_ang=theta.at(i);
				max = true;
			}
		}

		// check if the max and min have been set during the loop;
		// if not, set them to a default value
		if(!min)
			min_las_ang=0;
		if(!max)
			max_las_ang=0.5;

		//ROS_INFO_STREAM("deg length " << deg.size() << " max_las_ang " << max_las_ang);

		/*-----Recompensate for possible negative angles -- START AT ZEROTH INDEX-------*/
		//MAYBE LOOK AT THIS AGAIN======================================================================================
		for(std::vector<float>::iterator dit = deg.begin(); dit != deg.end(); dit++)
			*dit = *dit - min_las_ang/laser_res;


		/* Sort ranges from high to low and keep associated deg*
		if(!r.empty() & !deg.empty()){
			float max_value=r.front();
			float max_deg=deg.front();
			uint max_ind=0;
			for(uint j=0;j<r.size();j++){
				for(int i = max_ind; i <r.size(); i++){
					if(*(r.begin()+i)>max_value){
						max_value = *(r.begin()+i);
						max_deg = *(deg.begin()+i);

						*(r.begin()+i)=*(r.begin()+max_ind);
						*(deg.begin()+i)=*(deg.begin()+max_ind);

						*(r.begin()+max_ind)=max_value;
						*(deg.begin()+max_ind)=max_deg;
					}
				}
				max_ind++;
				max_value=*(r.begin()+max_ind);
			}
		}

		*/
		/*
		for(int i=0; i<theta.size(); i++){
			if(theta.at(i)<min_las_ang){
				min_las_ang=theta.at(i);
			}
		}
		for(int i=0; i<theta.size(); i++){
			if(theta.at(i)>max_las_ang){
				max_las_ang=theta.at(i);
			}
		}
		*/


		/* ---------- make output scan message ros-style ---------- */
		unsigned int num_readings = (int)((max_las_ang-min_las_ang)/laser_res + 1);
		double laser_frequency = 40;
		double ranges[num_readings];
		double intensities[num_readings];

		ros::Time scan_time = ros::Time::now();
		scan.header.stamp = scan_time;
		scan.header.frame_id = "segway/base_link";
		scan.angle_min = M_PI*min_las_ang/180;
		scan.angle_max = M_PI*max_las_ang/180;
		scan.angle_increment = M_PI*laser_res/180;
		scan.time_increment = (1 / laser_frequency) / (num_readings);
		scan.range_min = 0.8;
		scan.range_max = 5.7;
		scan.ranges.resize(num_readings);
			scan.intensities.resize(num_readings);

		//fill original scan with Very long range OR HUGE_VALF (for infinity)
		for(unsigned int i = 0; i < num_readings; ++i)
		{
			  scan.ranges[i] = HUGE_VALF;
			  //scan.intensities[i] = 1;
			}

		/*-----fill the scan-----*/
		//populate with actual values
		for(uint i = 0; i < deg.size(); i++)
		{
			scan.ranges[(int)deg.at(i)]=r.at(i);
		}
		
		
		
		
		ros::Time flan_time = ros::Time::now();
		flan.header.stamp = flan_time;
		flan.header.frame_id = "segway/base_link";
		flan.angle_min = -M_PI*40/180;
		flan.angle_max = M_PI*40/180;
		flan.angle_increment = M_PI*laser_res/180;
		flan.time_increment = (1 / laser_frequency) / (num_readings);
		flan.range_min = 0.8;
		flan.range_max = 5.7;
		flan.ranges.resize(161);

		//fill original scan with Very long range OR HUGE_VALF (for infinity)
		for(unsigned int i = 0; i < 161; ++i)
		{
			  flan.ranges[i] = HUGE_VALF;
			  //scan.intensities[i] = 1;
			}

		/*-----fill the scan-----*
		//populate with actual values
		int sin = vfh_deg.at(0);
		for(uint i = 0; i < vfh_deg.size(); i++)
		{
			if(((int)vfh_deg.at(i))<0)
				sin = vfh_deg.at(i);
			//flan.ranges[(int)vfh_deg.at(i)]=r.at(i);
		}
		
			ROS_INFO_STREAM("SHITTT!" << sin);
	*/	
		

		x.clear();
		y.clear();
		theta.clear();
		r.clear();
		deg.clear();
		vfh_deg.clear();
		} // --------------- END lane_detect_callback --------------- //

	  //******************************************************************************END PUBLIC SECTION

	  private:
		// Subscriber
		ros::Subscriber sub_imu;

		ros::Subscriber sub_ground;
		ros::Subscriber sub_metric;

		// Publishers
		ros::Publisher pub_it_im;
		ros::Publisher pub_ground_img;
		ros::Publisher pub_metric_img;
		ros::Publisher pub_lane_lines;
		ros::Publisher pub_laser;

		ros::Publisher pub_cloud;
		//int jaus;

	  }; // END CLASS point_cloud

  PLUGINLIB_EXPORT_CLASS(color_cloud_nodelet::point_cloud, nodelet::Nodelet);  //export point_cloud as a plugin

}//END NAMESPACE color_cloud_nodelet

