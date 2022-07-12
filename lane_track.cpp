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

using namespace cv;

// FRAME ACQUISITION AND RESIZING VARIABLES
char newImage;
Mat frame, resized, cropped_image;

// MATH VARIABLES
int image_width = 640;
int image_height = 480;
int t, i, j, totalTime, dataX, dataY, dataXleft, dataYleft, dataXright, dataYright;

// PIXEL VALUE NORMALIZATION VARIABLES
Mat op_img, op_img1, op_img2, op_img3;
Mat mask, binaryOnes;

// LANE SCANNING VARIABLES
bool coordinates, LANE, cont, pixelFound;
int length; // length of detect white points array
int counter = 0;
int horizantalValue;
int imageWindow1, imageWindow2, imageWindow3, imageWindow4, vehicleXY;
int wl; // which lane digit to determine lane points to populate
int runItUp; // determine if white point was found in last few iterations

Point chosenPoint, detectedPoint, newdetectedPoint, realPoint, midPointLeft, midPointRight;
Point *CP;
Vec3b imagePixel, imagePixel1;

Point *L;// = LEFTpoints.data();
Point *R;// = RIGHTpoints.data();

std::vector<int> xVals, yVals, newXvals, newYvals, legit_points;
std::vector<Point> circlePoints, leftPoints, leftEgoPoints, leftAdjPoints, LEFTpoints;
std::vector<Point> rightPoints, rightEgoPoints, rightAdjPoints, RIGHTpoints;
std::vector<Point> FinalLeftAdj, FinalLeftEgo, FinalRightAdj, FinalRightEgo;

Point *C, *D, *E;


// IMAGE COLORING VARIABLES
Scalar cyan(255, 150, 0);
Scalar green(0, 255, 0);
Scalar blue(255, 0, 0);
Scalar red(0, 0, 255);
Scalar purple(130,20,70);
Scalar orange(20,150,200);

std::string whichLane;

int thickness = 2;
int *posX;
int *posY;
int LL, LY, RL, RY;
Point *legitLeft;
Point *legitRight;

ros::Time begin;
ros::Time end;

// REUSABLE LANE FUNCTIONS
void leftRightScan(int horizantalValue) //take in the horizantal pixel value to scan across 
{
    int j = horizantalValue;
    for (int i = 1; i < resized.cols; i++) // scan left to right
    {
        if (resized.at<Vec3b>(j, i)[0] > 0) // HERE ONLY READING (Y,X) WORKS - NOT (X,Y) AS USUAL
        {
            // determine pixel coordinates and output pixel value
            imagePixel = resized.at<Vec3b>(j, i);

            // store pixel coordinates in dynamic array
            xVals.push_back(i);
            yVals.push_back(j);

            // store some test points to output
            // chosenPoint = Point(375, j); //blue circle
            detectedPoint = Point(i, j); // green circle = last detected point
        }
    }

    // READ FOUND PIXELS AND POPULATE THE PAIRS
    posY = yVals.data();
    posX = xVals.data();
}

void checkLegitLane(std::vector<Point> lanePoints, int horizantalValue)
{
    j = horizantalValue;
    CP = lanePoints.data();
    for (int d = 0; d < lanePoints.size(); d = d + 10) // check every 3 left points to confirm they are still in the y-axis
    {
        // ROS_INFO_STREAM("Circle Points Data 1: " << *CP);
        int pointValue1 = *posX;
        // ROS_INFO_STREAM("Position X1: " <<pointValue1);

        *CP++;
        int *posx = posX + 1;
        // ROS_INFO_STREAM("Circle Points Data 2: " << *CP);
        int pointValue2 = *posx;
        // ROS_INFO_STREAM("Position X2: " <<pointValue2);

        // CHECK TO DETERMINE IF VALID
        int result = pointValue1 - pointValue2;
        bool outside;
        bool true_lane;

        if (result > -3 && result < 3)
        {
            // if point are within 3 tolerance then increment counter
            // if counter is abovee a value then legitimate lane
            counter++;
            //ROS_INFO_STREAM("In Range - Counter: " << counter);
            outside = false;
        }

        else
        {
            //ROS_INFO_STREAM("Outside Range");
            outside = true;
            // counter = 0;
        }

        if (outside == false)
        {
            if (counter < 3)
            {
                true_lane = false;
            }
            else
                true_lane = true;
        }

        Point legit_point;

        if (true_lane == true)
        {
            //ROS_INFO_STREAM("Legitimate Lane");
            // HERE CREATE A VECTOR OF POINTS THAT ONLY GETS PUSHED BACK WITH LEGITIMATE LEFT LANE SEGMENTS
            legit_point = Point(pointValue1,j);

            if (*posX < resized.cols / 2)
            {
                LEFTpoints.push_back(legit_point);
            }

            if (*posX > resized.cols / 2)
            {
                RIGHTpoints.push_back(legit_point);
            }
            
            circlePoints.push_back(legit_point);

        }

        if (true_lane == false)
        {
            // ROS_INFO_STREAM("Not Legitimate Lane");
            // HERE WE WILL IGNORE PAIRS THAT DON'T MEET POTENTIAL LANE DISTANCE AND COUNTER REQUIREMENTS
        }
    }
}

void findMidpoints(std::vector<Point> Ls, std::vector<Point> Rs)
{
    L = Ls.data();
    R = Rs.data();
    
    for (int l = 0; l < Ls.size(); l++)
    {
        if (l == Ls.size() / 2 + 1)
        {
            dataXleft = u_int(L->x);
            dataYleft = u_int(L->y);
            midPointLeft = Point(dataXleft, dataYleft);
            ROS_INFO_STREAM("Left Lane Mid Point: " << midPointLeft);
            LANE = true;
        }
        *L++;
    }

    for (int r = 0; r < Rs.size(); r++)
    {
        if (r == Rs.size() / 2 + 1)
        {
            dataXright = u_int(R->x);
            dataYright = u_int(R->y);
            midPointRight = Point(dataXright, dataYright);
            ROS_INFO_STREAM("Right Lane Mid Point: " << midPointRight);
            LANE = true;
        }
        *R++;
    }
}

void populateLanes()
{        
    // HERE I WANT TO READ THE X AND Y VALUE TO DETERMINE IF LEFT OR RIGHT LANE SEGMENT THEN POPULATE ACCORDINGLY 
    for (int c = 0; c < yVals.size(); c++)
    {
        // imwrite("circle_pre.png",op_img1);

        // POPULATE CIRCLE POINT ON EACH ITERATOIN FOR checkLegitLane FUNCTION
        Point circlePoint(*posX, *posY);

        // CHECK IF LEGITIMATE LANE SEGMENT
        checkLegitLane(leftPoints, j); // outputs LEFTpoints which holds legit lanes
        checkLegitLane(rightPoints, j); // outputs RIGHTpoints which holds legit lanes

        // CONFIRM LEFT EGO OR ADJACENT

        int dataValue = *posX;
        if (*posX < resized.cols / 2)
        {
            leftPoints.push_back(circlePoint);
            circle(op_img1, circlePoint, 2, cyan, thickness); // here draw on each point detected before moving
        }
        // CONFIRM RIGHT EGO OR ADJACENT
        if (*posX > resized.cols / 2)
        {
            rightPoints.push_back(circlePoint);
            circle(op_img1, circlePoint, 2, cyan, thickness); // here draw on each point detected before moving
        }

        // HERE WE WANT TO TRACK ON EACH POTENTIAL LANE INITAL POINT AND CHECK 3 POINTS VERTICAL TO IT - IF 3 OR MORE THEN LANE - IF LESS THAN NOT

        //imshow("Lane Segments", op_img1); // BEFORE OUTLIER POINT REMOVAL

        *posX++;
        *posY++;
    }

    // WE EXPERIMENTALLY DETERMINE 290 TO BE THE DISTANCE BETWEEN LANES

    findMidpoints(LEFTpoints,RIGHTpoints);

    vehicleXY = (midPointRight.x + midPointLeft.x)/ 2;
    imageWindow1 = 1; // first pixel furthest left
    imageWindow2 = vehicleXY - 290 / 2 - 10; //blue
    imageWindow3 = vehicleXY + 290; //purple
    imageWindow4 = vehicleXY + 290 / 2 + 10; //orange

    Point vehicleLocation(vehicleXY, j);
    Point window1(imageWindow1, j);
    Point window2(imageWindow2, j);
    Point window3(imageWindow3, j);
    Point window4(imageWindow4, j);

    op_img3 = op_img2.clone();

    circle(op_img3, vehicleLocation, 2, green, thickness);
    circle(op_img3, window1, 2, red, thickness);
    circle(op_img3, window2, 2, blue, thickness);
    circle(op_img3, window3, 2, purple, thickness);
    circle(op_img3, window4, 2, orange, thickness);

    imshow("Lane Sections", op_img3);

    legitLeft = LEFTpoints.data();
    legitRight = RIGHTpoints.data();

    
    for(int e=1; e<LEFTpoints.size();e++)
    {
        LL = u_int(legitLeft->x);
        LY = u_int(legitLeft->y);

        RL = u_int(legitRight->x);
        RY = u_int(legitRight->y);

        if(LL < vehicleXY)
        {
            //ROS_INFO_STREAM("LL Value: "<<LL);
            //ROS_INFO_STREAM("Image Window Thresh: "<<imageWindow2);

            if(LL > imageWindow2 && LL < vehicleXY)
            {
                //ROS_INFO_STREAM("Left Ego");
                whichLane = "Left Adjacent";
                wl = 1;
                Point leftEgoPoint (LL,LY);
                leftEgoPoints.push_back(leftEgoPoint);
            }
            
            if(LL > imageWindow1 && LL < vehicleXY)
            {
                //ROS_INFO_STREAM("Left Adjacent");
                whichLane = "Left Ego";
                wl = 2;
                Point leftAdjPoint (LL,LY);
                leftAdjPoints.push_back(leftAdjPoint);
            }

        }
        
        //ROS_INFO_STREAM("Which Lane: "<<whichLane);

        if(RL > vehicleXY)
        {
            //ROS_INFO_STREAM("RL Value: "<<RL);
            //ROS_INFO_STREAM("Image Window Thresh: "<<imageWindow2);

            if(RL < imageWindow4 && RL > vehicleXY)
            {
                whichLane = "Right Ego";
                wl = 3;
                Point rightEgoPoint (RL,RY);
                rightEgoPoints.push_back(rightEgoPoint);
            }

            if (RL > imageWindow4 && RL < imageWindow3)
            {
                whichLane = "Right Adjacent";
                wl = 4;
                Point rightAdjPoint (RL,RY);
                rightAdjPoints.push_back(rightAdjPoint);
            }
        }

        //ROS_INFO_STREAM("Which Lane: "<<whichLane);

        *legitLeft++;

    }

    D = leftEgoPoints.data();
    E = rightEgoPoints.data();

    for(int o = 0; o < leftEgoPoints.size();o++)
    {
        //ROS_INFO_STREAM("Left Ego Point: (" <<u_int(D->x) <<","<<u_int(D->y)<<")");
        //ROS_INFO_STREAM("Right Ego Point: (" <<u_int(E->x) <<","<<u_int(E->y)<<")");
        *D++;
        *E++;
    }

}

void trackLanes(Point TrackPoints, int laneNumber)
{
    int startLeftX = TrackPoints.x;
    int startLeftY = TrackPoints.y;

    int window_left = startLeftX - 15;
    int window_right = startLeftX + 15;
    int window_top = startLeftY + 5;
    int window_bottom = startLeftY - 5;

    cont = true;
    runItUp = 1;

    while (window_bottom > 10 && cont == true)
    {
        for (i = window_left; i < window_right; i++) // scan left to right
        {
            for (j = window_bottom; j < window_top; j++)
            {
                if (resized.at<Vec3b>(j, i)[0] > 0) // HERE ONLY READING (Y,X) WORKS - NOT (X,Y) AS USUAL
                {
                    //ROS_INFO_STREAM("White Pixel Detected in Window");
                    // determine pixel coordinates and output pixel value
                    imagePixel1 = resized.at<Vec3b>(j, i);

                    // store pixel coordinates in dynamic array
                    newXvals.push_back(i);
                    newYvals.push_back(j);

                    //ROS_INFO_STREAM("MidPointLeft: (" << TrackPoints.x << "," << TrackPoints.y << ")");

                    pixelFound = true;
                    
                    TrackPoints.x = i;
                    TrackPoints.y = j;

                    window_left = TrackPoints.x - 15;  // increment left window scan start point
                    window_right = TrackPoints.x + 15; // increment right window scan start point

                    window_top = TrackPoints.y - 10;
                    window_bottom = TrackPoints.y - 20;
                }

                /* // LAST RESORT FAILSAFE
                if (window_bottom < 90) // above 90 seems to be beyond the curve and creates a recursive loop - investigate deeper
                {
                    cont = false; //stop running the loop
                }
                */

                //ROS_INFO_STREAM("Window Bottom: " << window_bottom);
                //ROS_INFO_STREAM("Window Top: " << window_top);
            }
        }

        //ROS_INFO_STREAM("Pixel Found: " << pixelFound);

        if (pixelFound == true)
        {
            runItUp = runItUp + 1;
        }

        if (pixelFound == false)
        {
            runItUp = runItUp - 1;
        }

        pixelFound = false;

        //ROS_INFO_STREAM("Run It UP: " << runItUp);

        if (runItUp < 1) // && window_bottom > resized.cols - 10)
        {
            cont = false;
        }

    }
}

// MAIN LANE TRACKING PROGRAM 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "inital_lane_localization");
    ros::NodeHandle nh__;

    std::string imgChoice = "Left";
    std::string path = "/home/tyler/catkin_ws/src/laneDetection/src/Perspective_Images/";
    std::string image_path = path + imgChoice + "Turn.png";

    frame = imread(image_path);
    // Error check and display path if image doesn't exist
    if (frame.empty())
    {
        ROS_INFO_STREAM("Image Path: " <<image_path);
        ROS_INFO_STREAM("EMPTY FRAME MESSAGE");
        return 0;
    }

    newImage = true; //if the frame isn't empty then new imgae true

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        if (newImage && !frame.empty())
        {
            begin = ros::Time::now();
            // Will need to read from ROS node here and set to false after proper read
            // then reset to true at the end of operation to read again
            newImage = false;

            // resize the image to 640 by 480
            resize(frame, resized, Size(image_width, image_height), INTER_LINEAR);

            // NORMALIZE PIXELS TO 0-255 values
            op_img = resized.clone();
            resize(op_img, op_img, Size(640, 480));

            op_img1 = resized.clone();
            resize(op_img1, op_img1, Size(640, 480));

            mask = op_img > 0;
            binaryOnes = mask / 255;

            // ROS_INFO_STREAM("Normalized Ones: "<<binaryOnes); // to confirm posotive pixels found
            //ROS_INFO_STREAM("Normalized Ones Size: " << binaryOnes.size());

            op_img2 = op_img1.clone(); //create image for removed outliers


            // PREPARE LANE SCANNING 
            //j = binaryOnes.rows - 70;// set to scan along potential trouble point w/ left turn image
            j = binaryOnes.rows - 10;
            // ROWS = Y AXIS
            // COLS = X AXIS

            leftRightScan(j); //scan the image to find initial lane starting points

            counter = 0;

            populateLanes(); // call function to populate proper lanes

            findMidpoints(LEFTpoints,RIGHTpoints); // call function to find starting midpoints to be used in window tracking
            
            C = circlePoints.data();
            for (int d =0; d<circlePoints.size();d++)
            {
                dataX = u_int(C->x);
                dataY = u_int(C->y);

                dataXleft = u_int(L->x);
                dataYleft = u_int(L->y);

                //ROS_INFO_STREAM("Circle Points: "<<dataX<<" "<<dataY);

                realPoint = Point(dataX,dataY);
                circle(op_img2, realPoint, 2, red, thickness);
                circle(op_img2, midPointLeft, 2, green, thickness);
                circle(op_img2, midPointRight, 2, green, thickness);
                *C++; //update point selection pointer to iterate through vector of points
            }

            imshow("Legit Lane", op_img2);

            // HERE I WILL USE THE INITIAL LANE MIDPOINTS TO TRACK THE LANE VERTICALLY AND GATHER POINTS FOR THE POINT CLOUD
            
            trackLanes(midPointLeft,wl); // run window tracking function on left lane

            trackLanes(midPointRight,wl); // run window tracking function on right lane

            int distance = (midPointRight.x) - (midPointLeft.x); // determine distance between 2 lanes for error checking
            ROS_INFO_STREAM("DISTANCE BETWEEN LANES: "<<distance);

            int *NX = newXvals.data(); //pointer to iterate through x values 
            int *NY = newYvals.data(); //pointer to iterate through y values

            if (newXvals.empty())
            {
                ROS_INFO_STREAM("New X Vals Empty");
            }

            for(int q=1; q<newYvals.size();q++) // increment through all the values & store in appropriate classification - then output them
            {
                newdetectedPoint = Point(*NX, *NY);
                //ROS_INFO_STREAM("New Point: " << newdetectedPoint);

                if(wl == 1)
                {
                    FinalLeftAdj.push_back(newdetectedPoint);
                    //ROS_INFO_STREAM("New Left Adj Point: "<<FinalLeftAdj.data());
                }

                if(wl == 2)
                {
                    FinalLeftEgo.push_back(newdetectedPoint);
                    //ROS_INFO_STREAM("New Left Ego Point: "<<FinalLeftEgo.data());
                }

                if(wl == 3)
                {
                    FinalRightAdj.push_back(newdetectedPoint);
                    //ROS_INFO_STREAM("New Right Adj Point: "<<FinalRightAdj.data());
                }

                if(wl == 4)
                {
                    FinalRightEgo.push_back(newdetectedPoint);
                    ///ROS_INFO_STREAM("New Right Ego Point: "<<FinalRightEgo.data());
                }

                circle(op_img2, newdetectedPoint, 2, red, thickness+1); //draw output points onto BW image

                *NX++;
                *NY++;
            }

            imshow("Lanes Tracked",op_img2);

            // CALUCLATE TIME PER FRAME 
            end = ros::Time::now();
            ros::Duration Duration = end - begin;

            ROS_INFO_STREAM("Time Duration Math: " << Duration.toSec() << "s");
        }

        waitKey(3);
        ros::spinOnce();
    }
    return 0;
}
