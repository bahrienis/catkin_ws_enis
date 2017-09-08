// Autor : Bahri Enis Demirtel




#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>











#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
















#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <ctime>

#include "IPM.h"
#include "IPM.cpp"






#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"







using namespace std; 
using namespace cv;












/* INTERNAL CAM TEST CODE */
/* REF: https://www.youtube.com/watch?v=HqNJbx0uAv0 */
























int main(int argc, char **argv)
{
	
	
	
	
	
	int count=0;
	


	


  
   
    
    
 

    
  
 
  
  
    //SetUP ROS.
  ros::init(argc, argv, "lane_detector_enis");


  
  
  
  
	
  
  
  
  
  
  
  
  
  
  //CV_CAP_ANY == 0 yazınca 2. kamera açılır.
  
      VideoCapture cap(CV_CAP_ANY ); // OPENT THE VIDEO CAMERO NO. 0
      
      
      cap.set(CV_CAP_PROP_FPS,30);		//change the frame value
      
      
      
      

	if(!cap.isOpened()) //if not success, exit program
	{
		cout<<"Cannot open the video cam" << endl;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of te video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	
	
	
	double dFrame = cap.get(CV_CAP_PROP_FPS);
	
	
	
	cout << "FPS value is " << dFrame << endl;
	
	
	

	cout << "Frame size: " << dWidth << " x " << dHeight << endl;
	
	
	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	namedWindow("Control",CV_WINDOW_AUTOSIZE); //create a window called "Control"
	
	int iLowH = 0;
	int iHighH = 179;
	
	int iLowS = 0;
	int iHighS = 255;
	
	int iLowV = 0;
	int iHighV = 255;
	
	cvCreateTrackbar("LowH", "Control", &iLowH,179);
	cvCreateTrackbar("HighH", "Control", &iHighH,179);
	
	cvCreateTrackbar("LowS", "Control", &iLowS,255);
	cvCreateTrackbar("HighS", "Control", &iHighS,255);
	
	cvCreateTrackbar("LowV", "Control", &iLowV,255);
	cvCreateTrackbar("HighV", "Control", &iHighV,255);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
		Mat inputImg;
		Mat inputImgGray;
		Mat outputImg;
	
	
	cv::VideoCapture video;
		


 
  
 
 
 
 	// Main loop
	int frameNum = 0;
	for( ; ; )
	{
		

		 clock_t begin = clock();
		 
		 
		 
		printf("FRAME #%6d ", frameNum);
		fflush(stdout);
		frameNum++;

bool bSuccess = cap.read(inputImg);





















Mat imgHSV;
cvtColor(inputImg, imgHSV, COLOR_BGR2HSV);

Mat imgThresholded;

inRange(imgHSV, Scalar(iLowH,iLowS,iLowV), Scalar(iHighH,iHighS,iHighV), imgThresholded);

erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );
dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );


dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );
erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5,5)) );




imshow("Thresholded Image", imgThresholded);
imshow("Original", inputImg);

















		  imshow("Input", inputImg);
		 	 waitKey(1);
		 

		 
		 
		 
		 
		 
		 	 clock_t end = clock();
		 double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		 printf("%.2f (ms)\r", 1000*elapsed_secs);
//		 cout << 1000*elapsed_secs <<endl;
		 
		 
		 		
	
		 
	}

	return 0;
	
	

	
	}

    
  
  
  
  
 
  
  
  
  
  


































