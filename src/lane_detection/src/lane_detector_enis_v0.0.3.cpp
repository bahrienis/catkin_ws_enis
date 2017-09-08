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
	
	
	
	
	
	
	


	


  
   
    
    
 

    
  
 
  
  
    //SetUP ROS.
  ros::init(argc, argv, "lane_detector_enis");


  
  
  
  
  
  
  
  
  
  
  
  
  
  
  //CV_CAP_ANY == 0 yazınca 2. kamera açılır.
  
      VideoCapture cap(CV_CAP_ANY ); // OPENT THE VIDEO CAMERO NO. 0
      
      
      cap.set(CV_CAP_PROP_FPS,10);		//change the frame value
      
      
      
      

	if(!cap.isOpened()) //if not success, exit program
	{
		cout<<"Cannot open the video cam" << endl;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of te video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	
	
	
	
	
	
	cap.set(CV_CAP_PROP_FPS,10);		//change the frame value

	
	
	
	double dFrame = cap.get(CV_CAP_PROP_FPS);
	
	
	
	cout << "FPS value is " << dFrame << endl;
	
	
	

	cout << "Frame size: " << dWidth << " x " << dHeight << endl;
	
	
	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
	
	//while(1)
	//{
		Mat inputImg;
		Mat inputImgGray;
		Mat outputImg;
	
	
	cv::VideoCapture video;
		
/*		
		
		bool bSuccess = cap.read(inputImg); // read a new frame from video
	
		if(!bSuccess) // if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}
		
		
	*/	

     
     
 /*    
     
Vec3b intensity = inputImg.at<Vec3b>(20, 20);
uchar blue = intensity.val[0];
uchar green = intensity.val[1];
uchar red = intensity.val[2];
     
 */    
 
 
 
 

 
  
 
 
 
 
 

	
	
 
 
 
 
 
 
 
 
 
 



 
 // The 4-points at the input image	
	vector<Point2f> origPoints;
	origPoints.push_back( Point2f(0, dHeight) );
	origPoints.push_back( Point2f(dWidth, dHeight) );
	origPoints.push_back( Point2f(dWidth/2+30, 140) );
	origPoints.push_back( Point2f(dWidth/2-50, 140) );
  	 
	// The 4-points correspondences in the destination image
	vector<Point2f> dstPoints;
	dstPoints.push_back( Point2f(0, dHeight) );
	dstPoints.push_back( Point2f(dWidth, dHeight) );
	dstPoints.push_back( Point2f(dWidth, 0) );
	dstPoints.push_back( Point2f(100, 0) );
		
	// IPM object
	
	 
	 
	IPM ipm( Size(dWidth, dHeight), Size(dWidth, dHeight), origPoints, dstPoints );
	
	
	
	

	
	
	
	
	
	
	
	// Main loop
	int frameNum = 0;
	for( ; ; )
	{
		printf("FRAME #%6d ", frameNum);
		fflush(stdout);
		frameNum++;



bool bSuccess = cap.read(inputImg);
		// Get current image		
/*		video >> inputImg;
		if( inputImg.empty() )
			break;
*/
		 // Color Conversion
		 if(inputImg.channels() == 3)		 
			 cvtColor(inputImg, inputImgGray, CV_BGR2GRAY);				 		 
		 else	 
			 inputImg.copyTo(inputImgGray);			 		 

		 // Process
		 clock_t begin = clock();
		 
		 
	//	 ipm.applyHomography( inputImgGray, outputImg );  //Gri yapmak istersek
		 ipm.applyHomography( inputImg, outputImg );	

		 	 
		 clock_t end = clock();
		 double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		 printf("%.2f (ms)\r", 1000*elapsed_secs);
	//	 ipm.drawPoints(origPoints, inputImg );
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 





		 
		 
		 
		 
		 
	 
		 
		 
		 
		 
		 
//Get and show an image	 




const char* filename = argc >= 2 ? argv[1] : "/home/enis/Schreibtisch/frame0006.jpg";

 Mat src = imread(filename, 0);
 if(src.empty())
 {
     cout << "can not open " << filename << endl;
     return -1;
 }
 	 
	
	
imshow( "Display window", src );	





	
	
	
	
	
/*



 //Region of Interest

  Rect Rec(0, 240, 640, 240);		 
		 
	  rectangle(src, Rec, Scalar(255), 1, 8, 0);	 
		 
		 
		 
	  Mat Roi = src(Rec);
  namedWindow("Step 3 Draw selected Roi", WINDOW_AUTOSIZE);
 imshow("Step 3 Draw selected Roi", Roi);	 
		 
		 
		 	
	
*/	

	
	
	
	
	
		

 	


	
	
	
	
	
	
	
		 
//Sobel Operator defined		
	
	 
Mat grad;
int scale = 1;
int delta = 0;
int ddepth = CV_16S;		 
		 
GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );	//GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );		 
		 
cvtColor( inputImg, inputImgGray, CV_BGR2GRAY );		 // Bunu sil
		 
		 
Mat grad_x, grad_y;
Mat abs_grad_x, abs_grad_y;

/// Gradient X
Sobel( src, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );   //inputImgGray i src ile değiştir
/// Gradient Y
Sobel( src, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );	//inputImgGray i src ile değiştir
		 
		 
convertScaleAbs( grad_x, abs_grad_x );
convertScaleAbs( grad_y, abs_grad_y );		 
		 
addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );		 
		 
imshow( "Sobel", grad );		 
		 
		 
		 
		 




		 
	



Mat cdst,dst;
Canny(grad, dst, 50, 200, 3);
cvtColor(dst, cdst, CV_GRAY2BGR);
 
 
/* 
vector<Vec2f> lines;
HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );


for( size_t i = 0; i < lines.size(); i++ )
{
  float rho = lines[i][0], theta = lines[i][1];
  Point pt1, pt2;
  double a = cos(theta), b = sin(theta);
  double x0 = a*rho, y0 = b*rho;
  pt1.x = cvRound(x0 + 1000*(-b));
  pt1.y = cvRound(y0 + 1000*(a));
  pt2.x = cvRound(x0 - 1000*(-b));
  pt2.y = cvRound(y0 - 1000*(a));
  line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
}*/
  vector<Vec4i> lines;
  HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
 
 
 
 
 
 
 /*
		
	// define a polygon (as a vector of points)

	vector<Point> contour;
	contour.push_back(Point(l[0], l[1]));
	contour.push_back(Point(l[2], l[3]));
	

	// create a pointer to the data as an array of points (via a conversion to 
	// a Mat() object)

	const cv::Point *pts = (const cv::Point*) Mat(contour).data;
	int npts = Mat(contour).rows;

	std::cout << "Number of polygon vertices: " << npts << std::endl;
	
	// draw the polygon 

	polylines(cdst, &pts,&npts, 1,
	    		true, 			// draw closed contour (i.e. joint end to start) 
	            Scalar(0,255,0),// colour RGB ordering (here = green) 
	    		3, 		        // line thickness
			    CV_AA, 0);
			    
						    
	 imshow("Sobel+Canny+Houasdasdadgh Trasformation", cdst);	  */  
  }
 
 imshow("Sobel+Canny+Hough Trasformation", cdst);
 
  



 
 
 
 
 
 
 
 
 

			    
	    
			    
			    
 
 
 
 
 
 
 

 
 Mat cdst1,dst1;
Canny(src, dst1, 50, 200, 3);
 cvtColor(dst1, cdst1, CV_GRAY2BGR);
 /*
vector<Vec2f> lines1;
HoughLines(dst1, lines1, 1, CV_PI/180, 100, 0, 0 );

for( size_t i1 = 0; i1 < lines1.size(); i1++ )
{
  float rho1 = lines1[i1][0], theta1 = lines1[i1][1];
  Point pt11, pt21;
  double a1 = cos(theta1), b1 = sin(theta1);
  double x01 = a1*rho1, y01 = b1*rho1;
  pt11.x = cvRound(x01 + 1000*(-b1));
  pt11.y = cvRound(y01 + 1000*(a1));
  pt21.x = cvRound(x01 - 1000*(-b1));
  pt21.y = cvRound(y01 - 1000*(a1));
  line( cdst1, pt11, pt21, Scalar(0,0,255), 3, CV_AA);
}
 
*/ 
 
  vector<Vec4i> lines1;
  HoughLinesP(dst1, lines1, 1, CV_PI/180, 50, 50, 10 );
  for( size_t i1 = 0; i1 < lines1.size(); i1++ )
  {
    Vec4i l1 = lines1[i1];
    line( cdst1, Point(l1[0], l1[1]), Point(l1[2], l1[3]), Scalar(0,0,255), 3, CV_AA);
  }
 
 
 
 imshow("Canny+Hough Trasformation", cdst1);
 
 
 
 
 
 
 
 
 
 
 

 
 
 
 

	 
		 

		 // View		
		 imshow("Input", inputImg);
		 imshow("Output", outputImg);
		 waitKey(1);
		 
	}

	return 0;
	
	
	
	
	
	
 
 
        
		
	/*	cvtColor(inputImg, inputImgGray, CV_BGR2GRAY);
		
		imshow("My video", inputImgGray);
		
		
		//imshow("My video", inputImg);




		if (waitKey(30) == 27) // wait for 'esc' key press form 30 ms
		{
			cout << "ESC key pressed by user"<< endl;
			break;
		}*/
	}
  //  }
    
  
  
  
  
 
  
  
  
  
  


































