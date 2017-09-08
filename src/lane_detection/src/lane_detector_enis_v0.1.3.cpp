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













#include<iostream>
#include<iomanip>
#include<cmath>





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
	
	
	
	
	
	
//	cap.set(CV_CAP_PROP_FPS,30);		//change the frame value

	
	
	
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
		


 
  
 
 
 
 	// Main loop
	int frameNum = 0;
	for( ; ; )
	{
		printf("FRAME #%6d ", frameNum);
		fflush(stdout);
		frameNum++;









bool bSuccess = cap.read(inputImg);










 


 
 // The 4-points at the input image	
	vector<Point2f> origPoints;
	origPoints.push_back( Point2f(0, dHeight) );
	origPoints.push_back( Point2f(dWidth, dHeight) );
	origPoints.push_back( Point2f(dWidth, 80) );
	origPoints.push_back( Point2f(0, 80) );
  	 
	// The 4-points correspondences in the destination image
	vector<Point2f> dstPoints;
	dstPoints.push_back( Point2f(dWidth/2-50, dHeight) );
	dstPoints.push_back( Point2f(dWidth/2+50, dHeight) );
	dstPoints.push_back( Point2f(dWidth, 0) );
	dstPoints.push_back( Point2f(0, 0));
	
	// IPM object
	
	 
	 
	IPM ipm( Size(dWidth, dHeight), Size(dWidth, dHeight), origPoints, dstPoints );
	
	
	
	

	
	
	
	
	
	
	




	
	
		 // Process
		 clock_t begin = clock();
	
	
	
	
	
		 
		 
	
		 ipm.applyHomography( inputImg, outputImg );	

	 	 
		 clock_t end = clock();
		 double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		 printf("%.2f (ms)\r", 1000*elapsed_secs);
	//	 cout << 1000*elapsed_secs <<endl;
		
		
	
		
		  
		 ipm.drawPoints(origPoints, inputImg );
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
	
	
	
		
		
		
		Vec3b intensity = outputImg.at<Vec3b>(240, 136);
uchar blue = intensity.val[0];
uchar green = intensity.val[1];
uchar red = intensity.val[2];

 
		 
 
	
	vector<Point> vertices{Point(0,0), Point(270,480), Point(0,480)};
	vector<vector<Point>> pts{vertices};
//	fillPoly(outputImg, pts, Scalar(100,100,100,0));
	fillPoly(outputImg, pts, Scalar(blue,green,red,0));
	
	
	
	
	
	
	
	
	
	
		Vec3b intensity1 = outputImg.at<Vec3b>(240, 504);
uchar blue1 = intensity1.val[0];
uchar green1 = intensity1.val[1];
uchar red1 = intensity1.val[2];


	
	vector<Point> vertices1{Point(640,0), Point(370,480), Point(640,480)};
	vector<vector<Point>> pts1{vertices1};
	//fillPoly(outputImg, pts1, Scalar(100,100,100,0));	
	fillPoly(outputImg, pts1, Scalar(blue1,green1,red1,0));	 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 // View		
		
		 
		 
		 
		 
		  imshow("Input", inputImg);
		  imshow("Output", outputImg);	
		 




		 
		 
		 

	
		 
//Sobel Operator defined		
	
	 
Mat grad;
int scale = 1;
int delta = 0;
int ddepth = CV_16S;		 
		 
GaussianBlur( outputImg, outputImg, Size(13,13), 0, 0, BORDER_DEFAULT );	//GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );		 
		 
//cvtColor( outputImg, inputImgGray, CV_BGR2GRAY );		 // Bunu sil
		 
		 
Mat grad_x, grad_y;
Mat abs_grad_x, abs_grad_y;

/// Gradient X
Sobel( outputImg, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );   //inputImgGray i src ile değiştir
/// Gradient Y
Sobel( outputImg, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );	//inputImgGray i src ile değiştir
		 
		 
convertScaleAbs( grad_x, abs_grad_x );
convertScaleAbs( grad_y, abs_grad_y );		 
		 
addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );		 
		 
imshow( "Sobel", grad );		 
		 
		 
		 




		 
	

 double x[10000],y[10000];






Mat cdst,dst;
Canny(grad, dst, 50, 200, 3);
cvtColor(dst, cdst, CV_GRAY2BGR);
 
 
				
  vector<Vec4i> lines;
  HoughLinesP(dst, lines, 1, CV_PI/180, 10, 10, 100 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];

 //   line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
   
   
   circle( cdst, Point(l[0], l[1]), 1, Scalar(0,0,255), 1, CV_AA, 0);
   circle( cdst, Point(l[2], l[3]), 1, Scalar(0,0,255), 1, CV_AA, 0);
   
   
   
   
   
  
  //Neden calismisyor ROI?
   
   
int yy;;
double xx; 
 for( yy=0;yy<480;yy++)
 {
	 xx=(270./480.)*yy+1;
	 if(yy==l[1] && l[0] <= xx){
	 x[i]=x[i-1];
	 y[i]=y[i-1];
   }
   
   
   
    xx=640-(270./480.)*yy-1;
 //   cout<<xx << " degeri " << yy << endl; 
	 if(yy==l[1] && l[0] >= xx){
	 x[i]=x[i-1];
	 y[i]=y[i-1];
	 cout<<"Buldum"<<endl;
   }
   
   
  }
    
   //Neden calismisyor ROI?
   
      x[i]=l[0];
    y[i]=l[1];
 //  x[i+lines.size()]=l[2];
 //   y[i+lines.size()]=l[3];
   
   
   
   
    }
 
 //imshow("Sobel+Canny+Hough Trasformation", cdst);
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   int i,j,k,n,N;
   cout.precision(4);  						//set precision
   cout.setf(ios::fixed);
   N = lines.size();
/*   double x[N],y[N];
   for(i=0;i<N;i++)
   x[i]=l[0];
    for(i=0;i<N;i++)
    y[i]=l[1];   */
    n = 2;   								//polinomun derecesi
    double X[2*n+1];
    for(i=0;i<2*n+1;i++)
    {
		X[i]=0;
		for(j=0;j<N;j++)
		X[i]=X[i]+pow(x[j],i);
	}
	double B[n+1][n+2],a[n+1];
	for(i=0;i<=n;i++)
		for(j=0;j<=n;j++)
		B[i][j]=X[i+j];
	double Y[n+1];
	for(i=0;i<n+1;i++)
	{
		Y[i]=0;
		for(j=0;j<N;j++)
		Y[i]=Y[i]+pow(x[j],i)*y[j];
	}
	for(i=0;i<=n;i++)
	B[i][n+1]=Y[i];
	n=n+1;
/*	cout<<"\nThe Normal (Augmented Matrix) is as follows:\n";
	for(i=0;i<n;i++)
	{
		for(j=0;j<=n;j++)
		cout<<B[i][j]<<setw(16);
		cout<<"\n";
	}
*/	for(i=0;i<n;i++)
	for(k=i+1;k<n;k++)
	if(B[i][i]<B[k][i])
	for(j=0;j<=n;j++)
	{
		double temp=B[i][j];
		B[i][j]=B[k][j];
		B[k][j]=temp;
	}
	for(i=0;i<n-1;i++)
	for(k=i+1;k<n;k++)
	{
		double t=B[k][i]/B[i][i];
		for(j=0;j<=n;j++)
		B[k][j]=B[k][j]-t*B[i][j];
	}
	for(i=n-1;i>=0;i--)
	{
		a[i]=B[i][n];
		for(j=0;j<n;j++)
		if(j!=i)
		a[i]=a[i]-B[i][j]*a[j];
		a[i]=a[i]/B[i][i];
	}
	cout<<"\nThe values of the cofficients are as follows:\n";
	for(i=0;i<n;i++)
	cout << "x´"<<i<<"="<<a[i]<<endl;
	cout<<"\nHence the fitted Polynomial is given by:\ny=";
	for(i=0;i<n;i++)
	cout<<" + ("<<a[i]<<")"<<"x´"<<i;
	cout<<"\n";

	
		
   
   
   
   
   
   
   

//Fonksiyonun mavi cizgi ile gösterimi  
   
   int p,r;
   for(p=0;p<480;p++)
   {
	   r=a[0] + a[1] * p + a[2] * p * p;
	   if(r>=0 && r<=640)
//	   cout<<p<< " degeri " <<r<<endl;
	   circle( cdst, Point(p, r), 1, Scalar(0,255,0), 1, CV_AA, 0);
	   }
   
   
  
   
   
   
   
   
   imshow("Sobel+Canny+Hough Trasformation", cdst);
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
 
 
  





	
		 waitKey(1);
		 
	}

	return 0;
	
	

	
	}
  //  }
    
  
  
  
  
 
  
  
  
  
  


































