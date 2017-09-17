// Autor : Bahri Enis Demirtel
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/flann/flann_base.hpp"
#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "opencv2/core/core.hpp"
#include <cmath> 
#include <ctime>
#include <vector>
#include "opencv2/imgcodecs.hpp"





#include "IPM.h"
#include "IPM.cpp"







using namespace std;
using namespace cv;


void curvefitting(int numofpointsoffirstlane2, int numofpointsoffirstlane1,
        vector<double> y, vector<double> x, Mat inputImg, string color);


int main(int argc, char **argv) {

    //SetUP ROS.
    ros::init(argc, argv, "lane_detector_enis");

    //CV_CAP_ANY == 0 yazınca 2. kamera açılır.

    VideoCapture cap(CV_CAP_ANY); // OPENT THE VIDEO CAMERO NO. 0

    cap.set(CV_CAP_PROP_FPS, 30); //change the frame value

    if (!cap.isOpened()) //if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
    }

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of te video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    double dFrame = cap.get(CV_CAP_PROP_FPS);

    cout << "FPS value is " << dFrame << endl;
    cout << "Frame size: " << dWidth << " x " << dHeight << endl;

   // namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"


    int sayi = 0;
    
    
    
     
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
        
        
        IPM backward_ipm( Size(dWidth, dHeight), Size(dWidth, dHeight), dstPoints, origPoints);
        
        

    // Main loop
    int frameNum = 0;
    for (;;) {


        Mat inputImg;
        Mat inputImgGray;
        Mat outputImg;
        Mat outputImg1;
        Mat outputImg2;
        Mat cdst1;
        Mat grad;
        Mat gradgray;
        Mat outputImg3;
        Mat outputImg4;
        Mat outputImg4gray;
        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Mat grad_x, grad_y;
        Mat abs_grad_x, abs_grad_y;
        Mat cdst, dst;
        Mat upOutputImggray, downOutputImggray;
        
        
        
        
        std::vector<Point2f> alldetectedpoints;
        
        double x1[10000], y1[10000];
        double x2[10000], y2[10000];
        double x3[10000], y3[10000];
        int numOfRedLinesUp = 0;
        int findredlinesarray1[640];
        
        
        
        int firstpicsize = 100;
        int secondpicsize = 150;
        int thirdpicsize = 230;
        

        clock_t begin = clock();


       
       if (sayi >= 294) {
            sayi = 0;
        }

        std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame" + std::to_string(sayi) + ".jpg";
        sayi++;
        cout << "frame : " << sayi << endl;

        
        
        
//std::string filename = "/home/enis/Desktop/Masterarbeit/photos_31.08.2017_geradeaus/frame187.jpg";
           //  std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame9.jpg";
//       std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame200.jpg";
     
        //std::string filename = "/home/enis/Desktop/Masterarbeit/deneme2/frame12.jpg";
        //std::string filename = "/home/enis/Desktop/Masterarbeit/frame0058.jpg";
        //std::string filename = "/home/enis/Desktop/frame12.jpg"; 
        //std::string filename = "/home/enis/Desktop/Masterarbeit/photos_31.08.2017_lighton/frame97.jpg";


       inputImg = imread(filename, CV_LOAD_IMAGE_COLOR);
        if (inputImg.empty()) {
            cout << "can not open " << filename << endl;
            return -1;
        }
        
        printf("FRAME #%6d ", frameNum);
        fflush(stdout);
        frameNum++;

        
//        bool bSuccess = cap.read(inputImg);

        
         
	
		 ipm.applyHomography( inputImg, outputImg );	
                 ipm.drawPoints(origPoints, inputImg );
                 
                 
                 imshow("Input", inputImg); 
                 imshow("Output",outputImg);
                 
                 
                 
                 
                 
             GaussianBlur(outputImg, outputImg1, Size(3, 3), 0, 0, BORDER_DEFAULT); //GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );		 

        cvtColor(outputImg1, outputImg2, CV_BGR2GRAY); // Bunu sil

        minMaxLoc(outputImg2, &minVal, &maxVal, &minLoc, &maxLoc);

        cout << "min val : " << minVal << endl;
        cout << "max val: " << maxVal << endl;

        
        
        
        
        threshold(outputImg2, cdst1, 0.6*maxVal, 255, 1);
        
        
        
        
        
        

        /// Gradient X
        Sobel(cdst1, grad_x, ddepth, 1, 0, 1, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir
        /// Gradient Y
        Sobel(cdst1, grad_y, ddepth, 0, 1, 1, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir

        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);



      //  Canny(grad, dst, 255, 255, 3);
        cvtColor(grad, gradgray, CV_GRAY2BGR);


       
 //       rectangle(dst, Rec4, Scalar(255), 1, 8, 0);


       
                 
                 
                 
                 
                 
         Rect Rec1(0, 320 , 640, 160);

        outputImg4 = grad(Rec1);         
        cvtColor(outputImg4, outputImg4gray, CV_GRAY2BGR);      
                 
                 
                vector<Vec2f> lines1;
        HoughLines(outputImg4, lines1, 2, CV_PI, 2, 0, 0);
        
          for (size_t i = 0; i < lines1.size(); i++) {
            float rho = lines1[i][0], theta = lines1[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(outputImg4gray, pt1, pt2, Scalar(0, 0, 150), 1, CV_AA);
        }  
                 
        
        
        
        
        
        
           for (int i = 639; i >= 0; i--) {
            Vec3b intensity1 = outputImg4gray.at<Vec3b>(10, i);
            uchar blue1 = intensity1.val[0];
            uchar green1 = intensity1.val[1];
            uchar red1 = intensity1.val[2];

            if (red1 >= 100 && red1 <= 200) {
              findredlinesarray1[numOfRedLinesUp] = i;
                numOfRedLinesUp++;
            }
        }
        
        
        
        int rightLineIndex = 0;
        int midLineIndex = 0;
        int leftLineIndex = 0;
                       
        for(int i=0;i<numOfRedLinesUp;i++){
            if(findredlinesarray1[i] - findredlinesarray1[i + 1] > 25){
                midLineIndex = i+1;
                break;
            }
        }
        if(midLineIndex>0){
            for(int i=midLineIndex;i<numOfRedLinesUp;i++){
               if(findredlinesarray1[i] - findredlinesarray1[i + 1] > 25){
                leftLineIndex = i+1;
                break;
               } 
            }
        }
        
        cout<< "Right line coord :" << findredlinesarray1[rightLineIndex] << endl;
        cout<< "Middle line coord :" << findredlinesarray1[midLineIndex]<< endl;
        cout<< "Left line coord :" << findredlinesarray1[leftLineIndex]<< endl;
    
        int x_coord_RightLine = findredlinesarray1[rightLineIndex];
        int x_coord_MiddleLine = findredlinesarray1[midLineIndex];
        int x_coord_LeftLine = findredlinesarray1[leftLineIndex];
        
        
        
        
        
        
        
        
        
        
        
        
        vector<Vec4i> lines1P;
        HoughLinesP(grad, lines1P, 2, CV_PI / 180, 3, 3, 3);
        for (size_t i = 0; i < lines1P.size(); i++) {
            Vec4i l = lines1P[i];
            circle(gradgray, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            alldetectedpoints.push_back(Point2f(l[0], l[1]));
        }
        
        
        
        
        
        int max_y_forRightLine =0;
        int max_y_forMiddleLine =0;
        int max_y_forLeftLine =0;
        for(int i=0;i < alldetectedpoints.size();i++){
            int x_coord = alldetectedpoints.at(i).x;
            int y_coord = alldetectedpoints.at(i).y;
            if(std::abs(x_coord-x_coord_RightLine)<3 && y_coord>max_y_forRightLine){
                max_y_forRightLine = y_coord;
            }
            if(std::abs(x_coord-x_coord_MiddleLine)<3 && y_coord>max_y_forMiddleLine){
                max_y_forMiddleLine = y_coord;
            }
            if(std::abs(x_coord-x_coord_LeftLine)<3 && y_coord>max_y_forLeftLine){
                max_y_forLeftLine = y_coord;
            }
        }
        
        cout << "Right Line starting point: " << Point(x_coord_RightLine,max_y_forRightLine) << endl;
        cout << "Middle Line starting point: " << Point(x_coord_MiddleLine,max_y_forMiddleLine) << endl;   
        cout << "Left Line starting point: " << Point(x_coord_LeftLine,max_y_forLeftLine) << endl;
            
        
        
        
        
      
        
            
                
                
        
        
  std::vector<Point> rightLine;     
  
  
        
  int numOfPoints=5;
  int step = 20;
  int current_x = x_coord_RightLine;
  vector<double> x_rightLine;
  vector<double> y_rightLine;
  flann::KDTreeIndexParams indexParams;
  flann::Index kdtree(Mat(alldetectedpoints).reshape(1), indexParams);
  for (int current_y = max_y_forRightLine; current_y>0; current_y-=step){
     
      
     std::vector<float> query;
     query.push_back(current_x);
     query.push_back(current_y);
     vector<int> indices;
     vector<float> dists;
     kdtree.knnSearch(query, indices, dists, numOfPoints);
    
     int x_sum = 0;
     int y_sum = 0;
     int threshold = 80;
     for(int i=0; i<indices.size(); i++){
         x_sum+=alldetectedpoints.at(indices.at(i)).x;
         y_sum+=alldetectedpoints.at(indices.at(i)).y;
     }
     double nextPointX = x_sum/indices.size();
     double nextPointY = y_sum/indices.size();
     
     if(x_rightLine.size()==0 || 
             (x_rightLine.size()>0 && 
             abs(nextPointX-x_rightLine.back())<threshold && 
             abs(nextPointY-y_rightLine.back())<threshold)){
        rightLine.push_back(Point(nextPointX, nextPointY));
        x_rightLine.push_back(nextPointX);
        y_rightLine.push_back(nextPointY);
     }
    
    
     current_x = nextPointX;
  }
  
   
        
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
      std::vector<Point> middleLine;     
  
  
        
  numOfPoints=5;
  step = 20;
  current_x = x_coord_MiddleLine;
  vector<double> x_middleLine;
  vector<double> y_middleLine;
 // flann::KDTreeIndexParams indexParams;
 // flann::Index kdtree(Mat(alldetectedpoints).reshape(1), indexParams);
  for (int current_y = max_y_forMiddleLine; current_y>0; current_y-=step){
     
      
     std::vector<float> query;
     query.push_back(current_x);
     query.push_back(current_y);
     vector<int> indices;
     vector<float> dists;
     kdtree.knnSearch(query, indices, dists, numOfPoints);
    
     int x_sum = 0;
     int y_sum = 0;
     int thresholdMiddleLine = 500;
     for(int i=0; i<indices.size(); i++){
         x_sum+=alldetectedpoints.at(indices.at(i)).x;
         y_sum+=alldetectedpoints.at(indices.at(i)).y;
     }
     double nextPointX = x_sum/indices.size();
     double nextPointY = y_sum/indices.size();
     
     if(x_middleLine.size()==0 || 
             (x_middleLine.size()>0 && 
             abs(nextPointX-x_middleLine.back())<thresholdMiddleLine && 
             abs(nextPointY-y_middleLine.back())<thresholdMiddleLine)){
        middleLine.push_back(Point(nextPointX, nextPointY));
        x_middleLine.push_back(nextPointX);
        y_middleLine.push_back(nextPointY);
     }
    
    
     current_x = nextPointX;
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
        
         curvefitting(0, x_rightLine.size(), y_rightLine, x_rightLine, gradgray, "blue");
         
         
         curvefitting(0, x_middleLine.size(), y_middleLine, x_middleLine, gradgray, "red");
        
    
        Mat tried;
         
         	 backward_ipm.applyHomography( gradgray, tried );	
                 //backward_ipm.drawPoints(origPoints, inputImg );
                 
                 
                 
                 
        
               imshow("try", tried); 
                 
       imshow("Hough", outputImg4gray);          
       imshow("Sobel", gradgray);  
                 

	 	 
	

        
   //     curvefitting(0, numofpointsoffirstlane1new, y11, x11, inputImg, "blue");
   //     curvefitting(0, numofpointsofsecondlane2new, y22, x22, inputImg, "green");
   //     curvefitting(0, numofpointsofthirdlane3new, y33, x33, inputImg, "red");


   
        
        
        
       waitKey(1);

        
        
        
        
        
        
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * elapsed_secs);
        cout << 1000 * elapsed_secs << endl;
    }
    return 0;
}







void curvefitting(int numofpointsoffirstlane2, int numofpointsoffirstlane1,
        vector<double> x, vector<double> y, Mat inputImg, string color) {
    int i, j, k, n, N;
    cout.precision(4); //set precision
    cout.setf(ios::fixed);
    N = numofpointsoffirstlane2 + numofpointsoffirstlane1; //lines.size();
    /*   double x[N],y[N];
       for(i=0;i<N;i++)
       x[i]=l[0];
        for(i=0;i<N;i++)
        y[i]=l[1];   */
    
    n = 2; //polinomun derecesi
    double X[2 * n + 1];
    for (i = 0; i < 2 * n + 1; i++) {
        X[i] = 0;
        for (j = 0; j < N; j++)
            X[i] = X[i] + pow(x.at(j), i);
    }
    double B[n + 1][n + 2], a[n + 1];
    for (i = 0; i <= n; i++)
        for (j = 0; j <= n; j++)
            B[i][j] = X[i + j];
    double Y[n + 1];
    for (i = 0; i < n + 1; i++) {
        Y[i] = 0;
        for (j = 0; j < N; j++)
            Y[i] = Y[i] + pow(x.at(j), i) * y.at(j);
    }
    for (i = 0; i <= n; i++)
        B[i][n + 1] = Y[i];
    n = n + 1;
    /*	cout<<"\nThe Normal (Augmented Matrix) is as follows:\n";
            for(i=0;i<n;i++)
            {
                    for(j=0;j<=n;j++)
                    cout<<B[i][j]<<setw(16);
                    cout<<"\n";
            }
     */ for (i = 0; i < n; i++)
        for (k = i + 1; k < n; k++)
            if (B[i][i] < B[k][i])
                for (j = 0; j <= n; j++) {
                    double temp = B[i][j];
                    B[i][j] = B[k][j];
                    B[k][j] = temp;
                }
    for (i = 0; i < n - 1; i++)
        for (k = i + 1; k < n; k++) {
            double t = B[k][i] / B[i][i];
            for (j = 0; j <= n; j++)
                B[k][j] = B[k][j] - t * B[i][j];
        }
    for (i = n - 1; i >= 0; i--) {
        a[i] = B[i][n];
        for (j = 0; j < n; j++)
            if (j != i)
                a[i] = a[i] - B[i][j] * a[j];
        a[i] = a[i] / B[i][i];
    }
    cout << "\nThe values of the cofficients are as follows:\n";
    for (i = 0; i < n; i++)
        cout << "x´" << i << "=" << a[i] << endl;
    cout << "\nHence the fitted Polynomial is given by:\ny=";
    for (i = 0; i < n; i++)
        cout << " + (" << a[i] << ")" << "x´" << i;
    cout << "\n";


    int p, r;
    for (p = 0; p < 480; p++) {
        r = a[0] + a[1] * p + a[2] * p * p;
        if (r >= 0 && r <= 640) {
            if (color == "red")
                circle(inputImg, Point(r, p), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            else if (color == "blue")
                circle(inputImg, Point(r, p), 1, Scalar(255, 0, 0), 1, CV_AA, 0);
            else if (color == "green")
                circle(inputImg, Point(r, p), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
        }
    }


} 
