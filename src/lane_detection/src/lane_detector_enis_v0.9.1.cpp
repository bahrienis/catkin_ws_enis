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
#include <stdio.h>
#include <ctime>
#include "opencv2/imgcodecs.hpp"





#include "IPM.h"
#include "IPM.cpp"





using namespace std;
using namespace cv;


void curvefitting(int numofpointsoffirstlane2, int numofpointsoffirstlane1,
        double *y, double *x, Mat inputImg, string color, IPM ipm);


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
    int widthofframe = 640;
    int heightofframe = 480;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
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
        Mat outputImg3;
        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Mat grad_x, grad_y;
        Mat abs_grad_x, abs_grad_y;
        Mat cdst, dst, gradcolor;
        Mat upOutputImggray, downOutputImggray;
        Mat gradSmall,gradcolorSmall;
        
        
        
        int firstpicsize = 100;
        int secondpicsize = 50;
        int thirdpicsize = 330;
        
        vector<int> redlines;
        vector<Point> houghpoints;
        int rightLaneIndex = 0;
        int rightLaneIndexStart = 0;
        int rightLaneIndexFinish = 0;
        int middleLaneIndex = 0;
        int middleLaneIndexStart = 0;
        int middleLaneIndexFinish = 0;
        int leftLaneIndexStart = 0;
        int leftLaneIndexFinish = 0;
        Point rightLaneStart = Point(0,0);
        Point middleLaneStart = Point(0,0);
        Point leftLaneStart = Point(0,0);
        
        vector<Point> rightLanePoints;
        

        clock_t begin = clock();


       
   /*     if (sayi >= 294) {
            sayi = 0;
        }

        std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame" + std::to_string(sayi) + ".jpg";
        sayi++;
        cout << "frame : " << sayi << endl;
*/
        
        
        
//std::string filename = "/home/enis/Desktop/Masterarbeit/photos_31.08.2017_geradeaus/frame187.jpg";
           //  std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame9.jpg";
      std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame152.jpg";
     
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

        
        
   
        
        
        
        
        
        
        /*
        Mat afterIPM;
     ipm.applyHomography( inputImg, afterIPM );
 
     */
     
     
     
     
        
        

        
        
      
        GaussianBlur(inputImg, outputImg2, Size(3, 3), 0, 0, BORDER_DEFAULT); //GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );		 

        
        cvtColor(outputImg2, outputImg3, CV_BGR2GRAY); // Bunu sil

        
        minMaxLoc(outputImg3, &minVal, &maxVal, &minLoc, &maxLoc);

        cout << "min val : " << minVal << endl;
        cout << "max val: " << maxVal << endl;

        
        
        
        
        threshold(outputImg3, cdst1, 0.5*maxVal, 255, 1);
        
        
        
        
        
        

        /// Gradient X
        Sobel(cdst1, grad_x, ddepth, 1, 0, 1, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir
        /// Gradient Y
        Sobel(cdst1, grad_y, ddepth, 0, 1, 1, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir

        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
        
        cvtColor(grad, gradcolor, CV_GRAY2BGR);
        
        
        
        
        
        
        
        
        
        
        
                
          Rect Rec1(0, firstpicsize , 640, (480 - firstpicsize));
//        line(inputImg, Point(0,99),Point(640,99),Scalar(0,255,0),1,CV_AA);
     //   rectangle(inputImg, Rec1, Scalar(255), 1, 8, 0);
    //    rectangle(inputImg, Point(0, 0),Point(640, 99), Scalar(255), 1, 8, 0);
        

        gradSmall = grad(Rec1);
        gradcolorSmall = gradcolor(Rec1);
        
        
        
        
        
        
        
        
                   vector<Vec2f> lines1;
        HoughLines(gradSmall, lines1, 2, CV_PI, 4, 0, 0);
 
        for (size_t i = 0; i < lines1.size(); i++) {
            float rho = lines1[i][0], theta = lines1[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(gradcolorSmall, pt1, pt2, Scalar(0, 0, 150), 1, CV_AA);
        }
        
        
        
        
        //redlines.push_back(widthofframe);
        for(int i=widthofframe-1;i>=0;i--){
              Vec3b intensity = gradcolorSmall.at<Vec3b>(10, i);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];

            if (red >= 100 && red <= 200) {
                redlines.push_back(i);

            }
            
           
            
        }
        
        
        for(int i=0;i<redlines.size();i++){
            if(i<=redlines.size()-2){
             if(redlines.at(i) - redlines.at(i+1) >=50){
                 if(rightLaneIndexFinish == 0){
               rightLaneIndexFinish = redlines.at(i);
               middleLaneIndexStart = redlines.at(i+1);
                 }
                 else{
                middleLaneIndexFinish = redlines.at(i);
                leftLaneIndexStart = redlines.at(i+1);
                 }
            }
            }
        }
        
        
        
        
        
        
        cout << "right Lane Index Finish : " << rightLaneIndexFinish << endl;
        cout << "middle Lane Index Start: " << middleLaneIndexStart << endl;
        cout << "middle Lane Index Finish: " << middleLaneIndexFinish << endl; 
        cout << "left Lane Index Start : " << leftLaneIndexStart << endl;
        
        
        
                                //1. Bild         

        vector<Vec4i> lines1P;
        HoughLinesP(grad, lines1P, 1, CV_PI / 180, 0, 0, 0);
        for (size_t i = 0; i < lines1P.size(); i++) {
            Vec4i l = lines1P[i];
            circle(inputImg, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
          //   circle(inputImg, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
                circle(inputImg, Point(l[2], l[3]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
                houghpoints.push_back(Point(l[0], l[1]));
                houghpoints.push_back(Point(l[2], l[3]));

         
  }
        
        
        
        
        
        for(int i=0;i<houghpoints.size();i++){
            if(houghpoints.at(i).x>=rightLaneIndexFinish && houghpoints.at(i).y > rightLaneStart.y){
                rightLaneStart = houghpoints.at(i);
            }
            
            if((houghpoints.at(i).x<=middleLaneIndexStart && houghpoints.at(i).x >= middleLaneIndexFinish) && houghpoints.at(i).y > middleLaneStart.y){
                middleLaneStart = houghpoints.at(i);
            }
            
            if(houghpoints.at(i).x <= leftLaneIndexStart && houghpoints.at(i).y > leftLaneStart.y){
                leftLaneStart = houghpoints.at(i);
            }
        }
        
        cout << "results right : " << rightLaneStart << endl;
        cout << "results middle : " << middleLaneStart << endl;
        cout << "results left : " << leftLaneStart << endl;
        
        
        
        
  /*      int right_x = rightLaneStart.x;
        int right_y = rightLaneStart.y;
        
     
        
       for(int i=0;i<houghpoints.size();i++){ 
           
       if((houghpoints.at(i).x>=rightLaneStart.x-right_y/10 && houghpoints.at(i).x<=rightLaneStart.x+right_y/10) && (houghpoints.at(i).y<=rightLaneStart.y+right_y/10 && houghpoints.at(i).y>= rightLaneStart.y-right_y/10)){ 
           rightLanePoints.push_back(houghpoints.at(i));
           rectangle(inputImg, Point(rightLaneStart.x-right_y/10, rightLaneStart.y+right_y/10),Point(rightLaneStart.x+right_y/10, rightLaneStart.y-right_y/10), Scalar(255), 1, 8, 0);
           
       }
       
       
       
       if(rightLanePoints.size()>=5 && i == houghpoints.size()-1){
           
          
           int temp = rightLanePoints.at(0).y;
           for(int j=0;j<rightLanePoints.size();j++){
               
               if(rightLanePoints.at(j).y< temp){
                   rightLaneStart = rightLanePoints.at(j);
                  
               }
           }
            cout << "deneme : " << rightLaneStart << endl;
           i=0;
            right_x = rightLaneStart.x;
         right_y = rightLaneStart.y;
           rightLanePoints.clear();
       }
        
       } 
       
       */ 
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        int x_value = rightLaneStart.x;
        int y_value = rightLaneStart.y;
   

      for(int i=0;i<houghpoints.size();i++) { 
         int x_coord = houghpoints.at(i).x;
         int  y_coord = houghpoints.at(i).y;
          
         if(x_coord>=x_value-y_value/8 && x_coord <= x_value+y_value/8
                 && y_coord < y_value && y_coord >= y_value-y_value/5){
        
  rectangle(inputImg,Point(x_value-y_value/8,y_value),Point(x_value+y_value/8,y_value-y_value/5),Scalar(255), 1, 8, 0);
  
    rightLanePoints.push_back(houghpoints.at(i));
  
      }
         
         if(rightLanePoints.size()>0 && i == houghpoints.size()-1){
             
             y_value = rightLanePoints.at(0).y;
             
             for(int j=1;j<rightLanePoints.size();j++){
                 if(rightLanePoints.at(j).y < y_value){
                   y_value = rightLanePoints.at(j).y;
                   x_value = rightLanePoints.at(j).x;
                   
                 }
             }
 
             rightLanePoints.clear();
             i=0; 
                      
         }
         if(rightLanePoints.size()==1 && i == houghpoints.size()-1){
             
            i =  houghpoints.size();
         }
  
      }
  
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        

        imshow("Input Image", inputImg);
        imshow("Sobel", grad);
        imshow("Sobel color", gradcolor);

      //  Canny(grad, dst, 255, 255, 3);
//        cvtColor(dst, cdst, CV_GRAY2BGR);

          waitKey(1);

}
}