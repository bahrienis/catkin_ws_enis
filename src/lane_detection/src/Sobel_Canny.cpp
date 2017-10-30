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


//Hough Transformation + Rectangle + Curve Fitting + IPM


using namespace std;




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





     // Main loop
    int frameNum = 0;
    for (;;) {


        Mat inputImg;
        Mat inputImgIPM;
        Mat inputImgGray;
        Mat outputImg;
        Mat outputImg1;
        Mat outputImg2;
        Mat cdst1,cdst2;
        Mat grad,gradgray;
        Mat outputImg3;
        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;
        double minVal,minVal1;
        double maxVal,maxVal1;
        Point minLoc,minLoc1;
        Point maxLoc,maxLoc1;
        Mat grad_x, grad_y;
        Mat abs_grad_x, abs_grad_y;
        Mat cdst, dst, gradcolor;
        Mat upOutputImggray, downOutputImggray;
        Mat gradSmall, gradcolorSmall;





 /*       if (sayi >= 1520) {
            sayi = 0;
        }

        std::string filename = "/home/enis/Desktop/Masterarbeit/photos_16.10.2017_lighton/frame" + std::to_string(sayi) + ".jpg";
        sayi++;
        cout << "frame : " << sayi << endl;

*/


        //std::string filename = "/home/enis/Desktop/Masterarbeit/photos_31.08.2017_geradeaus/frame187.jpg";
  //        std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame152.jpg";
      //   std::string filename = "/home/enis/Desktop/Masterarbeit/photos_16.10.2017_lighton/frame1470.jpg";


std::string filename = "/home/enis/Desktop/deneme5/frame0004.jpg";
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


//             bool bSuccess = cap.read(inputImg);







//        imshow("Input", inputImg);
        
        
        

        
        
//        imshow("Output", outputImg);





      GaussianBlur(inputImg, outputImg1, Size(3, 3), 0, 0, BORDER_DEFAULT); //GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );		 

        cvtColor(outputImg1, outputImg2, CV_BGR2GRAY); // Bunu sil




     minMaxLoc(outputImg2, &minVal, &maxVal, &minLoc, &maxLoc);

        cout << "min val : " << minVal << endl;
        cout << "max val: " << maxVal << endl;


        threshold(outputImg2, cdst1, 0.9 * maxVal, 255, 2);
        
        
        
        
        
 
        clock_t begin = clock();

        /// Gradient X
        Sobel(cdst1, grad_x, ddepth, 1, 0, 1, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir
        /// Gradient Y
        Sobel(cdst1, grad_y, ddepth, 0, 1, 1, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir

        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
        
        
        
     



    clock_t end_Sobel = clock();    

          Canny(cdst1, dst, 255, 255, 3);
   //     cvtColor(grad, gradgray, CV_GRAY2BGR);

    clock_t end_Canny = clock();   
    
    
    
    
    
      double Sobel_elapsed_secs = double(end_Sobel - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * Sobel_elapsed_secs);
        cout << "Sobel time : " << 1000 * Sobel_elapsed_secs << endl;
        



  double Canny_elapsed_secs = double(end_Canny - end_Sobel) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * Canny_elapsed_secs);
        cout << "Canny time : " << 1000 * Canny_elapsed_secs << endl;
        



 
        
        imshow("Input",inputImg);
        imshow("Sobel",grad);
        imshow("Canny",dst);
      waitKey(1);
        
        }
        return 0;
        }
