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
























int main(int argc, char **argv) {





    int count = 0;

















    //SetUP ROS.
    ros::init(argc, argv, "lane_detector_enis");












    //Get and show an image	 



    //const char* filename = argc >= 2 ? argv[1] : "/home/enis/Schreibtisch/deneme2/frame" + std::to_string(count) + ".jpg";
    //const char* filename = argc >= 2 ? argv[1] : "/home/enis/Schreibtisch/deneme2/frame22.jpg";







    //imshow( "Display window", src );	



























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


    namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"













    //Mat inputImg;
    Mat inputImgGray;
    Mat outputImg;


    cv::VideoCapture video;





    int sayi = 0;


    // Main loop
    int frameNum = 0;
    for (;;) {


        clock_t begin = clock();










/*

        if (sayi >= 84) {
            sayi = 0;
        }

        std::string filename = "/home/enis/Schreibtisch/deneme2/frame" + std::to_string(sayi) + ".jpg";
        sayi++;

*/
 std::string filename = "/home/enis/Schreibtisch/frame0058.jpg";



        Mat inputImg = imread(filename, CV_LOAD_IMAGE_COLOR);
        if (inputImg.empty()) {
            cout << "can not open " << filename << endl;
            return -1;
        }














        printf("FRAME #%6d ", frameNum);
        fflush(stdout);
        frameNum++;

        //bool bSuccess = cap.read(inputImg);





        Mat outputImg2;


        Rect Rec(0, 300, 640, 180);
        rectangle(inputImg, Rec, Scalar(255), 1, 8, 0);


        Mat outputImg1 = inputImg(Rec);




        
        
        
        
        
        
        
        //Mat outputImg5;
        
        
         Rect Rec1(0, 120, 640, 180);
        rectangle(inputImg, Rec1, Scalar(255), 1, 8, 0);


        Mat outputImg5 = inputImg(Rec1);




        //inRange(outputImg1, Scalar(150,150,150),Scalar(255,255,255),outputImg2);










        
        
        





        //Sobel Operator defined		

        Mat cdst1;
        Mat grad;
        Mat outputImg3;
        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;

        GaussianBlur(outputImg1, outputImg2, Size(3, 3), 0, 0, BORDER_DEFAULT); //GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );		 

        cvtColor(outputImg2, outputImg3, CV_BGR2GRAY); // Bunu sil












        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;

        minMaxLoc(outputImg3, &minVal, &maxVal, &minLoc, &maxLoc);

        cout << "min val : " << minVal << endl;
        cout << "max val: " << maxVal << endl;














        threshold(outputImg3, cdst1, 0.6 * maxVal, 255, 1);





        Mat grad_x, grad_y;
        Mat abs_grad_x, abs_grad_y;

        /// Gradient X
        Sobel(cdst1, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir
        /// Gradient Y
        Sobel(cdst1, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir


        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

        imshow("Sobel", grad);













        Mat cdst, dst;
        Canny(grad, dst, 255, 255, 3);
        cvtColor(dst, cdst, CV_GRAY2BGR);





        //threshold(cdst,cdst1, 200, 255, 1 );
        //imshow("Input", cdst1);





        /*
				
         vector<Vec4i> lines;
         HoughLinesP(dst, lines, 1, CV_PI/180, 10, 10, 80 );
         for( size_t i = 0; i < lines.size(); i++ )
         {
           Vec4i l = lines[i];

           line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
   
         }
  
         */


        vector<Vec2f> lines;
        HoughLines(dst, lines, 4, CV_PI, 10, 0, 0);

        for (size_t i = 0; i < lines.size(); i++) {
            float rho = lines[i][0], theta = lines[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(cdst, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);




        }


        imshow("Sobel+Canny+Hough Trasformation", cdst);










        imshow("Input", inputImg);
        imshow("Output", outputImg1);
        imshow("Output2", outputImg2);
        imshow("Output5", outputImg5);
        waitKey(1);







        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * elapsed_secs);
        cout << 1000 * elapsed_secs << endl;





    }

    return 0;




}







