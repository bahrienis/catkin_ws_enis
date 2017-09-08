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
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;




void curvefitting(int numberofpointsoffirstlane2, int numberofpointsoffirstlane1,
        double *x, double *y, Mat inputImg);




void findcurvesvalues(int *findredlinesarray, int a2, Mat outputImg5, Mat outputImg5gray,
        double *x1,double *y1, double *x2, double *y2, double *x3, double *y3,
        int numberofpointsoffirstlane1, int numberofpointsoffirstlane2,
        int numberofpointsofsecondlane1, int numberofpointsofsecondlane2,
        int numberofpointsofthirdlane1, int numberofpointsofthirdlane2
        );







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

    namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"




    //int sayi = 0;

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
        Mat cdst, dst;
        Mat outputImg4gray, outputImg5gray;














        clock_t begin = clock();
        /*
                 if (sayi >= 84) {
                     sayi = 0;
                 }

                 std::string filename = "/home/enis/Schreibtisch/deneme2/frame" + std::to_string(sayi) + ".jpg";
                 sayi++;
         */

        
         //    std::string filename = "/home/enis/Schreibtisch/Masterarbeit/photos_31.08.2017_lightoff/frame79.jpg";

     //           std::string filename = "/home/enis/Schreibtisch/Masterarbeit/deneme2/frame12.jpg";
              std::string filename = "/home/enis/Schreibtisch/Masterarbeit/frame0058.jpg";

               

               inputImg = imread(filename, CV_LOAD_IMAGE_COLOR);
               if (inputImg.empty()) {
                   cout << "can not open " << filename << endl;
                   return -1;
               }
         

        printf("FRAME #%6d ", frameNum);
        fflush(stdout);
        frameNum++;

      //bool bSuccess = cap.read(inputImg);

        Rect Rec1(0, 120, 640, 360);
        rectangle(inputImg, Rec1, Scalar(255), 1, 8, 0);

        outputImg1 = inputImg(Rec1);

        GaussianBlur(outputImg1, outputImg2, Size(3, 3), 0, 0, BORDER_DEFAULT); //GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );		 

        cvtColor(outputImg2, outputImg3, CV_BGR2GRAY); // Bunu sil


        minMaxLoc(outputImg3, &minVal, &maxVal, &minLoc, &maxLoc);

        cout << "min val : " << minVal << endl;
        cout << "max val: " << maxVal << endl;

        threshold(outputImg3, cdst1, 0.6 * maxVal, 255, 1);

        /// Gradient X
        Sobel(cdst1, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir
        /// Gradient Y
        Sobel(cdst1, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT); //inputImgGray i src ile değiştir

        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

        imshow("Sobel", grad);

        Canny(grad, dst, 255, 255, 3);
        //       cvtColor(dst, cdst, CV_GRAY2BGR);

        Rect Rec4(0, 0, 640, 180);
        rectangle(dst, Rec4, Scalar(255), 1, 8, 0);

        Mat outputImg4 = dst(Rec4);
        cvtColor(outputImg4, outputImg4gray, CV_GRAY2BGR);

        /*			
         vector<Vec4i> lines;
         HoughLinesP(dst, lines, 1, CV_PI/180, 10, 10, 80 );
         for( size_t i = 0; i < lines.size(); i++ )
         {
           Vec4i l = lines[i];
           line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
         }
         */


        int a11 = 0;
        int findredlinesarray1[640];

        vector<Vec2f> lines1;
        HoughLines(outputImg4, lines1, 4, CV_PI, 10, 0, 0);

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

        for (int k = 639; k >= 0; k--) {
            Vec3b intensity1 = outputImg4gray.at<Vec3b>(10, k);
            uchar blue1 = intensity1.val[0];
            uchar green1 = intensity1.val[1];
            uchar red1 = intensity1.val[2];

            if (red1 >= 100 && red1 <= 200) {
                findredlinesarray1[a11] = k;
                a11++;
              
            }
        }

        /*     for (int i = 0; i < a1; i++) {
                 cout << "x değerleri : " << findredlinesarray1[i] << endl;
             }
         */

        int thebiggestvalue1 = 0;
        int thebiggestvalue11 = 0;
        int thebiggestvalue12 = 0;
        int thebiggestvalue11position = 0;
        int thebiggestvalue12position = 0;
        int thebiggestvalue11_1_position = 0;
        int thebiggestvalue12_1_position = 0;

        int jj = 3;

        thebiggestvalue11 = findredlinesarray1[jj - 3] - findredlinesarray1[jj - 2];
        thebiggestvalue12 = findredlinesarray1[jj - 2] - findredlinesarray1[jj - 1];

        thebiggestvalue11position = findredlinesarray1[jj - 3];
        thebiggestvalue12position = findredlinesarray1[jj - 2];

        for (jj = 3; jj < a11; jj++) {
            thebiggestvalue1 = findredlinesarray1[jj - 1] - findredlinesarray1[jj];
            if (thebiggestvalue1 >= thebiggestvalue11 || thebiggestvalue1 >= thebiggestvalue12) {
                if (thebiggestvalue1 >= thebiggestvalue11) {
                    thebiggestvalue12 = thebiggestvalue11;
                    thebiggestvalue11 = thebiggestvalue1;
                    thebiggestvalue12position = thebiggestvalue11position;
                    thebiggestvalue11position = findredlinesarray1[jj];
                    thebiggestvalue11_1_position = findredlinesarray1[jj - 1];
                } else {           
                    thebiggestvalue12 = thebiggestvalue1;
                    thebiggestvalue12position = findredlinesarray1[jj];
                    thebiggestvalue12_1_position = findredlinesarray1[jj - 1];
                }
            }
        }
        

        cout << "thebiggestvalue1 : " << thebiggestvalue11 << " thebiggestvalue12 : " << thebiggestvalue12 << endl;
        cout << "the biggest poisiton : " << thebiggestvalue11position << " tsbp : " << thebiggestvalue12position << endl;
        cout << "deneme 1 : " << thebiggestvalue11_1_position << " deneme 2 : " << thebiggestvalue12_1_position << endl;

        double x1[10000], y1[10000];
        double x2[10000], y2[10000];
        double x3[10000], y3[10000];
        int numberofpointsoffirstlane1 = 0;
        int numberofpointsoffirstlane2 = 0;
        int numberofpointsofsecondlane1 = 0;
        int numberofpointsofsecondlane2 = 0;
        int numberofpointsofthirdlane1 = 0;
        int numberofpointsofthirdlane2 = 0;

        //1. Bild         

        vector<Vec4i> lines1P;
        HoughLinesP(outputImg4, lines1P, 1, CV_PI / 180, 10, 10, 10);
        for (size_t i = 0; i < lines1P.size(); i++) {
            Vec4i l = lines1P[i];
            circle(outputImg4gray, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            circle(outputImg4gray, Point(l[2], l[3]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            if (l[0] >= ((thebiggestvalue11position + thebiggestvalue11_1_position) / 2) && l[0] < 639) {
                x1[numberofpointsoffirstlane1] = l[0];
                //    x[i+1] = l[2];
                y1[numberofpointsoffirstlane1] = l[1] + 120;
                numberofpointsoffirstlane1++;
                //  y[i+1] = l[3];

            } else if (l[0] <= ((thebiggestvalue11position + thebiggestvalue11_1_position) / 2) && l[0] >= ((thebiggestvalue12position + thebiggestvalue12_1_position) / 2)) {
                x2[numberofpointsofsecondlane1] = l[0];
                y2[numberofpointsofsecondlane1] = l[1] + 120;
                numberofpointsofsecondlane1++;
            } else if (l[0] > 0 && l[0] <= ((thebiggestvalue12position + thebiggestvalue12_1_position) / 2)) {
                x3[numberofpointsofthirdlane1] = l[0];
                y3[numberofpointsofthirdlane1] = l[1] + 120;
                numberofpointsofthirdlane1++;
            }
        }
        /*
         for(int xxx=0;xxx<numberofpointsofsecondlane1;xxx++)
        {
            cout << "x degeri : "<< x2[xxx] << " y degeri : " << y2[xxx] << endl;
        }
  
         */

        Rect Rec5(0, 180, 640, 180);
        rectangle(dst, Rec5, Scalar(255), 1, 8, 0);

        Mat outputImg5 = dst(Rec5);
        cvtColor(outputImg5, outputImg5gray, CV_GRAY2BGR);

        vector<Vec2f> lines2;
        HoughLines(outputImg5, lines2, 4, CV_PI, 10, 0, 0);

        for (size_t i = 0; i < lines2.size(); i++) {
            float rho = lines2[i][0], theta = lines1[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(outputImg5gray, pt1, pt2, Scalar(0, 0, 150), 1, CV_AA);
        }

        int findredlinesarray2[640];
        int a2 = 0;

        for (int kk = 639; kk >= 0; kk--) {

            Vec3b intensity2 = outputImg5gray.at<Vec3b>(10, kk);
            uchar blue2 = intensity2.val[0];
            uchar green2 = intensity2.val[1];
            uchar red2 = intensity2.val[2];
            if (red2 >= 100 && red2 <= 200) {
                findredlinesarray2[a2] = kk;
                a2++;
            }
        }

        int thebiggestvalue2 = 0;
        int thebiggestvalue21 = 0;
        int thebiggestvalue22 = 0;
        int thebiggestvalue21position = 0;
        int thebiggestvalue22position = 0;
        int thebiggestvalue21_1_position = 0;
        int thebiggestvalue22_1_position = 0;
        int jjj = 3;

        thebiggestvalue21 = findredlinesarray2[jjj - 3] - findredlinesarray2[jjj - 2];
        thebiggestvalue22 = findredlinesarray2[jjj - 2] - findredlinesarray2[jjj - 1];
        thebiggestvalue21position = findredlinesarray2[jjj - 3];
        thebiggestvalue22position = findredlinesarray2[jjj - 2];

        for (jjj = 3; jjj < a2; jjj++) {
            thebiggestvalue2 = findredlinesarray2[jjj - 1] - findredlinesarray2[jjj];
            if (thebiggestvalue2 >= thebiggestvalue21 || thebiggestvalue2 >= thebiggestvalue22) {
                if (thebiggestvalue2 >= thebiggestvalue21) {
                    thebiggestvalue22 = thebiggestvalue21;
                    thebiggestvalue21 = thebiggestvalue2;
                    thebiggestvalue22position = thebiggestvalue21position;
                    thebiggestvalue21position = findredlinesarray2[jjj];
                    thebiggestvalue21_1_position = findredlinesarray2[jjj - 1];
                } else {
                    thebiggestvalue22 = thebiggestvalue2;
                    thebiggestvalue22position = findredlinesarray2[jjj];
                    thebiggestvalue22_1_position = findredlinesarray2[jjj - 1];
                }
            }
        }

        cout << "thebiggestvalue2 : " << thebiggestvalue21 << " thebiggestvalue22 : " << thebiggestvalue22 << endl;
        cout << "the biggest poisiton21 : " << thebiggestvalue21position << " tsbp : " << thebiggestvalue22position << endl;
        cout << "deneme 1 : " << thebiggestvalue21_1_position << " deneme 2 : " << thebiggestvalue22_1_position << endl;

        
        
        
        
        
        
        
        
        //2. bild

        vector<Vec4i> lines2P;
        HoughLinesP(outputImg5, lines2P, 1, CV_PI / 180, 10, 10, 10);
        for (size_t i = 0; i < lines2P.size(); i++) {
            Vec4i l = lines2P[i];
            circle(outputImg5gray, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            circle(outputImg5gray, Point(l[2], l[3]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            if (l[0] >= ((thebiggestvalue21position + thebiggestvalue21_1_position) / 2) && l[0] < 639) {
                x1[numberofpointsoffirstlane2 + numberofpointsoffirstlane1] = l[0];
                //    x[i+1] = l[2];
                y1[numberofpointsoffirstlane2 + numberofpointsoffirstlane1] = l[1] + 300;
                numberofpointsoffirstlane2++;
                //  y[i+1] = l[3];

            } else if (l[0] <= ((thebiggestvalue21position + thebiggestvalue21_1_position) / 2) && l[0] >= ((thebiggestvalue22position + thebiggestvalue22_1_position) / 2)) {
                x2[numberofpointsofsecondlane2 + numberofpointsofsecondlane1] = l[0];
                y2[numberofpointsofsecondlane2 + numberofpointsofsecondlane1] = l[1] + 300;
                numberofpointsofsecondlane2++;

            } else if (l[0] > 0 && l[0] <= ((thebiggestvalue22position + thebiggestvalue22_1_position) / 2)) {
                x3[numberofpointsofthirdlane2 + numberofpointsofthirdlane1] = l[0];
                y3[numberofpointsofthirdlane2 + numberofpointsofthirdlane1] = l[1] + 300;
                numberofpointsofthirdlane2++;
            }
        }

/*
          for (int aaaa = 0; aaaa < (numberofpointsofthirdlane2 + numberofpointsofthirdlane1); aaaa++) {
              cout << "x : " << x3[aaaa] << " y :" << y3[aaaa] << endl;
          }
   */      

        curvefitting(numberofpointsoffirstlane2, numberofpointsoffirstlane1, x1, y1, inputImg);
        curvefitting(numberofpointsofsecondlane2, numberofpointsofsecondlane1, x2, y2, inputImg);
        curvefitting(numberofpointsofthirdlane2, numberofpointsofthirdlane1, x3, y3, inputImg);


        imshow("Sobel+Canny+Hough Trasformation1", outputImg4gray);
        imshow("Sobel+Canny+Hough Trasformation2", outputImg5gray);
        imshow("Input", inputImg);
        imshow("Output2", outputImg2);

        waitKey(1);

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * elapsed_secs);
        cout << 1000 * elapsed_secs << endl;
    }
    return 0;
}













void findcurvesvalues(int *findredlinesarray, int a2, Mat outputImg5, Mat outputImg5gray,
        double *x1,double *y1, double *x2, double *y2, double *x3, double *y3,
        int numberofpointsoffirstlane1, int numberofpointsoffirstlane2,
        int numberofpointsofsecondlane1, int numberofpointsofsecondlane2,
        int numberofpointsofthirdlane1, int numberofpointsofthirdlane2
        ) {




    
    
    
   

        int thebiggestvalue = 0;
        int thebiggestvalue1 = 0;
        int thebiggestvalue2 = 0;
        int thebiggestvalue1position = 0;
        int thebiggestvalue2position = 0;
        int thebiggestvalue1_1_position = 0;
        int thebiggestvalue2_1_position = 0;
        int jjj = 3;

        thebiggestvalue1 = findredlinesarray[jjj - 3] - findredlinesarray[jjj - 2];
        thebiggestvalue2 = findredlinesarray[jjj - 2] - findredlinesarray[jjj - 1];
        thebiggestvalue1position = findredlinesarray[jjj - 3];
        thebiggestvalue2position = findredlinesarray[jjj - 2];

        for (jjj = 3; jjj < a2; jjj++) {
            thebiggestvalue = findredlinesarray[jjj - 1] - findredlinesarray[jjj];
            if (thebiggestvalue >= thebiggestvalue1 || thebiggestvalue >= thebiggestvalue2) {
                if (thebiggestvalue >= thebiggestvalue1) {
                    thebiggestvalue2 = thebiggestvalue1;
                    thebiggestvalue1 = thebiggestvalue;
                    thebiggestvalue2position = thebiggestvalue1position;
                    thebiggestvalue1position = findredlinesarray[jjj];
                    thebiggestvalue1_1_position = findredlinesarray[jjj - 1];
                } else {
                    thebiggestvalue2 = thebiggestvalue;
                    thebiggestvalue2position = findredlinesarray[jjj];
                    thebiggestvalue2_1_position = findredlinesarray[jjj - 1];
                }
            }
        }

        cout << "thebiggestvalue2 : " << thebiggestvalue1 << " thebiggestvalue22 : " << thebiggestvalue2 << endl;
        cout << "the biggest poisiton21 : " << thebiggestvalue1position << " tsbp : " << thebiggestvalue2position << endl;
        cout << "deneme 1 : " << thebiggestvalue1_1_position << " deneme 2 : " << thebiggestvalue2_1_position << endl;

        //2. bild

        vector<Vec4i> lines;
        HoughLinesP(outputImg5, lines, 1, CV_PI / 180, 10, 10, 10);
        for (size_t i = 0; i < lines.size(); i++) {
            Vec4i l = lines[i];
            circle(outputImg5gray, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            circle(outputImg5gray, Point(l[2], l[3]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            if (l[0] >= ((thebiggestvalue1position + thebiggestvalue1_1_position) / 2) && l[0] < 639) {
                x1[numberofpointsoffirstlane2 + numberofpointsoffirstlane1] = l[0];
                //    x[i+1] = l[2];
                y1[numberofpointsoffirstlane2 + numberofpointsoffirstlane1] = l[1] + 300;
                numberofpointsoffirstlane2++;
                //  y[i+1] = l[3];

            } else if (l[0] <= ((thebiggestvalue1position + thebiggestvalue1_1_position) / 2) && l[0] >= ((thebiggestvalue2position + thebiggestvalue2_1_position) / 2)) {
                x2[numberofpointsofsecondlane2 + numberofpointsofsecondlane1] = l[0];
                y2[numberofpointsofsecondlane2 + numberofpointsofsecondlane1] = l[1] + 300;
                numberofpointsofsecondlane2++;

            } else if (l[0] > 0 && l[0] <= ((thebiggestvalue2position + thebiggestvalue2_1_position) / 2)) {
                x3[numberofpointsofthirdlane2 + numberofpointsofthirdlane1] = l[0];
                y3[numberofpointsofthirdlane2 + numberofpointsofthirdlane1] = l[1] + 300;
                numberofpointsofthirdlane2++;
            }
        }


        /*  for (int aaaa = 0; aaaa < 100; aaaa++) {
              cout << "x : " << x3[aaaa] << " y :" << y3[aaaa] << endl;
          }
         */
        
        
        
        
        
  }













void curvefitting(int numberofpointsoffirstlane2, int numberofpointsoffirstlane1,
        double *x, double *y, Mat inputImg) {
    int i, j, k, n, N;
    cout.precision(4); //set precision
    cout.setf(ios::fixed);
    N = numberofpointsoffirstlane2 + numberofpointsoffirstlane1; //lines.size();
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
            X[i] = X[i] + pow(x[j], i);
    }
    double B[n + 1][n + 2], a[n + 1];
    for (i = 0; i <= n; i++)
        for (j = 0; j <= n; j++)
            B[i][j] = X[i + j];
    double Y[n + 1];
    for (i = 0; i < n + 1; i++) {
        Y[i] = 0;
        for (j = 0; j < N; j++)
            Y[i] = Y[i] + pow(x[j], i) * y[j];
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
    for (p = 0; p < 640; p++) {
        r = a[0] + a[1] * p + a[2] * p * p;
        if (r >= 0 && r <= 480)
            // cout<<p<< " degeri " <<r<<endl;
            circle(inputImg, Point(p, r), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
    }

} 