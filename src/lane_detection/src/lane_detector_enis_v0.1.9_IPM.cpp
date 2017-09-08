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




#include "IPM.h"
#include "IPM.cpp"




using namespace std;
using namespace cv;




void curvefitting(int numofpointsoffirstlane2, int numofpointsoffirstlane1,
        double *x, double *y, Mat inputImg, string color);




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
        Mat upOutputImggray, downOutputImggray;














        clock_t begin = clock();
        /*
                 if (sayi >= 84) {
                     sayi = 0;
                 }

                 std::string filename = "/home/enis/Schreibtisch/deneme2/frame" + std::to_string(sayi) + ".jpg";
                 sayi++;
         */

        
   //         std::string filename = "/home/enis/Schreibtisch/Masterarbeit/photos_31.08.2017_geradeaus/frame187.jpg";
    //   std::string filename = "/home/enis/Schreibtisch/Masterarbeit/photos_03.09.2017/frame290.jpg";

      //          std::string filename = "/home/enis/Schreibtisch/Masterarbeit/deneme2/frame12.jpg";
    //          std::string filename = "/home/enis/Schreibtisch/Masterarbeit/frame0058.jpg";

   //        std::string filename = "/home/enis/Schreibtisch/frame12.jpg"; 
           

  /*             inputImg = imread(filename, CV_LOAD_IMAGE_COLOR);
               if (inputImg.empty()) {
                   cout << "can not open " << filename << endl;
                   return -1;
               }
  */       

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
	
	
	
	

	
	
	
	
	
	
	




	
	

		 
	
		 ipm.applyHomography( inputImg, outputImg );	

	 	 
	
	
		
		  
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
		 
		 
		 
		 
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        

        Rect Rec1(0, 120, 640, 360);
        rectangle(outputImg, Rec1, Scalar(255), 1, 8, 0);

        outputImg1 = outputImg(Rec1);

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

        Mat upOutputImg = dst(Rec4);
        cvtColor(upOutputImg, upOutputImggray, CV_GRAY2BGR);

        /*			
         vector<Vec4i> lines;
         HoughLinesP(dst, lines, 1, CV_PI/180, 10, 10, 80 );
         for( size_t i = 0; i < lines.size(); i++ )
         {
           Vec4i l = lines[i];
           line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
         }
         */


        int numOfRedLinesUp = 0;
        int findredlinesarray1[640];

        vector<Vec2f> lines1;
        HoughLines(upOutputImg, lines1, 4, CV_PI, 10, 0, 0);

        for (size_t i = 0; i < lines1.size(); i++) {
            float rho = lines1[i][0], theta = lines1[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(upOutputImggray, pt1, pt2, Scalar(0, 0, 150), 1, CV_AA);
        }

        for (int k = 639; k >= 0; k--) {
            Vec3b intensity1 = upOutputImggray.at<Vec3b>(10, k);
            uchar blue1 = intensity1.val[0];
            uchar green1 = intensity1.val[1];
            uchar red1 = intensity1.val[2];

            if (red1 >= 100 && red1 <= 200) {
                findredlinesarray1[numOfRedLinesUp] = k;
                numOfRedLinesUp++;
              
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

        int i = 3;

        thebiggestvalue11 = findredlinesarray1[i - 3] - findredlinesarray1[i - 2];
        thebiggestvalue12 = findredlinesarray1[i - 2] - findredlinesarray1[i - 1];

        thebiggestvalue11position = findredlinesarray1[i - 3];
        thebiggestvalue12position = findredlinesarray1[i - 2];

        for (i = 3; i < numOfRedLinesUp; i++) {
            thebiggestvalue1 = findredlinesarray1[i - 1] - findredlinesarray1[i];
            if (thebiggestvalue1 >= thebiggestvalue11 || thebiggestvalue1 >= thebiggestvalue12) {
                if (thebiggestvalue1 >= thebiggestvalue11) {
                    thebiggestvalue12 = thebiggestvalue11;
                    thebiggestvalue11 = thebiggestvalue1;
                    thebiggestvalue12position = thebiggestvalue11position;
                    thebiggestvalue11position = findredlinesarray1[i];
                    thebiggestvalue11_1_position = findredlinesarray1[i - 1];
                } else {           
                    thebiggestvalue12 = thebiggestvalue1;
                    thebiggestvalue12position = findredlinesarray1[i];
                    thebiggestvalue12_1_position = findredlinesarray1[i - 1];
                }
            }
        }
        
        
        
        
         if(thebiggestvalue12 < 40){
                    thebiggestvalue12position = 0;
                    thebiggestvalue12_1_position = findredlinesarray1[i];
                }
        
        
        
        
        
        
        

        cout << "thebiggestvalue1 : " << thebiggestvalue11 << " thebiggestvalue12 : " << thebiggestvalue12 << endl;
        cout << "the biggest poisiton : " << thebiggestvalue11position << " tsbp : " << thebiggestvalue12position << endl;
        cout << "deneme 1 : " << thebiggestvalue11_1_position << " deneme 2 : " << thebiggestvalue12_1_position << endl;

        double x1[10000], y1[10000];
        double x2[10000], y2[10000];
        double x3[10000], y3[10000];
        int numofpointsoffirstlane1 = 0;
        int numofpointsoffirstlane2 = 0;
        int numofpointsofsecondlane1 = 0;
        int numofpointsofsecondlane2 = 0;
        int numofpointsofthirdlane1 = 0;
        int numofpointsofthirdlane2 = 0;

        //1. Bild         

        vector<Vec4i> lines1P;
        HoughLinesP(upOutputImg, lines1P, 1, CV_PI / 180, 10, 10, 10);
        for (size_t i = 0; i < lines1P.size(); i++) {
            Vec4i l = lines1P[i];
            circle(upOutputImggray, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
        //    circle(upOutputImggray, Point(l[2], l[3]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            if (l[0] >= ((thebiggestvalue11position + thebiggestvalue11_1_position) / 2) && l[0] < 639) {
                x1[numofpointsoffirstlane1] = l[0];
                //    x[i+1] = l[2];
                y1[numofpointsoffirstlane1] = l[1] + 120;
                numofpointsoffirstlane1++;
                //  y[i+1] = l[3];

            } else if (l[0] > 0 && l[0] <= ((thebiggestvalue11position + thebiggestvalue11_1_position) / 2) && l[0] >= ((thebiggestvalue12position + thebiggestvalue12_1_position) / 2)) {
                x2[numofpointsofsecondlane1] = l[0];
                y2[numofpointsofsecondlane1] = l[1] + 120;
                numofpointsofsecondlane1++;
            } else if (l[0] > 0 && l[0] <= ((thebiggestvalue12position + thebiggestvalue12_1_position) / 2)) {
                x3[numofpointsofthirdlane1] = l[0];
                y3[numofpointsofthirdlane1] = l[1] + 120;
                numofpointsofthirdlane1++;
            }
        }
       
        
        
        

        Rect Rec5(0, 180, 640, 180);
        rectangle(dst, Rec5, Scalar(255), 1, 8, 0);

        Mat downOutputImg = dst(Rec5);
        cvtColor(downOutputImg, downOutputImggray, CV_GRAY2BGR);

        vector<Vec2f> lines2;
        HoughLines(downOutputImg, lines2, 4, CV_PI, 10, 0, 0);

        for (size_t i = 0; i < lines2.size(); i++) {
            float rho = lines2[i][0], theta = lines1[i][1];
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            line(downOutputImggray, pt1, pt2, Scalar(0, 0, 150), 1, CV_AA);
        }

        int findredlinesarray2[640];
        int numOfRedLinesDown = 0;

        for (int kk = 639; kk >= 0; kk--) {

            Vec3b intensity2 = downOutputImggray.at<Vec3b>(10, kk);
            uchar blue2 = intensity2.val[0];
            uchar green2 = intensity2.val[1];
            uchar red2 = intensity2.val[2];
            if (red2 >= 100 && red2 <= 200) {
                findredlinesarray2[numOfRedLinesDown] = kk;
                numOfRedLinesDown++;
            }
        }

        
        
        
        
        /*
        x1,y1,x2,y2,x3,y3 = findcurvesvalues(findredlinesarray2, numOfRedLinesDown,downOutputImg,downOutputImggray,
        x1,y1, x2, y2, x3, y3,
        numofpointsoffirstlane1,numofpointsoffirstlane2,
        numofpointsofsecondlane1,numofpointsofsecondlane2,
        numofpointsofthirdlane1,numofpointsofthirdlane2
        );
        
        */
        string color;
        
        
        int thebiggestvalue2 = 0;
        int thebiggestvalue21 = 0;
        int thebiggestvalue22 = 0;
        int thebiggestvalue21position = 0;
        int thebiggestvalue22position = 0;
        int thebiggestvalue21_1_position = 0;
        int thebiggestvalue22_1_position = 0;
        int j = 3;

        thebiggestvalue21 = findredlinesarray2[j - 3] - findredlinesarray2[j - 2];
        thebiggestvalue22 = findredlinesarray2[j - 2] - findredlinesarray2[j - 1];
        thebiggestvalue21position = findredlinesarray2[j - 3];
        thebiggestvalue22position = findredlinesarray2[j - 2];

        for (j = 3; j < numOfRedLinesDown; j++) {
            thebiggestvalue2 = findredlinesarray2[j - 1] - findredlinesarray2[j];
            if (thebiggestvalue2 >= thebiggestvalue21 || thebiggestvalue2 >= thebiggestvalue22) {
                if (thebiggestvalue2 >= thebiggestvalue21) {
                    thebiggestvalue22 = thebiggestvalue21;
                    thebiggestvalue21 = thebiggestvalue2;
                    thebiggestvalue22position = thebiggestvalue21position;
                    thebiggestvalue21position = findredlinesarray2[j];
                    thebiggestvalue21_1_position = findredlinesarray2[j - 1];
                } 
                else {
                    thebiggestvalue22 = thebiggestvalue2;
                    thebiggestvalue22position = findredlinesarray2[j];
                    thebiggestvalue22_1_position = findredlinesarray2[j - 1];
                }
            }
        }
        
        
        
                  if(thebiggestvalue22 < 40){
                    thebiggestvalue22position = 0;
                    thebiggestvalue22_1_position = findredlinesarray2[j];
                }
               

        
        
        
        cout << "thebiggestvalue2 : " << thebiggestvalue21 << " thebiggestvalue22 : " << thebiggestvalue22 << endl;
        cout << "the biggest poisiton21 : " << thebiggestvalue21position << " tsbp : " << thebiggestvalue22position << endl;
        cout << "deneme 1 : " << thebiggestvalue21_1_position << " deneme 2 : " << thebiggestvalue22_1_position << endl;
        
        
        
        
        
        
        
        
        
        //2. bild

        vector<Vec4i> lines2P;
        HoughLinesP(downOutputImg, lines2P, 1, CV_PI / 180, 10, 10, 10);
        for (size_t i = 0; i < lines2P.size(); i++) {
            
            
            Vec4i l = lines2P[i];
            circle(downOutputImggray, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
          //  circle(downOutputImggray, Point(l[2], l[3]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            if (l[0] >= ((thebiggestvalue21position + thebiggestvalue21_1_position) / 2) && l[0] < 639) {
                x1[numofpointsoffirstlane2 + numofpointsoffirstlane1] = l[0];
                //    x[i+1] = l[2];
                y1[numofpointsoffirstlane2 + numofpointsoffirstlane1] = l[1] + 300;
                numofpointsoffirstlane2++;
                //  y[i+1] = l[3];

            } else if (l[0] > 0 && l[0] <= ((thebiggestvalue21position + thebiggestvalue21_1_position) / 2) && l[0] >= ((thebiggestvalue22position + thebiggestvalue22_1_position) / 2)) {
                x2[numofpointsofsecondlane2 + numofpointsofsecondlane1] = l[0];
                y2[numofpointsofsecondlane2 + numofpointsofsecondlane1] = l[1] + 300;
                numofpointsofsecondlane2++;

            } else if (l[0] > 0 && l[0] <= ((thebiggestvalue22position + thebiggestvalue22_1_position) / 2)) {
                x3[numofpointsofthirdlane2 + numofpointsofthirdlane1] = l[0];
                y3[numofpointsofthirdlane2 + numofpointsofthirdlane1] = l[1] + 300;
                numofpointsofthirdlane2++;
            }
        }


         
         
        
        for(int abc = 0; abc < (numofpointsoffirstlane1 + numofpointsoffirstlane2); abc++){
            cout << "x1 degeri : " << x1[abc] << " y1 degeri : " << y1[abc] << endl;
            
        }
        
        
        
        
        
        

        
        
        
        
        curvefitting(numofpointsoffirstlane2, numofpointsoffirstlane1, x1, y1, outputImg, "blue");
        curvefitting(numofpointsofsecondlane2, numofpointsofsecondlane1, x2, y2, outputImg, "green");
        curvefitting(numofpointsofthirdlane2, numofpointsofthirdlane1, x3, y3, outputImg, "red");


        imshow("Sobel+Canny+Hough Trasformation1", upOutputImggray);
        imshow("Sobel+Canny+Hough Trasformation2", downOutputImggray);
        imshow("Input", inputImg);
        imshow("Output2", outputImg2);
         imshow("Output", outputImg);

        waitKey(1);

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * elapsed_secs);
        cout << 1000 * elapsed_secs << endl;
    }
    return 0;
}








        
     



void curvefitting(int numofpointsoffirstlane2, int numofpointsoffirstlane1,
        double *x, double *y, Mat inputImg, string color) {
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
        if (r >= 0 && r <= 480){
            if(color == "red")
            circle(inputImg, Point(p, r), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            else if(color == "blue")
                circle(inputImg, Point(p, r), 1, Scalar(255, 0, 0), 1, CV_AA, 0);
            else if(color == "green")
                circle(inputImg, Point(p, r), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
        }
    }

} 
