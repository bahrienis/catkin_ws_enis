// Autor : Bahri Enis Demirtel
//IPM + Hough Transformation + KNN + Curve Fitting
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





#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"








using namespace std;
//using namespace cv;

int heightofframe = 480;
int widthofframe = 640;
int degreeofthepolynom = 2;
int fpsvalue = 30;
double thresholdvalue = 0.6;
double r[3];
int firstpicsize = heightofframe/2.4;




double * curvefitting(int numofHoughpoints,
        vector<double> x, vector<double> y, Mat inputImg, string color) {
    int i, j, k, n, N;
    cout.precision(4); //set precision
    cout.setf(ios::fixed);
    N = numofHoughpoints; 

    n = degreeofthepolynom;
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
    
     for (i = 0; i < n; i++)
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
    
    
    
        for(int i=0;i < (degreeofthepolynom + 1); i++){

	r[i] = a[i];

}






int curvey, curvex;
    for (curvey = 0; curvey < heightofframe; curvey++) {
        curvex = a[0] + a[1] * curvey + a[2] * curvey * curvey;
        if (curvex >= 0 && curvex <= widthofframe) {
            if (color == "red") {
                circle(inputImg, Point(curvex, curvey), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            } else if (color == "blue") {
                circle(inputImg, Point(curvex, curvey), 1, Scalar(255, 0, 0), 1, CV_AA, 0);
            } else if (color == "green") {
                circle(inputImg, Point(curvex, curvey), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
            }
        }
    }

return r;



}






void detectDirection(vector<double> x_coord_vect, bool& direction_detected, int& direction) {
    int last_x = x_coord_vect.back();
    int first_x = x_coord_vect.at(0);
    if (abs(last_x - first_x) > 10) {
        direction_detected = true;
        direction = last_x - first_x > 0 ? 1 : 0;
    }
}

void getLinePoints(vector<double>& x_coord_vect, vector<double>& y_coord_vect, string name, vector<Point2f> alldetectedpoints, int starting_x, int starting_y,
        int step, flann::Index& kdtree, int numOfPoints, int dWidth, int dHeight, Mat inputImg) {
    cout << "Inside getLinePoints for " << name << endl;
    bool direction_detected = false;
    int direction = 0;
    int current_x = starting_x;

    for (int current_y = starting_y; current_y > 0; current_y -= step) {
        if (x_coord_vect.size() > 5) {

            detectDirection(x_coord_vect, direction_detected, direction);
            if (direction_detected) {
                cout << "direction for " << name << " is " << direction << endl;
            }
        }
        std::vector<float> query;
        query.push_back(current_x);
        query.push_back(current_y);
        vector<int> indices;
        vector<float> dists;
        kdtree.knnSearch(query, indices, dists, numOfPoints);

        vector<int> k_nearest_x;
        vector<int> k_nearest_y;

        for (int i = 0; i < indices.size(); i++) {
            int cur_x = alldetectedpoints.at(indices.at(i)).x;
            int cur_y = alldetectedpoints.at(indices.at(i)).y;

            bool is_x_coord_valid = !direction_detected || (direction_detected && direction == 1 ? cur_x - x_coord_vect.back() > 0 : cur_x - x_coord_vect.back() < 0);
            bool is_y_coord_valid = y_coord_vect.size() == 0 || abs(cur_y - y_coord_vect.back()) < widthofframe/6.4;

            if (is_x_coord_valid && is_y_coord_valid) {
              //if (true) {
                k_nearest_x.push_back(cur_x);
                k_nearest_y.push_back(cur_y);
            }
        }


        double nextPointX = accumulate(k_nearest_x.begin(), k_nearest_x.end(), 0.0) / k_nearest_x.size();
        double nextPointY = accumulate(k_nearest_y.begin(), k_nearest_y.end(), 0.0) / k_nearest_y.size();


        if (nextPointX > 0 && nextPointX < dWidth && nextPointY > 0 && nextPointY < dHeight) {

            x_coord_vect.push_back(nextPointX);
            y_coord_vect.push_back(nextPointY);
            cout << "Point in " << name << ": [" << nextPointX << "," << nextPointY << "]" << endl;
            if (name == "left")
                circle(inputImg, Point(nextPointX, nextPointY), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            else if (name == "mid")
                circle(inputImg, Point(nextPointX, nextPointY), 1, Scalar(255, 0, 0), 1, CV_AA, 0);
            else if (name == "right")
                circle(inputImg, Point(nextPointX, nextPointY), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
            current_x = nextPointX;
        } else if (direction_detected) {
            current_x = direction == 1 ? current_x + 15 : current_x - 15;
        }
    }
}

int main(int argc, char **argv) {

    //SetUP ROS.
    ros::init(argc, argv, "lane_detector_enis");
    
    
             
 
   ros::NodeHandle n;
    
    
   ros::Publisher rightLane_pub = n.advertise<std_msgs::Float32MultiArray>("rightLane", 1000);
   ros::Publisher middleLane_pub = n.advertise<std_msgs::Float32MultiArray>("middleLane", 1000);
   ros::Publisher leftLane_pub = n.advertise<std_msgs::Float32MultiArray>("leftLane", 1000);

  ros::Rate loop_rate(30);

 
 

    //CV_CAP_ANY == 0 turns on the 2. camera

    VideoCapture cap(CV_CAP_ANY); // OPENT THE VIDEO CAMERO NO. 0

	cap.set(CV_CAP_PROP_FRAME_WIDTH,widthofframe);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,heightofframe);
    cap.set(CV_CAP_PROP_FPS, fpsvalue); //change the frame value
    
    
    
    
    
    
   
    
    
    

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
    
    vector<double> left_line_a0;
    vector<double> left_line_a1;
    vector<double> left_line_a2;
    
    vector<double> middle_line_a0;
    vector<double> middle_line_a1;
    vector<double> middle_line_a2;
    
    vector<double> right_line_a0;
    vector<double> right_line_a1;
    vector<double> right_line_a2;



    // The 4-points at the input image	
    vector<Point2f> origPoints;
    origPoints.push_back(Point2f(0, heightofframe));
    origPoints.push_back(Point2f(widthofframe, heightofframe));
    origPoints.push_back(Point2f(widthofframe, heightofframe/6));
    origPoints.push_back(Point2f(0, heightofframe/6));

    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back(Point2f(widthofframe / 2 - (widthofframe/12.8), heightofframe));
    dstPoints.push_back(Point2f(widthofframe / 2 + (widthofframe/12.8), heightofframe));
    dstPoints.push_back(Point2f(widthofframe, 0));
    dstPoints.push_back(Point2f(0, 0));

    // IPM object



    IPM ipm(Size(widthofframe, heightofframe), Size(widthofframe, heightofframe), origPoints, dstPoints);


    IPM backward_ipm(Size(widthofframe, heightofframe), Size(widthofframe, heightofframe), dstPoints, origPoints);



    // Main loop
    int frameNum = 0;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

 

  
  
  
  
  
    for (;;) {


        Mat inputImg;
        Mat outputImg;
        Mat outputImgAfterGaussian;
        Mat outputImgAfterGaussianGray;
        Mat outputImgAfterThreshold;
        Mat grad;
        Mat gradgray;
        Mat gradSmall;
        Mat gradcolorSmall;
        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Mat grad_x, grad_y;
        Mat abs_grad_x, abs_grad_y;





        std::vector<Point2f> alldetectedpoints;


        int numOfRedLinesUp = 0;
        int findredlinesarray[640];



      

        clock_t begin = clock();



 /*       if (sayi >= 1520) {
            sayi = 0;
        }

        std::string filename = "/home/enis/Desktop/Masterarbeit/photos_16.10.2017_lighton/frame" + std::to_string(sayi) + ".jpg";
        sayi++;
        cout << "frame : " << sayi << endl;

*/


        //std::string filename = "/home/enis/Desktop/Masterarbeit/photos_31.08.2017_geradeaus/frame187.jpg";
          std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame152.jpg";
		//    std::string filename = "/home/enis/Desktop/Masterarbeit/photos_16.10.2017_lighton/frame1470.jpg";

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










        ipm.applyHomography(inputImg, outputImg);
        ipm.drawPoints(origPoints, inputImg);


        imshow("Input", inputImg);
        
        
        imshow("Output", outputImg);





        GaussianBlur(outputImg, outputImgAfterGaussian, Size(5, 5), 0, 0, BORDER_DEFAULT); //GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );		 

        cvtColor(outputImgAfterGaussian, outputImgAfterGaussianGray, CV_BGR2GRAY); // Bunu sil

        minMaxLoc(outputImgAfterGaussianGray, &minVal, &maxVal, &minLoc, &maxLoc);

        cout << "min val : " << minVal << endl;
        cout << "max val: " << maxVal << endl;


        
        
        
        
        threshold(outputImgAfterGaussianGray, outputImgAfterThreshold, thresholdvalue * maxVal, 255, 0);



        
        
        
        
        /// Gradient X
        Sobel(outputImgAfterThreshold, grad_x, ddepth, 1, 0, 1, scale, delta, BORDER_DEFAULT); 
        /// Gradient Y
        Sobel(outputImgAfterThreshold, grad_y, ddepth, 0, 1, 1, scale, delta, BORDER_DEFAULT); 

        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);



        
        cvtColor(grad, gradgray, CV_GRAY2BGR);


        




        Rect Rec1(0, heightofframe-firstpicsize, widthofframe, firstpicsize);

        gradSmall = grad(Rec1);
        cvtColor(gradSmall, gradcolorSmall, CV_GRAY2BGR);


        vector<Vec2f> lines1;
        HoughLines(gradSmall, lines1, 2, CV_PI, 2, 0, 0);

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



        for (int i = 639; i >= 0; i--) {
            Vec3b intensity1 = gradcolorSmall.at<Vec3b>(10, i);
            uchar blue1 = intensity1.val[0];
            uchar green1 = intensity1.val[1];
            uchar red1 = intensity1.val[2];

            if (red1 >= 100 && red1 <= 200) {
                findredlinesarray[numOfRedLinesUp] = i;
                numOfRedLinesUp++;
            }
        }

        int rightLineIndex = 0;
        int midLineIndex = 0;
        int leftLineIndex = 0;

        for (int i = 0; i < numOfRedLinesUp; i++) {
            if (findredlinesarray[i] - findredlinesarray[i + 1] > widthofframe/25.6) {
                midLineIndex = i + 1;
                break;
            }
        }
        if (midLineIndex > 0) {
            for (int i = midLineIndex; i < numOfRedLinesUp; i++) {
                if (findredlinesarray[i] - findredlinesarray[i + 1] > widthofframe/25.6) {
                    leftLineIndex = i + 1;
                    break;
                }
            }
        }

//        cout << "Right line coord :" << findredlinesarray[rightLineIndex] << endl;
//        cout << "Middle line coord :" << findredlinesarray[midLineIndex] << endl;
//        cout << "Left line coord :" << findredlinesarray[leftLineIndex] << endl;

        int x_coord_RightLine = findredlinesarray[rightLineIndex];
        int x_coord_MiddleLine = findredlinesarray[midLineIndex];
        int x_coord_LeftLine = findredlinesarray[leftLineIndex];


        vector<Vec4i> lines1P;
        HoughLinesP(grad, lines1P, 2, CV_PI / 180, 2, 2, 2);
        for (size_t i = 0; i < lines1P.size(); i++) {
            Vec4i l = lines1P[i];
             circle(gradgray, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
             circle(gradgray, Point(l[2], l[3]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            alldetectedpoints.push_back(Point2f(l[0], l[1]));
        }



imshow("Probabilistic Hough Transformation", gradgray);



        int max_y_forRightLine = 0;
        int max_y_forMiddleLine = 0;
        int max_y_forLeftLine = 0;
        for (int i = 0; i < alldetectedpoints.size(); i++) {
            int x_coord = alldetectedpoints.at(i).x;
            int y_coord = alldetectedpoints.at(i).y;
            if (std::abs(x_coord - x_coord_RightLine) < 3 && y_coord > max_y_forRightLine) {
                max_y_forRightLine = y_coord;
            }
            if (std::abs(x_coord - x_coord_MiddleLine) < 3 && y_coord > max_y_forMiddleLine) {
                max_y_forMiddleLine = y_coord;
            }
            if (std::abs(x_coord - x_coord_LeftLine) < 3 && y_coord > max_y_forLeftLine) {
                max_y_forLeftLine = y_coord;
            }
        }

//      cout << "Right Line starting point: " << Point(x_coord_RightLine, max_y_forRightLine) << endl;
//      cout << "Middle Line starting point: " << Point(x_coord_MiddleLine, max_y_forMiddleLine) << endl;
//      cout << "Left Line starting point: " << Point(x_coord_LeftLine, max_y_forLeftLine) << endl;




        flann::KDTreeIndexParams indexParams;
        flann::Index kdtree(Mat(alldetectedpoints).reshape(1), indexParams);

        int numOfPoints = 2;
        int step = 30;
        vector<double> x_rightLine;
        vector<double> y_rightLine;
        vector<double> x_middleLine;
        vector<double> y_middleLine;
        vector<double> x_leftLine;
        vector<double> y_leftLine;
        
        

        getLinePoints(x_rightLine, y_rightLine, "right", alldetectedpoints, x_coord_RightLine, max_y_forRightLine,
                step, kdtree, numOfPoints, dWidth, dHeight, gradgray);


        getLinePoints(x_middleLine, y_middleLine, "mid", alldetectedpoints, x_coord_MiddleLine, max_y_forMiddleLine,
                step, kdtree, numOfPoints, dWidth, dHeight, gradgray);


        if(leftLineIndex>0){
            getLinePoints(x_leftLine, y_leftLine, "left", alldetectedpoints, x_coord_LeftLine, max_y_forLeftLine,
                    step, kdtree, numOfPoints, dWidth, dHeight, gradgray);
        }



  double *p = curvefitting(x_rightLine.size(), y_rightLine, x_rightLine, gradgray, "red");





    std_msgs::Float32MultiArray arrayRight;
   arrayRight.data.clear();
		for (int i = 0; i < (degreeofthepolynom + 1); i++)
		{
			arrayRight.data.push_back(p[i]);
		}
    
    rightLane_pub.publish(arrayRight);





     double *s = curvefitting(x_middleLine.size(), y_middleLine, x_middleLine, gradgray, "green");





std_msgs::Float32MultiArray arrayMiddle;
   arrayMiddle.data.clear();
		
		for (int i = 0; i < (degreeofthepolynom + 1); i++)
		{
			
			arrayMiddle.data.push_back(s[i]);
		}
    
    middleLane_pub.publish(arrayMiddle);
    
    
    
    
    
        if(leftLineIndex>0){
        double *t = curvefitting(x_leftLine.size(), y_leftLine, x_leftLine, gradgray, "blue");
        std_msgs::Float32MultiArray arrayLeft;
   arrayLeft.data.clear();
		
		for (int i = 0; i < (degreeofthepolynom + 1); i++)
		{
			
			arrayLeft.data.push_back(s[i]);
		}
    
    leftLane_pub.publish(arrayLeft);
        }
        
        
        
        
        

    ros::spinOnce();

    loop_rate.sleep();
    
    
    


        /*Mat tried;
         
               backward_ipm.applyHomography( gradgray, tried );	
               backward_ipm.drawPoints(origPoints, inputImg );
                 
             imshow("try", tried); 
         */
         
         
         
         


        imshow("Hough", gradcolorSmall);
        imshow("Sobel", gradgray);



        waitKey(1);



        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * elapsed_secs);
        cout << 1000 * elapsed_secs << endl;
    }
    return 0;
}







