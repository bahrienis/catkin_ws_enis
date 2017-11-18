// Autor : Bahri Enis Demirtel
//Method 3 
//IPM + Hough Transformation + Rectangle Method + Curve Fitting
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

int heightofframe = 240;
int widthofframe = 320;
int degreeofthepolynom = 2;
int fpsvalue = 30;
double thresholdvalue = 0.8;
double r[3];
int firstpicsize = (2 * heightofframe / 3);

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
    for (int i = 0; i < (degreeofthepolynom + 1); i++) {
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

int main(int argc, char **argv) {
    //SetUP ROS.
    ros::init(argc, argv, "lane_detector_enis");
    ros::NodeHandle n;
    ros::Publisher rightLane_pub = n.advertise<std_msgs::Float32MultiArray>("rightLane", 1000);
    ros::Publisher middleLane_pub = n.advertise<std_msgs::Float32MultiArray>("middleLane", 1000);
    ros::Publisher leftLane_pub = n.advertise<std_msgs::Float32MultiArray>("leftLane", 1000);
    ros::Rate loop_rate(30);

    VideoCapture cap(CV_CAP_ANY); // OPENT THE VIDEO CAMERO NO. 0

    cap.set(CV_CAP_PROP_FRAME_WIDTH, widthofframe);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, heightofframe);
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

    int number = 0;

    // The 4-points at the input image	
    vector<Point2f> origPoints;
    origPoints.push_back(Point2f(0, heightofframe));
    origPoints.push_back(Point2f(widthofframe, heightofframe));
    origPoints.push_back(Point2f(widthofframe, heightofframe / 6));
    origPoints.push_back(Point2f(0, heightofframe / 6));

    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back(Point2f(widthofframe / 2 - (widthofframe / 12.8), heightofframe));
    dstPoints.push_back(Point2f(widthofframe / 2 + (widthofframe / 12.8), heightofframe));
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
        std::vector<Point2f> rightline;
        std::vector<Point2f> rightlinerect;
        std::vector<double> x_right;
        std::vector<double> y_right;

        std::vector<Point2f> middleline;
        std::vector<Point2f> middlelinerect;
        std::vector<double> x_middle;
        std::vector<double> y_middle;

        std::vector<Point2f> leftline;
        std::vector<Point2f> leftlinerect;
        std::vector<double> x_left;
        std::vector<double> y_left;

        int numOfRedLinesUp = 0;
        int findredlinesarray1[640];

        clock_t begin = clock();

        /*     if (number >= 251) {
                   number = 0;
               }

               std::string filename = "/home/enis/Desktop/Masterarbeit/photos_14.09.2017_lightoff/frame" + std::to_string(number) + ".jpg";
               number++;
               cout << "frame : " << number << endl;
         */



        //std::string filename = "/home/enis/Desktop/Masterarbeit/photos_31.08.2017_geradeaus/frame187.jpg";
        //  std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame9.jpg";
        //      std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame152.jpg";

        //std::string filename = "/home/enis/Desktop/Masterarbeit/deneme2/frame12.jpg";
        //std::string filename = "/home/enis/Desktop/Masterarbeit/frame0058.jpg";
        //std::string filename = "/home/enis/Desktop/frame12.jpg"; 
        //std::string filename = "/home/enis/Desktop/Masterarbeit/photos_31.08.2017_lighton/frame97.jpg";


        /*       inputImg = imread(filename, CV_LOAD_IMAGE_COLOR);
                if (inputImg.empty()) {
                    cout << "can not open " << filename << endl;
                    return -1;
                }
         */
        printf("FRAME #%6d ", frameNum);
        fflush(stdout);
        frameNum++;

        bool bSuccess = cap.read(inputImg);

        ipm.applyHomography(inputImg, outputImg);
        ipm.drawPoints(origPoints, inputImg);

        imshow("Input", inputImg);
        imshow("Output", outputImg);

        GaussianBlur(outputImg, outputImgAfterGaussian, Size(3, 3), 0, 0, BORDER_DEFAULT);
        cvtColor(outputImgAfterGaussian, outputImgAfterGaussianGray, CV_BGR2GRAY);
        minMaxLoc(outputImgAfterGaussianGray, &minVal, &maxVal, &minLoc, &maxLoc);

        cout << "min val : " << minVal << endl;
        cout << "max val: " << maxVal << endl;

        threshold(outputImgAfterGaussianGray, outputImgAfterThreshold, thresholdvalue*maxVal, 255, 0);

        /// Gradient X
        Sobel(outputImgAfterThreshold, grad_x, ddepth, 1, 0, 1, scale, delta, BORDER_DEFAULT);
        /// Gradient Y
        Sobel(outputImgAfterThreshold, grad_y, ddepth, 0, 1, 1, scale, delta, BORDER_DEFAULT);

        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

        cvtColor(grad, gradgray, CV_GRAY2BGR);

        Rect Rec1(0, firstpicsize, widthofframe, heightofframe - firstpicsize);
        gradSmall = grad(Rec1);
        cvtColor(gradSmall, gradcolorSmall, CV_GRAY2BGR);

        vector<Vec2f> lines1;
        HoughLines(gradSmall, lines1, 2, CV_PI, 2, 2, 2);

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

        for (int i = widthofframe - 1; i >= 0; i--) {
            Vec3b intensity1 = gradcolorSmall.at<Vec3b>(10, i);
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

        for (int i = 0; i < numOfRedLinesUp; i++) {
            if (findredlinesarray1[i] - findredlinesarray1[i + 1] > (widthofframe / 25.6)) {
                midLineIndex = i + 1;
                break;
            }
        }
        if (midLineIndex > 0) {
            for (int i = midLineIndex; i < numOfRedLinesUp; i++) {
                if (findredlinesarray1[i] - findredlinesarray1[i + 1] > (widthofframe / 25.6)) {
                    leftLineIndex = i + 1;
                    break;
                }
            }
        }

        cout << "Right line coord :" << findredlinesarray1[rightLineIndex] << endl;
        cout << "Middle line coord :" << findredlinesarray1[midLineIndex] << endl;
        cout << "Left line coord :" << findredlinesarray1[leftLineIndex] << endl;

        int x_coord_RightLine = findredlinesarray1[rightLineIndex];
        int x_coord_MiddleLine = findredlinesarray1[midLineIndex];
        int x_coord_LeftLine = findredlinesarray1[leftLineIndex];

        vector<Vec4i> lines1P;
        HoughLinesP(grad, lines1P, 4, CV_PI / 180, 2, 2, 2);
        for (size_t i = 0; i < lines1P.size(); i++) {
            Vec4i l = lines1P[i];
            circle(gradgray, Point(l[0], l[1]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            circle(gradgray, Point(l[2], l[3]), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
            alldetectedpoints.push_back(Point2f(l[0], l[1]));
            alldetectedpoints.push_back(Point2f(l[2], l[3]));
        }

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

        cout << "Right Line starting point: " << Point(x_coord_RightLine, max_y_forRightLine) << endl;
        cout << "Middle Line starting point: " << Point(x_coord_MiddleLine, max_y_forMiddleLine) << endl;
        cout << "Left Line starting point: " << Point(x_coord_LeftLine, max_y_forLeftLine) << endl;

        int x_value = x_coord_RightLine;
        int y_value = max_y_forRightLine;

        for (int i = 0; i < alldetectedpoints.size(); i++) {
            int x_coord = alldetectedpoints.at(i).x;
            int y_coord = alldetectedpoints.at(i).y;

            if (x_coord >= x_value - (widthofframe / 20) && x_coord <= x_value + (widthofframe / 20)
                    && y_coord < y_value && y_coord >= y_value - (heightofframe / 9.6)) {

                rectangle(gradgray, Point(x_value - (widthofframe / 20), y_value), Point(x_value + (widthofframe / 20), y_value - (heightofframe / 9.6)), Scalar(255), 1, 8, 0);
                rightline.push_back(alldetectedpoints.at(i));
                x_right.push_back(alldetectedpoints.at(i).x);
                y_right.push_back(alldetectedpoints.at(i).y);

                rightlinerect.push_back(alldetectedpoints.at(i));
            }
            if (rightlinerect.size() > 0 && i == alldetectedpoints.size() - 1) {

                y_value = rightlinerect.at(0).y;
                for (int j = 1; j < rightlinerect.size(); j++) {
                    if (rightlinerect.at(j).y < y_value) {
                        y_value = rightlinerect.at(j).y;
                        x_value = rightlinerect.at(j).x;
                    }
                }
                rightlinerect.clear();
                i = 0;
            }
            if (rightlinerect.size() == 1 && i == alldetectedpoints.size() - 1) {
                i = alldetectedpoints.size();
            }

        }
        x_value = x_coord_MiddleLine;
        y_value = max_y_forMiddleLine;

        for (int i = 0; i < alldetectedpoints.size(); i++) {
            int x_coord = alldetectedpoints.at(i).x;
            int y_coord = alldetectedpoints.at(i).y;

            if (x_coord >= x_value - (widthofframe / 12.8) && x_coord <= x_value + (widthofframe / 12.8)
                    && y_coord < y_value && y_coord >= y_value - (heightofframe / 5.3)) {

                rectangle(gradgray, Point(x_value - (widthofframe / 12.8), y_value), Point(x_value + (widthofframe / 12.8), y_value - (heightofframe / 5.3)), Scalar(255), 1, 8, 0);
                middleline.push_back(alldetectedpoints.at(i));
                x_middle.push_back(alldetectedpoints.at(i).x);
                y_middle.push_back(alldetectedpoints.at(i).y);

                middlelinerect.push_back(alldetectedpoints.at(i));

            }
            if (middlelinerect.size() > 0 && i == alldetectedpoints.size() - 1) {
                y_value = middlelinerect.at(0).y;
                for (int j = 1; j < middlelinerect.size(); j++) {
                    if (middlelinerect.at(j).y < y_value) {
                        y_value = middlelinerect.at(j).y;
                        x_value = middlelinerect.at(j).x;
                    }
                }
                middlelinerect.clear();
                i = 0;

            }
            if (middlelinerect.size() == 1 && i == alldetectedpoints.size() - 1) {

                i = alldetectedpoints.size();
            }
        }
        x_value = x_coord_LeftLine;
        y_value = max_y_forLeftLine;

        for (int i = 0; i < alldetectedpoints.size(); i++) {
            int x_coord = alldetectedpoints.at(i).x;
            int y_coord = alldetectedpoints.at(i).y;

            if (x_coord >= x_value - (widthofframe / 20) && x_coord <= x_value + (widthofframe / 20)
                    && y_coord < y_value && y_coord >= y_value - (heightofframe / 9.6)) {

                rectangle(gradgray, Point(x_value - (widthofframe / 20), y_value), Point(x_value + (widthofframe / 20), y_value - (heightofframe / 9.6)), Scalar(255), 1, 8, 0);
                leftline.push_back(alldetectedpoints.at(i));
                x_left.push_back(alldetectedpoints.at(i).x);
                y_left.push_back(alldetectedpoints.at(i).y);
                leftlinerect.push_back(alldetectedpoints.at(i));
            }
            if (leftlinerect.size() > 0 && i == alldetectedpoints.size() - 1) {
                y_value = leftlinerect.at(0).y;
                for (int j = 1; j < leftlinerect.size(); j++) {
                    if (leftlinerect.at(j).y < y_value) {
                        y_value = leftlinerect.at(j).y;
                        x_value = leftlinerect.at(j).x;
                    }
                }
                leftlinerect.clear();
                i = 0;
            }
            if (leftlinerect.size() == 1 && i == alldetectedpoints.size() - 1) {
                i = alldetectedpoints.size();
            }
        }

        double *p = curvefitting(rightline.size(), y_right, x_right, gradgray, "red");

        std_msgs::Float32MultiArray arrayRight;
        arrayRight.data.clear();
        for (int i = 0; i < (degreeofthepolynom + 1); i++) {
            arrayRight.data.push_back(p[i]);
        }
        rightLane_pub.publish(arrayRight);

        double *s = curvefitting(middleline.size(), y_middle, x_middle, gradgray, "green");

        std_msgs::Float32MultiArray arrayMiddle;
        arrayMiddle.data.clear();
        for (int i = 0; i < (degreeofthepolynom + 1); i++) {
            arrayMiddle.data.push_back(s[i]);
        }
        middleLane_pub.publish(arrayMiddle);

        if (x_coord_LeftLine != x_coord_RightLine) {

            double *t = curvefitting(leftline.size(), y_left, x_left, gradgray, "blue");
            std_msgs::Float32MultiArray arrayLeft;
            arrayLeft.data.clear();
            for (int i = 0; i < (degreeofthepolynom + 1); i++) {
                arrayLeft.data.push_back(s[i]);
            }
            leftLane_pub.publish(arrayLeft);
        }
        ros::spinOnce();
        loop_rate.sleep();

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
