// Autor : Bahri Enis Demirtel
//Method 1 and Method 1b
//Hough Transformation + Rectangle + Curve Fitting + IPM
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
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "IPM.h"
#include "IPM.cpp"

using namespace std;

int numberoftheframe = 0;
int widthofframe = 640;
int heightofframe = 480;
int fpsvalue = 30;
int degreeofthepolynom = 2;
int firstpicsize = heightofframe / 5.2;
int secondpicsize = heightofframe / 5.2;
int thirdpicsize = heightofframe - (firstpicsize + secondpicsize);
double thresholdvalue = 0.6;
double r[3];

double * curvefitting(vector<Point> lanePoints, Mat inputImg, Mat inputImgIPM, string color, IPM ipm) {

    int i, j, k, n, N;
    cout.precision(4); //set precision
    cout.setf(ios::fixed);
    N = lanePoints.size();
    n = degreeofthepolynom;
    double X[2 * n + 1];
    for (i = 0; i < 2 * n + 1; i++) {
        X[i] = 0;
        for (j = 0; j < N; j++)
            X[i] = X[i] + pow((double(ipm.applyHomography(lanePoints.at(j)).y)), i);
    }
    double B[n + 1][n + 2], a[n + 1];
    for (i = 0; i <= n; i++)
        for (j = 0; j <= n; j++)
            B[i][j] = X[i + j];
    double Y[n + 1];
    for (i = 0; i < n + 1; i++) {
        Y[i] = 0;
        for (j = 0; j < N; j++)
            Y[i] = Y[i] + pow((double(ipm.applyHomography(lanePoints.at(j)).y)), i) * double(ipm.applyHomography(lanePoints.at(j)).x);
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
                circle(inputImgIPM, ipm.applyHomography(Point(curvex, curvey)), 1, Scalar(0, 0, 255), 1, CV_AA, 0);
                circle(inputImg, Point(curvex, curvey), 1, Scalar(0, 0, 255), 1, CV_AA, 0);

            } else if (color == "blue") {
                circle(inputImgIPM, ipm.applyHomography(Point(curvex, curvey)), 1, Scalar(255, 0, 0), 1, CV_AA, 0);
                circle(inputImg, Point(curvex, curvey), 1, Scalar(255, 0, 0), 1, CV_AA, 0);
            } else if (color == "green") {
                circle(inputImgIPM, ipm.applyHomography(Point(curvex, curvey)), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
                circle(inputImg, Point(curvex, curvey), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
            }
        }
    }

    return r;

}

vector<Point> rectangle(Mat inputImg, Point Startpoint, vector<Point> houghpoints, String whichlane) {

    double dividebyx1 = 0;
    double dividebyx2 = 0;
    double dividebyy = 0;

    if (whichlane == "middle") {
        dividebyx1 = 5;
        dividebyx2 = 5;
        dividebyy = 2.4;
    } else {
        dividebyx1 = 8;
        dividebyx2 = 8;
        dividebyy = 8;
    }

    vector<Point> rectanglePoints;
    vector<Point> lanePoints;

    int x_value = Startpoint.x;
    int y_value = Startpoint.y;



    int i = 0;
    while (houghpoints.size() > 0) {

        int x_coord = houghpoints.at(i).x;
        int y_coord = houghpoints.at(i).y;

        if (x_coord >= x_value - y_value / dividebyx1 && x_coord <= x_value + y_value / dividebyx2
                && y_coord <= y_value && y_coord >= y_value - y_value / dividebyy) {

            rectanglePoints.push_back(houghpoints.at(i));
            lanePoints.push_back(houghpoints.at(i));
            houghpoints.erase(houghpoints.begin() + i);
        } else {
            i++;
        }

        if (rectanglePoints.size() > 0 && i == houghpoints.size() - 1) {
            rectangle(inputImg, Point(x_value - y_value / dividebyx1, y_value), Point(x_value + y_value / dividebyx2, y_value - y_value / dividebyy), Scalar(255), 1, 8, 0);

            y_value = rectanglePoints.at(0).y;
            x_value = rectanglePoints.at(0).x;

            for (int j = 1; j < rectanglePoints.size(); j++) {
                if (rectanglePoints.at(j).y <= y_value) {
                    y_value = rectanglePoints.at(j).y;
                    x_value = rectanglePoints.at(j).x;
                }
            }

            rectanglePoints.clear();
            i = 0;
            continue;

        }
        if (i == houghpoints.size() - 1 && rectanglePoints.size() == 0) {
            break;
        }
    }

    return lanePoints;
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

    cap.set(CV_CAP_PROP_FPS, fpsvalue); //change the frame value
    cap.set(CV_CAP_PROP_FRAME_WIDTH, widthofframe);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, heightofframe);

    if (!cap.isOpened()) //if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
    }

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    double dFrame = cap.get(CV_CAP_PROP_FPS);

    cout << "FPS value is " << dFrame << endl;
    cout << "Frame size: " << dWidth << " x " << dHeight << endl;

    // The 4-points at the input image	
    vector<Point2f> origPoints;
    origPoints.push_back(Point2f(0, heightofframe));
    origPoints.push_back(Point2f(widthofframe, heightofframe));
    origPoints.push_back(Point2f(widthofframe, heightofframe / 8));
    origPoints.push_back(Point2f(0, heightofframe / 8));

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
        Mat inputImgIPM;
        Mat outputImgAfterGaussian;
        Mat outputImgAfterThreshold;
        Mat grad;
        Mat outputImgAfterGaussianGray;
        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S;
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Mat grad_x, grad_y;
        Mat abs_grad_x, abs_grad_y;
        Mat gradcolor;
        Mat gradSmall, gradcolorSmall;
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
        Point rightLaneStart = Point(0, 0);
        Point middleLaneStart = Point(0, 0);
        Point leftLaneStart = Point(0, 0);

        clock_t begin = clock();

        /*        if (numberoftheframe >= 294) {
                    numberoftheframe = 0;
                }
         
        std::string filename = "/home/enis/Desktop/Masterarbeit/photos_04.09.2017/frame" + std::to_string(numberoftheframe) + ".jpg";
        numberoftheframe++;
        cout << "frame : " << numberoftheframe << endl;
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

        //                bool bSuccess = cap.read(inputImg);
        ipm.applyHomography(inputImg, inputImgIPM);


        GaussianBlur(inputImg, outputImgAfterGaussian, Size(3, 3), 0, 0, BORDER_DEFAULT);
        cvtColor(outputImgAfterGaussian, outputImgAfterGaussianGray, CV_BGR2GRAY);
        minMaxLoc(outputImgAfterGaussianGray, &minVal, &maxVal, &minLoc, &maxLoc);

        cout << "min val : " << minVal << endl;
        cout << "max val: " << maxVal << endl;

        threshold(outputImgAfterGaussianGray, outputImgAfterThreshold, thresholdvalue * maxVal, 255, 1);

        /// Gradient X
        Sobel(outputImgAfterThreshold, grad_x, ddepth, 1, 0, 1, scale, delta, BORDER_DEFAULT);
        /// Gradient Y
        Sobel(outputImgAfterThreshold, grad_y, ddepth, 0, 1, 1, scale, delta, BORDER_DEFAULT);

        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);

        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

        cvtColor(grad, gradcolor, CV_GRAY2BGR);

        Rect Rec1(0, firstpicsize + secondpicsize, widthofframe, thirdpicsize);
        gradSmall = grad(Rec1);
        gradcolorSmall = gradcolor(Rec1);

        vector<Vec2f> lines;
        HoughLines(gradSmall, lines, 2, CV_PI, 2, 0, 0);

        for (size_t i = 0; i < lines.size(); i++) {
            float rho = lines[i][0], theta = lines[i][1];
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
            Vec3b intensity = gradcolorSmall.at<Vec3b>(10, i);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];

            if (red >= 100 && red <= 200) {
                redlines.push_back(i);
            }
        }

        for (int i = 0; i < redlines.size(); i++) {
            if (i <= redlines.size() - 2) {
                if (redlines.at(i) - redlines.at(i + 1) >= (widthofframe / 12.8)) {
                    if (rightLaneIndexFinish == 0) {
                        rightLaneIndexFinish = redlines.at(i);
                        middleLaneIndexStart = redlines.at(i + 1);
                    } else {
                        middleLaneIndexFinish = redlines.at(i);
                        leftLaneIndexStart = redlines.at(i + 1);
                    }
                }
            }
        }

        cout << "right Lane Index Finish : " << rightLaneIndexFinish << endl;
        cout << "middle Lane Index Start: " << middleLaneIndexStart << endl;
        cout << "middle Lane Index Finish: " << middleLaneIndexFinish << endl;
        cout << "left Lane Index Start : " << leftLaneIndexStart << endl;

        vector<Vec4i> linesP;
        HoughLinesP(grad, linesP, 2, CV_PI / 180, 2, 2, 2);
        for (size_t i = 0; i < linesP.size(); i++) {
            Vec4i l = linesP[i];
            circle(inputImg, Point(l[0], l[1]), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
            circle(inputImg, Point(l[2], l[3]), 1, Scalar(0, 255, 0), 1, CV_AA, 0);
            houghpoints.push_back(Point(l[0], l[1]));
            houghpoints.push_back(Point(l[2], l[3]));
        }

        for (int i = 0; i < houghpoints.size(); i++) {
            if (houghpoints.at(i).x >= rightLaneIndexFinish && houghpoints.at(i).y > rightLaneStart.y) {
                rightLaneStart = houghpoints.at(i);
            }

            if ((houghpoints.at(i).x <= middleLaneIndexStart && houghpoints.at(i).x >= middleLaneIndexFinish) && houghpoints.at(i).y > middleLaneStart.y) {
                middleLaneStart = houghpoints.at(i);
            }

            if (houghpoints.at(i).x <= leftLaneIndexStart && houghpoints.at(i).y > leftLaneStart.y) {
                leftLaneStart = houghpoints.at(i);
            }
        }

        cout << "results right : " << rightLaneStart << endl;
        cout << "results middle : " << middleLaneStart << endl;
        cout << "results left : " << leftLaneStart << endl;

        vector<Point> rightLanePoints = rectangle(inputImg, rightLaneStart, houghpoints, "right");
        double *p = curvefitting(rightLanePoints, inputImg, inputImgIPM, "red", ipm);
        std_msgs::Float32MultiArray arrayRight;
        arrayRight.data.clear();
        for (int i = 0; i < (degreeofthepolynom + 1); i++) {
            arrayRight.data.push_back(p[i]);
        }

        rightLane_pub.publish(arrayRight);
        vector<Point> middleLanePoints = rectangle(inputImg, middleLaneStart, houghpoints, "middle");
        double *s = curvefitting(middleLanePoints, inputImg, inputImgIPM, "green", ipm);
        std_msgs::Float32MultiArray arrayMiddle;
        arrayMiddle.data.clear();
        for (int i = 0; i < (degreeofthepolynom + 1); i++) {
            arrayMiddle.data.push_back(s[i]);
        }
        middleLane_pub.publish(arrayMiddle);

        vector<Point> leftLanePoints = rectangle(inputImg, leftLaneStart, houghpoints, "left");
        double *t = curvefitting(leftLanePoints, inputImg, inputImgIPM, "blue", ipm);
        std_msgs::Float32MultiArray arrayLeft;
        arrayLeft.data.clear();

        for (int i = 0; i < (degreeofthepolynom + 1); i++) {
            arrayLeft.data.push_back(s[i]);
        }

        leftLane_pub.publish(arrayLeft);
        ros::spinOnce();
        loop_rate.sleep();

        clock_t end = clock();

        imshow("Input Image", inputImg);
        imshow("Sobel", grad);
        imshow("Sobel color", gradcolor);
        imshow("IPM", inputImgIPM);

        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * elapsed_secs);
        cout << "time : " << 1000 * elapsed_secs << endl;

        waitKey(1);

    }
}
