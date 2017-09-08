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













    Mat inputImg;
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
         
     //    std::string filename = "/home/enis/Schreibtisch/Masterarbeit/deneme2/frame12.jpg";
        //std::string filename = "/home/enis/Schreibtisch/Masterarbeit/frame0058.jpg";
std::string filename = "/home/enis/Schreibtisch/Masterarbeit/photos_31.08.2017_geradeaus/frame187.jpg";

        inputImg = imread(filename, CV_LOAD_IMAGE_COLOR);
        if (inputImg.empty()) {
            cout << "can not open " << filename << endl;
            return -1;
        }














        printf("FRAME #%6d ", frameNum);
        fflush(stdout);
        frameNum++;

  //           bool bSuccess = cap.read(inputImg);








        Rect Rec1(0, 200, 640, 280);
        rectangle(inputImg, Rec1, Scalar(255), 1, 8, 0);


        Mat outputImg1 = inputImg(Rec1);



















        //Sobel Operator defined		
        Mat outputImg2;
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

        Mat outputImg4gray, outputImg5gray;


        Canny(grad, dst, 255, 255, 3);
        //       cvtColor(dst, cdst, CV_GRAY2BGR);







        Rect Rec4(0, 0, 640, 140);
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



        int findredlinesarray1[640];
        int a11 = 0;

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
             thebiggestvalue12position  = findredlinesarray1[jj - 2];     
                    
                    
                    
            
            
        for (jj = 3; jj < a11; jj++) {
            thebiggestvalue1 = findredlinesarray1[jj-1] - findredlinesarray1[jj];
            
            if (thebiggestvalue1 >= thebiggestvalue11 || thebiggestvalue1 >= thebiggestvalue12) {
                if(thebiggestvalue1 >= thebiggestvalue11){
                thebiggestvalue12 = thebiggestvalue11;
                thebiggestvalue11 = thebiggestvalue1;
                
                
                thebiggestvalue12position = thebiggestvalue11position;
                thebiggestvalue11position = findredlinesarray1[jj];
                thebiggestvalue11_1_position = findredlinesarray1[jj - 1];
                
                }
                else{
                    thebiggestvalue12 = thebiggestvalue1;
                    
                    
                    thebiggestvalue12position = findredlinesarray1[jj];
                    thebiggestvalue12_1_position = findredlinesarray1[jj - 1];
                }
            }
        }

        cout << "thebiggestvalue1 : " << thebiggestvalue11 << " thebiggestvalue12 : " << thebiggestvalue12 << endl;
         cout << "the biggest poisiton : " << thebiggestvalue11position << " tsbp : " << thebiggestvalue12position << endl;
         cout << "deneme 1 : " << thebiggestvalue11_1_position << " deneme 2 : " << thebiggestvalue12_1_position <<endl;

         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
         
  double x1[10000],y1[10000];
  double x2[10000],y2[10000]; 
   double x3[10000],y3[10000];
  int numberofpointsoffirstlane1 = 0;
  int numberofpointsoffirstlane2 = 0;
  int numberofpointsofsecondlane1 = 0;
  int numberofpointsofsecondlane2 = 0;
  int numberofpointsofthirdlane1 = 0;
  int numberofpointsofthirdlane2 = 0;
  

         
  //1. Bild         
				
  vector<Vec4i> lines1P;
  HoughLinesP(outputImg4, lines1P, 1, CV_PI/180, 10, 10, 10 );
  for( size_t i = 0; i < lines1P.size(); i++)
  
  {
    Vec4i l = lines1P[i];

 //   line( outputImg4gray, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
   
 //   cout << "deneme : " << lines.size() << endl;
    
    
   circle( outputImg4gray, Point(l[0], l[1]), 1, Scalar(0,0,255), 1, CV_AA, 0);
   circle( outputImg4gray, Point(l[2], l[3]), 1, Scalar(0,0,255), 1, CV_AA, 0);
   
   
   
   
   
   

   if(l[0]>= ((thebiggestvalue11position + thebiggestvalue11_1_position) / 2) && l[0]<639){
      x1[numberofpointsoffirstlane1]=l[0];
  //    x[i+1] = l[2];
    y1[numberofpointsoffirstlane1]=l[1]+200;
    numberofpointsoffirstlane1++;
  //  y[i+1] = l[3];
    
    
    
    
 
  
   }
   
   else if(l[0]<= ((thebiggestvalue11position + thebiggestvalue11_1_position) / 2) && l[0]>= ((thebiggestvalue12position + thebiggestvalue12_1_position) / 2)){
       x2[numberofpointsofsecondlane1] = l[0];
       y2[numberofpointsofsecondlane1] = l[1] + 200;
       numberofpointsofsecondlane1++;
   }
   
   
   
   
   else if(l[0] > 0 && l[0]<= (thebiggestvalue12position + thebiggestvalue12_1_position) / 2){
       x3[numberofpointsofthirdlane1] = l[0];
       y3[numberofpointsofthirdlane1] = l[1] + 200;
       numberofpointsofthirdlane1++;
   }
  
   
  }
  
 
  
  
  
  
  
  
  
  /*
   for(int xxx=0;xxx<numberofpointsofsecondlane1;xxx++)
  {
      cout << "x degeri : "<< x2[xxx] << " y degeri : " << y2[xxx] << endl;
  }
  
  */
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
        
         
              Rect Rec5(0, 140, 640, 140);
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

        for (int k = 639; k >= 0; k--) {

            Vec3b intensity2 = outputImg5gray.at<Vec3b>(10, k);
            uchar blue2 = intensity2.val[0];
            uchar green2 = intensity2.val[1];
            uchar red2 = intensity2.val[2];






            if (red2 >= 100 && red2 <= 200) {

                findredlinesarray2[a2] = k;
                a2++;
            }
        }


   /*     for (int i = 0; i < a11; i++) {


            cout << "x değerleri : " << findredlinesarray1[i] << endl;

        }

*/

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
             thebiggestvalue22position  = findredlinesarray2[jjj - 2];     
                    
                    
                    
            
            
        for (jjj = 3; jjj < a11; jjj++) {
            thebiggestvalue2 = findredlinesarray2[jjj-1] - findredlinesarray2[jjj];
            
            if (thebiggestvalue2 >= thebiggestvalue21 || thebiggestvalue2 >= thebiggestvalue22) {
                if(thebiggestvalue2 >= thebiggestvalue21){
                thebiggestvalue22 = thebiggestvalue21;
                thebiggestvalue21 = thebiggestvalue2;
                
                
                thebiggestvalue22position = thebiggestvalue21position;
                thebiggestvalue21position = findredlinesarray2[jjj];
                thebiggestvalue21_1_position = findredlinesarray2[jjj - 1];
                
                }
                else{
                    thebiggestvalue22 = thebiggestvalue2;
                    
                    
                    thebiggestvalue22position = findredlinesarray2[jjj];
                    thebiggestvalue22_1_position = findredlinesarray2[jjj - 1];
                }
            }
        }

        cout << "thebiggestvalue2 : " << thebiggestvalue21 << " thebiggestvalue22 : " << thebiggestvalue22 << endl;
         cout << "the biggest poisiton21 : " << thebiggestvalue21position << " tsbp : " << thebiggestvalue22position << endl;
cout << "deneme 1 : " << thebiggestvalue21_1_position << " deneme 2 : " << thebiggestvalue22_1_position <<endl;
        











  
  
  
 
  
  //2. bild
  
  
  
    vector<Vec4i> lines2P;
  HoughLinesP(outputImg5, lines2P, 1, CV_PI/180, 10, 10, 10 );
  for( size_t i = 0; i < lines2P.size(); i++)
  

  
  {
    Vec4i l = lines2P[i];

 //   line( outputImg4gray, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
   
 //   cout << "deneme : " << lines.size() << endl;
    
    
   circle( outputImg5gray, Point(l[0], l[1]), 1, Scalar(0,0,255), 1, CV_AA, 0);
   circle( outputImg5gray, Point(l[2], l[3]), 1, Scalar(0,0,255), 1, CV_AA, 0);
   
   
   
   
   
   

   if(l[0]>= ((thebiggestvalue21position + thebiggestvalue21_1_position) / 2 )&& l[0]<639){
      x1[numberofpointsoffirstlane2+numberofpointsoffirstlane1]=l[0];
  //    x[i+1] = l[2];
    y1[numberofpointsoffirstlane2+numberofpointsoffirstlane1]=l[1]+340;
    numberofpointsoffirstlane2++;
  //  y[i+1] = l[3];
    
    
    
  //  cout << "x degeri : "<< x[i+lines1P.size()] << " y değeri : " << y[i+lines1P.size()] << endl;
  
   }
   
   
   
   
    else if(l[0]<= ((thebiggestvalue21position + thebiggestvalue21_1_position) / 2 ) && l[0]>= ((thebiggestvalue22position + thebiggestvalue22_1_position) / 2 )){
       x2[numberofpointsofsecondlane2+numberofpointsofsecondlane1] = l[0];
       y2[numberofpointsofsecondlane2+numberofpointsofsecondlane1] = l[1] + 340;
       numberofpointsofsecondlane2++;
   
  
  } 
   
   
   
   
   
      else if(l[0] > 0 && l[0]<= ((thebiggestvalue22position + thebiggestvalue22_1_position) / 2 )){
       x2[numberofpointsofthirdlane2+numberofpointsofthirdlane1] = l[0];
       y2[numberofpointsofthirdlane2+numberofpointsofthirdlane1] = l[1] + 340;
       numberofpointsofthirdlane2++;
   
  
  } 
   
   
   
  }
  
  
  
  

   
   
   
   
   
   
  /*
  for(int xxx=0;xxx<numberofpointsofsecondlane2+numberofpointsofsecondlane1;xxx++)
  {
      cout << "x degeri : "<< x2[xxx] << " y degeri : " << y2[xxx] << endl;
  }
   
     
 */  
   
   
   
   
   
   
   
   
   
   
   
   
   int i1,j1,k1,n1,N1;
   cout.precision(4);  						//set precision
   cout.setf(ios::fixed);
   N1 = numberofpointsoffirstlane2+numberofpointsoffirstlane1;//lines.size();
/*   double x[N],y[N];
   for(i=0;i<N;i++)
   x[i]=l[0];
    for(i=0;i<N;i++)
    y[i]=l[1];   */
    n1 = 2;   								//polinomun derecesi
    double X1[2*n1+1];
    for(i1=0;i1<2*n1+1;i1++)
    {
		X1[i1]=0;
		for(j1=0;j1<N1;j1++)
		X1[i1]=X1[i1]+pow(x1[j1],i1);
	}
	double B1[n1+1][n1+2],a1[n1+1];
	for(i1=0;i1<=n1;i1++)
		for(j1=0;j1<=n1;j1++)
		B1[i1][j1]=X1[i1+j1];
	double Y1[n1+1];
	for(i1=0;i1<n1+1;i1++)
	{
		Y1[i1]=0;
		for(j1=0;j1<N1;j1++)
		Y1[i1]=Y1[i1]+pow(x1[j1],i1)*y1[j1];
	}
	for(i1=0;i1<=n1;i1++)
	B1[i1][n1+1]=Y1[i1];
	n1=n1+1;
/*	cout<<"\nThe Normal (Augmented Matrix) is as follows:\n";
	for(i=0;i<n;i++)
	{
		for(j=0;j<=n;j++)
		cout<<B[i][j]<<setw(16);
		cout<<"\n";
	}
*/	for(i1=0;i1<n1;i1++)
	for(k1=i1+1;k1<n1;k1++)
	if(B1[i1][i1]<B1[k1][i1])
	for(j1=0;j1<=n1;j1++)
	{
		double temp1=B1[i1][j1];
		B1[i1][j1]=B1[k1][j1];
		B1[k1][j1]=temp1;
	}
	for(i1=0;i1<n1-1;i1++)
	for(k1=i1+1;k1<n1;k1++)
	{
		double t1=B1[k1][i1]/B1[i1][i1];
		for(j1=0;j1<=n1;j1++)
		B1[k1][j1]=B1[k1][j1]-t1*B1[i1][j1];
	}
	for(i1=n1-1;i1>=0;i1--)
	{
		a1[i1]=B1[i1][n1];
		for(j1=0;j1<n1;j1++)
		if(j1!=i1)
		a1[i1]=a1[i1]-B1[i1][j1]*a1[j1];
		a1[i1]=a1[i1]/B1[i1][i1];
	}
	cout<<"\nThe values of the cofficients are as follows:\n";
	for(i1=0;i1<n1;i1++)
	cout << "x´"<<i1<<"="<<a1[i1]<<endl;
	cout<<"\nHence the fitted Polynomial is given by:\ny=";
	for(i1=0;i1<n1;i1++)
	cout<<" + ("<<a1[i1]<<")"<<"x´"<<i1;
	cout<<"\n";

	
		
   
   
   
   
   
   
   
   int p1,r1;
   for(p1=0;p1<640;p1++)
   {
	   r1=a1[0] + a1[1] * p1 + a1[2] * p1 * p1;
	   if(r1>=0 && r1<=480)
	  // cout<<p<< " degeri " <<r<<endl;
	   circle( inputImg, Point(p1, r1), 1, Scalar(0,255,0), 1, CV_AA, 0);
	   }
         
         
         
         
         
         
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   int i2,j2,k2,n2,N2;
   cout.precision(4);  						//set precision
   cout.setf(ios::fixed);
   N2 = numberofpointsofsecondlane2+numberofpointsofsecondlane1;//lines.size();
/*   double x[N],y[N];
   for(i=0;i<N;i++)
   x[i]=l[0];
    for(i=0;i<N;i++)
    y[i]=l[1];   */
    n2 = 2;   								//polinomun derecesi
    double X2[2*n2+1];
    for(i2=0;i2<2*n2+1;i2++)
    {
		X2[i2]=0;
		for(j2=0;j2<N2;j2++)
		X2[i2]=X2[i2]+pow(x2[j2],i2);
	}
	double B2[n2+1][n2+2],a22[n2+1];
	for(i2=0;i2<=n2;i2++)
		for(j2=0;j2<=n2;j2++)
		B2[i2][j2]=X2[i2+j2];
	double Y2[n2+1];
	for(i2=0;i2<n2+1;i2++)
	{
		Y2[i2]=0;
		for(j2=0;j2<N2;j2++)
		Y2[i2]=Y2[i2]+pow(x2[j2],i2)*y2[j2];
	}
	for(i2=0;i2<=n2;i2++)
	B2[i2][n2+1]=Y2[i2];
	n2=n2+1;
/*	cout<<"\nThe Normal (Augmented Matrix) is as follows:\n";
	for(i=0;i<n;i++)
	{
		for(j=0;j<=n;j++)
		cout<<B[i][j]<<setw(16);
		cout<<"\n";
	}
*/	for(i2=0;i2<n2;i2++)
	for(k2=i2+1;k2<n2;k2++)
	if(B2[i2][i2]<B2[k2][i2])
	for(j2=0;j2<=n2;j2++)
	{
		double temp2=B2[i2][j2];
		B2[i2][j2]=B2[k2][j2];
		B2[k2][j2]=temp2;
	}
	for(i2=0;i2<n2-1;i2++)
	for(k2=i2+1;k2<n2;k2++)
	{
		double t2=B2[k2][i2]/B2[i2][i2];
		for(j2=0;j2<=n2;j2++)
		B2[k2][j2]=B2[k2][j2]-t2*B2[i2][j2];
	}
	for(i2=n2-1;i2>=0;i2--)
	{
		a22[i2]=B2[i2][n2];
		for(j2=0;j2<n2;j2++)
		if(j2!=i2)
		a22[i2]=a22[i2]-B2[i2][j2]*a22[j2];
		a22[i2]=a22[i2]/B2[i2][i2];
	}
	cout<<"\nThe values of the cofficients are as follows:\n";
	for(i2=0;i2<n2;i2++)
	cout << "x2´"<<i2<<"="<<a22[i2]<<endl;
	cout<<"\nHence the fitted Polynomial is given by:\ny=";
	for(i2=0;i2<n2;i2++)
	cout<<" + ("<<a22[i2]<<")"<<"x2´"<<i2;
	cout<<"\n";

	
		
   
   
   
   
   int p2,r2;
   for(p2=0;p2<640;p2++)
   {
	   r2=a22[0] + a22[1] * p2 + a22[2] * p2 * p2;
	   if(r2>=0 && r2<=480)
	  // cout<<p<< " degeri " <<r<<endl;
	   circle( inputImg, Point(p2, r2), 1, Scalar(0,255,0), 1, CV_AA, 0);
	   }
   
   
   
  
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
      int i3,j3,k3,n3,N3;
   cout.precision(4);  						//set precision
   cout.setf(ios::fixed);
   N3 = numberofpointsofthirdlane2+numberofpointsofthirdlane1;//lines.size();
/*   double x[N],y[N];
   for(i=0;i<N;i++)
   x[i]=l[0];
    for(i=0;i<N;i++)
    y[i]=l[1];   */
    n3 = 2;   								//polinomun derecesi
    double X3[2*n3+1];
    for(i3=0;i3<2*n3+1;i3++)
    {
		X3[i3]=0;
		for(j3=0;j3<N3;j3++)
		X3[i3]=X3[i3]+pow(x3[j3],i3);
	}
	double B3[n3+1][n3+2],a3[n3+1];
	for(i3=0;i3<=n3;i3++)
		for(j3=0;j3<=n3;j3++)
		B3[i3][j3]=X3[i3+j3];
	double Y3[n3+1];
	for(i3=0;i3<n3+1;i3++)
	{
		Y3[i3]=0;
		for(j3=0;j3<N3;j3++)
		Y3[i3]=Y3[i3]+pow(x3[j3],i3)*y3[j3];
	}
	for(i3=0;i3<=n3;i3++)
	B3[i3][n3+1]=Y3[i3];
	n3=n3+1;
/*	cout<<"\nThe Normal (Augmented Matrix) is as follows:\n";
	for(i=0;i<n;i++)
	{
		for(j=0;j<=n;j++)
		cout<<B[i][j]<<setw(16);
		cout<<"\n";
	}
*/	for(i3=0;i3<n3;i3++)
	for(k3=i3+1;k3<n3;k3++)
	if(B3[i3][i3]<B3[k3][i3])
	for(j3=0;j3<=n3;j3++)
	{
		double temp3=B3[i3][j3];
		B3[i3][j3]=B3[k3][j3];
		B3[k3][j3]=temp3;
	}
	for(i3=0;i3<n3-1;i3++)
	for(k3=i3+1;k3<n3;k3++)
	{
		double t3=B3[k3][i3]/B3[i3][i3];
		for(j3=0;j3<=n3;j3++)
		B3[k3][j3]=B3[k3][j3]-t3*B3[i3][j3];
	}
	for(i3=n3-1;i3>=0;i3--)
	{
		a3[i3]=B3[i3][n3];
		for(j3=0;j3<n3;j3++)
		if(j3!=i3)
		a3[i3]=a3[i3]-B3[i3][j3]*a3[j3];
		a3[i3]=a3[i3]/B3[i3][i3];
	}
	cout<<"\nThe values of the cofficients are as follows:\n";
	for(i3=0;i3<n3;i3++)
	cout << "x3´"<<i3<<"="<<a3[i3]<<endl;
	cout<<"\nHence the fitted Polynomial is given by:\ny=";
	for(i3=0;i3<n3;i3++)
	cout<<" + ("<<a3[i3]<<")"<<"x3´"<<i3;
	cout<<"\n";

	
		
   
   
   
   
   int p3,r3;
   for(p3=0;p3<640;p3++)
   {
	   r3=a3[0] + a3[1] * p3 + a3[2] * p3 * p3;
	   if(r3>=0 && r3<=480)
	  // cout<<p<< " degeri " <<r<<endl;
	   circle( inputImg, Point(p3, r3), 1, Scalar(0,255,0), 1, CV_AA, 0);
	   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
         
         
         
         
         
         
         
         
        imshow("Sobel+Canny+Hough Trasformation1", outputImg4gray);


        imshow("Sobel+Canny+Hough Trasformation2", outputImg5gray);







































        imshow("Input", inputImg);
        imshow("Output2", outputImg2);

        waitKey(1000000);







        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        printf("%.2f (ms)\r", 1000 * elapsed_secs);
        cout << 1000 * elapsed_secs << endl;





    }

    return 0;




}







