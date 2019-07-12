#include <iostream>
#include "opencv2/opencv.hpp"
#include <math.h>


using namespace std;
using namespace cv;


int main()
{
    VideoCapture cap("/home/einstein/下载/风车神符模拟击打短版.mp4"); //capture the video from web cam
    // if webcam is not available then exit the program
    if ( !cap.isOpened() )
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    Mat image,binary;
    while(true){
        // read a new frame from webcam
        bool flag = cap.read(image);
         if (!flag)
        {
             cout << "Cannot read a frame from webcam" << endl;
             break;
        }
        image.copyTo(binary);
        resize(image,image,Size(image.cols*0.5,binary.rows*0.5));
        resize(binary,binary,Size(binary.cols*0.5,binary.rows*0.5));

        cvtColor(image,image,COLOR_BGR2GRAY);

        threshold(image, image, 100, 255, THRESH_BINARY);

        floodFill(image,Point(5,100),Scalar(255),0,FLOODFILL_FIXED_RANGE);

        threshold(image, image, 100, 255, THRESH_BINARY_INV);

        vector<vector<Point>> contours;
        findContours(image, contours, RETR_LIST, CHAIN_APPROX_NONE);
        for (size_t i = 0; i < contours.size(); i++){

            vector<Point> points;
            double area = contourArea(contours[i]);
            if (area < 800 || 1000 < area) continue;           //装甲板链接范围大小
            drawContours(image, contours, static_cast<int>(i), Scalar(0), 2);

            points = contours[i];
            //cout <<area <<endl;
            RotatedRect rrect = fitEllipse(points);
            cv::Point2f* vertices = new cv::Point2f[4];
                rrect.points(vertices);


                for (int j = 0; j < 4; j++)
                {
                    cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),4);
                }

                float middle = 100000;

                for(size_t j = 1;j < contours.size();j++){

                    vector<Point> pointsA;
                    double area = contourArea(contours[j]);
                    if (area < 1200 || 5000 < area) continue;   //链接轮廓大小

                    pointsA = contours[j];

                    RotatedRect rrectA = fitEllipse(pointsA);

                    float distance = sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x)+
                                          (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));



                    if (middle > distance  )
                        middle = distance;
                }

                if( middle > 100){                        //要求间距阈值大小
                    cv::circle(binary,Point(rrect.center.x,rrect.center.y),15,cv::Scalar(0,0,255),4);
                }


        }


        imshow("frame",binary);
        imshow("Original", image);

        if (waitKey(100) == 'q')
        {
            break;
        }

    }
}
