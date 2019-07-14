#include <iostream>
#include "opencv2/opencv.hpp"
#include <math.h>


using namespace std;
using namespace cv;


int main()
{
    VideoCapture cap("/home/einstein/桌面/wind.mp4"); //capture the video from web cam
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

        threshold(image, image, 80, 255, THRESH_BINARY);        //阈值要自己调


        dilate(image,image,Mat());
        dilate(image,image,Mat());


        floodFill(image,Point(5,50),Scalar(255),0,FLOODFILL_FIXED_RANGE);

        threshold(image, image, 80, 255, THRESH_BINARY_INV);

        vector<vector<Point>> contours;
        findContours(image, contours, RETR_LIST, CHAIN_APPROX_NONE);
        for (size_t i = 0; i < contours.size(); i++){

            vector<Point> points;
            double area = contourArea(contours[i]);
            if (area < 50 || 1e4 < area) continue;           
            drawContours(image, contours, static_cast<int>(i), Scalar(0), 2);

            points = contours[i];
            //cout <<area <<endl;
            RotatedRect rrect = fitEllipse(points);
            cv::Point2f* vertices = new cv::Point2f[4];
            rrect.points(vertices);

            float aim = rrect.size.height/rrect.size.width;
            if(aim > 1.7 && aim < 2.6){

                for (int j = 0; j < 4; j++)
                {
                    cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),4);
                }

                float middle = 100000;

                for(size_t j = 1;j < contours.size();j++){

                    vector<Point> pointsA;
                    double area = contourArea(contours[j]);
                    if (area < 50 || 1e4 < area) continue;

                    pointsA = contours[j];

                    RotatedRect rrectA = fitEllipse(pointsA);

                    float aimA = rrectA.size.height/rrectA.size.width;

                    if(aimA > 3.0){
                    float distance = sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x)+
                                          (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));


                    if (middle > distance  )
                        middle = distance;
                    }
                }

                if( middle > 60){                               //这个距离也要根据实际情况调
                    cv::circle(binary,Point(rrect.center.x,rrect.center.y),15,cv::Scalar(0,0,255),4);
                }
            }


        }

        imshow("frame",binary);
        imshow("Original", image);

        if (waitKey(1) == 'q')
        {
            break;
        }
    }
}
