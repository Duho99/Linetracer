#include "opencv2/opencv.hpp"
#include <iostream>
#include <cmath>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include "dxl.hpp"
using namespace cv;
using namespace std;
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}
double distance(Point p1, Point p2);
int main(void) {

    string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, \
    height=(int)360, format=(string)NV12 ! \
    nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, \
    format=(string)BGRx ! videoconvert ! \
    video/x-raw, format=(string)BGR !appsink";

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc \
    insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.157 \
    port=8001 sync=false";

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc \
    insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.157 \
    port=8002 sync=false";

    //VideoCapture source("linetracer100rpmleftturn.mp4");

    VideoCapture source(src, CAP_GSTREAMER);
    if (!source.isOpened()) { cerr << "Video error" << endl; return -1; }

    VideoWriter writer1(dst1, 0, (double)30, cv::Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

    VideoWriter writer2(dst2, 0, (double)30, cv::Size(640, 90), true);
    if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }


    Mat frame_original, frame_roi, frame, frame_threshold, frame_labeling;
    Mat labels, stats, centroids;
    Point PtOld = Point(320, 45); //Ã³À½¿£ Áß¾Ó
    Scalar color = Scalar(255, 0, 0);
    int error;
    Dxl mx;
    bool mode = false;
    struct timeval start, end1;
    double diff1;

    signal(SIGINT, ctrlc);
    if (!mx.open()) { cout << "dynamixel open error" << endl; return -1; }
    double vel1 = 0, vel2 = 0, gain = 0.2;


    while (true) {
        gettimeofday(&start, NULL);

        source >> frame_original;
        if (frame_original.empty()) {
            cerr << "frame empty!" << endl;
            break;
        }

        //resize(frame_original, frame_original, Size(640, 360));
        frame_roi = frame_original(Rect(0, 270, 640, 90));
        cvtColor(frame_roi, frame_roi, COLOR_BGR2GRAY);

        //¹à±â º¸Á¤
        Scalar mean1 = mean(frame_roi);
        frame = frame_roi + (100 - mean1[0]);

        //¹à±â º¸Á¤°á°ú È®ÀÎ
        /* Scalar mean2 = mean(frame);
        cout << "mean1: " << mean1[0] << endl;
        cout <<"mean2: " << mean2[0] << endl;    */

        //threshold(frame, frame_threshold, 125, 255, THRESH_BINARY);  //ÀÓ°è°©: 150
        threshold(frame, frame_threshold, 0, 255, THRESH_BINARY | THRESH_OTSU);

        int cnt = connectedComponentsWithStats(frame_threshold, labels, stats, centroids);
        cvtColor(frame_threshold, frame_labeling, COLOR_GRAY2BGR);


        vector<Point> vector_point;
        for (int i = 1; i < cnt; i++) {
            int* p = stats.ptr<int>(i);
            if (p[4] < 100) continue;

            double* q = centroids.ptr<double>(i);
            Point PtObj = Point(q[0], q[1]);
            vector_point.push_back(PtObj);
        }
        Point PtMin = vector_point[0];
        double  min = distance(PtOld, vector_point[0]);
        for (int j = 1; j < vector_point.size(); j++) {
            if (distance(PtOld, vector_point[j]) < min) {
                min = distance(PtOld, vector_point[j]);
                PtMin = vector_point[j];
            }
        }
        for (int k = 1; k < cnt; k++) {
            int* p = stats.ptr<int>(k);
            if (p[4] < 100) continue;

            double* q = centroids.ptr<double>(k);
            if (Point(q[0], q[1]) == PtMin) {
                color = Scalar(0, 0, 255);
                error = 320 - int(PtMin.x);
                cout << "error: " << error << endl;
            }
            else {
                color = Scalar(255, 0, 0);
            }
            rectangle(frame_labeling, Rect(p[0], p[1], p[2], p[3]), color, 2);
            circle(frame_labeling, Point(q[0], q[1]), 5, color, -1);
        }

        PtOld = PtMin;

        vel1 = 200 - gain * error;
        vel2 = 200 + gain * error;

        if (mode)mx.setVelocity(vel1, -vel2);
        if (mx.kbhit()) {
            char c = mx.getch();
            if (c == 'q')break;
            else if (c == 's')mode = true;
        }
        //cout<<"vel1: "<<vel1 << "  vel2: "<<vel2 << endl;



        writer1 << frame_original;
        writer2 << frame_labeling;


        if (ctrl_c_pressed) break;
        gettimeofday(&end1, NULL);
        diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
        cout << "Period:" << diff1 << "sec" << endl << endl;
    }
    mx.setVelocity(0, 0);
    mx.close();
    return 0;
}
double distance(Point p1, Point p2) {

    double distance;

    distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

    return distance;
}