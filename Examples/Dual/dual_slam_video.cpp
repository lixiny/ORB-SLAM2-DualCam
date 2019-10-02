//STD C++ INCLUDES
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <memory>
#include <sys/types.h>
#include <dirent.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

#include <sys/select.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "System.h"
#include "Tracking.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;
using namespace std::chrono;

#define KEYCODE_ESC         0x1B

static void sleep_ms(unsigned int secs);
string getTimeStampInString();
int kbhit(void);

int main(int argc, char** argv)
{

    if(argc != 4)
    {
        cerr << endl << "Usage: ./dual_slam_video   path_to_vocabulary   path_to_settings   path_to_video" << endl;
        return 1;
    }
    string video = argv[3];
    string pathToVoc = argv[1];
    string pathToSetting = argv[2];

    int skip = 30;
    cv::VideoCapture cap(video);
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cv::Size S = cv::Size(width,  height);
    cout << "Image Size: width " << width << " height: " << height << endl;

    shared_ptr<ORB_SLAM2::System> pSLAM = make_shared<ORB_SLAM2::System>(pathToVoc,pathToSetting,ORB_SLAM2::System::DUAL,true);
    pSLAM->init();

    Rect rectLeft(0,0,640,480);
    Rect rectRight(640,0,640,480);
    cv::Mat frame;
    vector<cv::Mat> imgs;
    imgs.resize(2);

    double timestamp = 0.0;

    while (true) {
        cap >> frame;
        if (frame.empty()) continue;
        if(skip-->0) continue;
        Mat rgbLeft = frame(rectLeft);
        Mat rgbRight = frame(rectRight);

        imgs[0] = rgbLeft;
        imgs[1] = rgbRight;
        timestamp += 1.0/30;   // assume 30fps
        pSLAM->TrackDual(imgs, timestamp);
        if( kbhit()) {
            char c = getchar();
            if(c == KEYCODE_ESC) break;
        }
    }
    cap.release();
    pSLAM->Shutdown();
    exit(0);
}


int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

string getTimeStampInString()
{
    milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    auto secs = duration_cast<seconds>(ms);
    ms -= duration_cast<milliseconds>(secs);
    auto mins = duration_cast<minutes>(secs);
    secs -= duration_cast<seconds>(mins);
    auto hour = duration_cast<hours>(mins);
    mins -= duration_cast<minutes>(hour);
    std::stringstream ss;
    ss << hour.count() << mins.count() << secs.count();
    return ss.str();
}

static void sleep_ms(unsigned int secs)

{

    struct timeval tval;

    tval.tv_sec=secs/1000;

    tval.tv_usec=(secs*1000)%1000000;

    select(0,NULL,NULL,NULL,&tval);

}

