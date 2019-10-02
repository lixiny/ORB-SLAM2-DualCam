#include <Eigen/StdVector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <map>
#include <unordered_map>
#include<string>
#include<thread>
#include <pangolin/pangolin.h>
#include <sys/types.h>
#include <dirent.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

using namespace std;
#define KEYCODE_ESC  0x1B

int kbhit(void);
cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
class Viewer
{
public:
    Viewer(){}

    int nCams;
    vector<cv::Mat> mvRealKFPose;
    vector<cv::Mat> mvRealFramePose;
    vector<cv::Mat> mvRealMPPosition;
    vector<vector<uchar>> mvMPColor;
    vector<int> MPViewedCamera;
    bool isDistCalc = false;
    float totalDist = 0;

    void DrawKeyFrames() {
        const float &w = 0.04;
        const float h = w*0.75;
        const float z = w*0.6;
        for(int i = 0; i < mvRealKFPose.size(); i++) {
            cv::Mat Tcw = mvRealKFPose[i];

            cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = Tcw.rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat Ow = -Rwc*tcw;
            cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
            Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
            Ow.copyTo(Twc.rowRange(0,3).col(3));
            Twc = Twc.t();
            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));
            // glColor3f(0.0f, 1.0f, 0.0f); //
            glColor4f(1.0f,0.28f,0.0f,0.8f);
            glLineWidth(0.8f * 2);

            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }

    }


    void DrawKFTrajctory() {
        glLineWidth(5.0f);
        // glColor4f(1.0f,0.28f,0.0f,0.6f);
        glBegin(GL_LINES);
        float range = mvRealKFPose.size();
        cv::Mat grayImg(1, mvRealKFPose.size(), CV_8UC1);
        for(size_t i=0, iend=mvRealKFPose.size(); i<iend;i++)
        {
            float ratio = i/range;
            float gray = 255 * ratio;
            if (gray < 0) gray = 0;
            if (gray > 255) gray = 255;
            grayImg.at<uchar>(0,i) = gray;
        }
        cv::Mat colorImg;
        cv::applyColorMap(grayImg, colorImg, cv::COLORMAP_WINTER);

        for(size_t i = 0; i < mvRealKFPose.size(); i++) {
            uchar r = colorImg.at<uchar>(0, i * 3);
            uchar g = colorImg.at<uchar>(0, i * 3 + 1);
            uchar b = colorImg.at<uchar>(0, i * 3 + 2);
            float rf = r/255.0;
            float gf = g/255.0;
            float bf = b/255.0;
            // glColor4f(rf,gf,bf,0.9f);
            glColor4f(1.0f,0.0f,0.0f,1.0f);

            cv::Mat Tcw = mvRealKFPose[i];
            cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = Tcw.rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat Ow = -Rwc*tcw;

            if(i != mvRealKFPose.size() - 1){
                cv::Mat Tcw2 = mvRealKFPose[i+1];
                cv::Mat Rcw2 = Tcw2.rowRange(0,3).colRange(0,3);
                cv::Mat tcw2 = Tcw2.rowRange(0,3).col(3);
                cv::Mat Rwc2 = Rcw2.t();
                cv::Mat Ow2 = -Rwc2*tcw2;

                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                if(!isDistCalc) {
                    cv::Mat dist = Ow2 - Ow;
                    float d = dist.dot(dist);
                    d = pow(d, 0.5);
                    totalDist += d;
                }

            }
        }
        if(!isDistCalc) {
            isDistCalc = true;
            cout << "Total Distance: " <<  totalDist << endl;
            cv::Mat Tcw0 = mvRealKFPose[0];
            cv::Mat Rcw0 = Tcw0.rowRange(0,3).colRange(0,3);
            cv::Mat tcw0 = Tcw0.rowRange(0,3).col(3);
            cv::Mat Rwc0 = Rcw0.t();
            cv::Mat Ow0 = -Rwc0*tcw0;

            cv::Mat Tcwl = mvRealKFPose[mvRealKFPose.size()-1];
            cv::Mat Rcwl = Tcwl.rowRange(0,3).colRange(0,3);
            cv::Mat tcwl = Tcwl.rowRange(0,3).col(3);
            cv::Mat Rwcl = Rcwl.t();
            cv::Mat Owl = -Rwcl*tcwl;

            cv::Mat dist = Owl - Ow0;
            float d = dist.dot(dist);
            d = pow(d, 0.5);
            cout << "Start-End Distance: " <<  d << endl;
            float ratio = d / totalDist;
            cout << "Ratio: " << ratio << endl;

            cv::Mat deltaR = Rcw0 * Rcwl.t();
            float trace = deltaR.at<float>(0,0) + deltaR.at<float>(1,1) + deltaR.at<float>(2,2);
            double ER = acos((trace - 1.0)/2.0) / totalDist;
            cout << "Error R" << ER << endl;


        }
        glEnd();
    }


    void DrawFrameTrajctory() {
        glLineWidth(2.0f);
        // glColor4f(1.0f,0.28f,0.0f,0.6f);
        glBegin(GL_LINES);
        float range = mvRealFramePose.size();
        cv::Mat grayImg(1, mvRealFramePose.size(), CV_8UC1);
        for(size_t i=0, iend=mvRealFramePose.size(); i<iend;i++)
        {
            float ratio = i/range;
            float gray = 255 * ratio;
            if (gray < 0) gray = 0;
            if (gray > 255) gray = 255;
            grayImg.at<uchar>(0,i) = gray;
        }
        cv::Mat colorImg;
        cv::applyColorMap(grayImg, colorImg, cv::COLORMAP_WINTER);

        for(size_t i = 0; i < mvRealFramePose.size(); i++) {
            uchar r = colorImg.at<uchar>(0, i * 3);
            uchar g = colorImg.at<uchar>(0, i * 3 + 1);
            uchar b = colorImg.at<uchar>(0, i * 3 + 2);
            float rf = r/255.0;
            float gf = g/255.0;
            float bf = b/255.0;
            // glColor4f(rf,gf,bf,0.9f);
            glColor4f(1.0f,0.0f,0.0f,0.5f);

            cv::Mat Tcw = mvRealFramePose[i];
            cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = Tcw.rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat Ow = -Rwc*tcw;

            if(i != mvRealFramePose.size() - 1){
                cv::Mat Tcw2 = mvRealFramePose[i+1];
                cv::Mat Rcw2 = Tcw2.rowRange(0,3).colRange(0,3);
                cv::Mat tcw2 = Tcw2.rowRange(0,3).col(3);
                cv::Mat Rwc2 = Rcw2.t();
                cv::Mat Ow2 = -Rwc2*tcw2;

                cv::Mat dist = Ow2 - Ow;
                float d = dist.dot(dist);
                d = pow(d, 0.5);
                if(d > 0.1) continue;

                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));


            }
        }

        glEnd();
    }


    void GenerateColorMap() {
        return;
        float dmin = 1000.0f;
        float dmax = 0;
        vector<int> grayMap;
        for(size_t i=0, iend=mvRealMPPosition.size(); i<iend;i++)
        {
            cv::Mat pos = mvRealMPPosition[i];
            float x = pos.at<float>(0);
            float y = pos.at<float>(1);
            float z = pos.at<float>(2);
            float distsquare = (x * x + y * y + z * z);
            float dists = pow( distsquare, 0.5);
            float dist = pow(pos.dot(pos),0.5);
            if(dist > 40.f) continue;
            if(dist > dmax) dmax = dist;
            if(dist < dmin) dmin = dist;
        }

        // cout <<"max min " <<  zmax << " " << zmin << endl;

        float range = dmax - dmin;
        cv::Mat grayImg(1, mvRealMPPosition.size(), CV_8UC1);
        for(size_t i=0, iend=mvRealMPPosition.size(); i<iend;i++)
        {
            cv::Mat pos = mvRealMPPosition[i];
            float x = pos.at<float>(0);
            float y = pos.at<float>(1);
            float z = pos.at<float>(2);
            float distsquare = (x * x + y * y + z * z);
            float dist = pow( distsquare, 0.5);


            float ratio = (dmax - dist)/range;
            float gray = 255 * ratio;
            if (gray < 0) gray = 0;
            if (gray > 255) gray = 255;
            grayImg.at<uchar>(0,i) = gray;
            cout << gray << "  ";cout.flush();
        }
        cv::Mat colorImg;
        cv::applyColorMap(grayImg, colorImg, cv::COLORMAP_RAINBOW);
        // cout << colorImg.type() << endl;

        for(size_t i=0, iend=mvRealMPPosition.size(); i<iend;i++)
        {
            uchar r = colorImg.at<uchar>(0, i * 3);
            uchar g = colorImg.at<uchar>(0, i * 3 + 1);
            uchar b = colorImg.at<uchar>(0, i * 3 + 2);
            vector<uchar> color(3);
            color[0] = r; color[1] = g; color[2]= b;
            mvMPColor.push_back(color);
        }



    }

    void DrawMapPoints() {
        glPointSize(2.75);
        glBegin(GL_POINTS);
        for(size_t i=0, iend=mvRealMPPosition.size(); i<iend;i++)
        {
            cv::Mat pos = mvRealMPPosition[i];
            int cam = MPViewedCamera[i];
            // vector<uchar> color = mvMPColor[i];
            // glColor3f(color[0]/255,color[1]/255,color[2]/255);

            // glColor4f(136.0f/255,72.0f/255,192.0f/255, 0.75);
            // glColor4f(234.0f/255, 147.0f/255 ,10.0f/255, 0.75);
            if(cam == 0) {
                glColor4f(136.0f/255,72.0f/255,192.0f/255, 0.75);
            } else {
                glColor4f(234.0f/255, 147.0f/255 ,10.0f/255, 0.75);
            }
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }
        glEnd();

    }


    void Run() {
        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1280,640);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
        pangolin::Var<bool> menuReset("menu.Reset",false,false);

        pangolin::OpenGlRenderState s_cam(
                    pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                    pangolin::ModelViewLookAt(0,-15.0f, -4.0f,   0,0,-4.0f,   0.0,0.0, 1.0)
                    );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));
        pangolin::OpenGlMatrix Twc;
        pangolin::OpenGlMatrix TwcRelc;
        Twc.SetIdentity();
        bool coloredMP = false;

        while(true)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            if(mvRealKFPose.empty() && mvRealMPPosition.empty()) continue;
            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            if( ! coloredMP){
                GenerateColorMap();
                coloredMP = true;
            }
            DrawKeyFrames();
            DrawKFTrajctory();
            DrawFrameTrajctory();
            DrawMapPoints();
            pangolin::FinishFrame();
            usleep(100);
        }
    }
};


vector<cv::Mat> LoadFramePosesTcw(const string& filePath)
{
    vector<cv::Mat> vKF;
    ifstream input_file;
    input_file.open(filePath, ifstream::in);
    int i = 0;
    while(!input_file.eof()) {
        string s;
        getline(input_file, s);
        //  cout<<s.c_str()<<endl;
        float x , y , z, qx, qy, qz, qw;
        sscanf(s.c_str(),
               "%f %f %f %f %f %f %f",
               &x,&y,&z,&qx,&qy,&qz,&qw);
        Eigen::Isometry3d egTcw = Eigen::Isometry3d::Identity();
        Eigen::Vector3d tcw (x, y, z);
        Eigen::Quaterniond qcw;
        qcw.x() = qx;
        qcw.y() = qy;
        qcw.z() = qz;
        qcw.w() = qw;

        egTcw.rotate(qcw);
        egTcw.pretranslate(tcw);
        cv::Mat Tcw = toCvMat(egTcw.matrix());
        vKF.push_back(Tcw);
    }

    return vKF;
}


vector<cv::Mat> LoadKFPosesTcw(const string& filePath)
{
    vector<cv::Mat> vKF;
    ifstream input_file;
    input_file.open(filePath, ifstream::in);
    int i = 0;
    while(!input_file.eof()) {
        string s;
        getline(input_file, s);
        //  cout<<s.c_str()<<endl;
        float x , y , z, qx, qy, qz, qw;
        sscanf(s.c_str(),
               "%f %f %f %f %f %f %f",
               &x,&y,&z,&qx,&qy,&qz,&qw);
        Eigen::Isometry3d egTcw = Eigen::Isometry3d::Identity();
        Eigen::Vector3d tcw (x, y, z);
        Eigen::Quaterniond qcw;
        qcw.x() = qx;
        qcw.y() = qy;
        qcw.z() = qz;
        qcw.w() = qw;

        egTcw.rotate(qcw);
        egTcw.pretranslate(tcw);
        cv::Mat Tcw = toCvMat(egTcw.matrix());
        vKF.push_back(Tcw);
    }

    return vKF;
}

vector<cv::Mat> LoadMapPoints(const string& filePath, vector<int>& MPColors)
{
    MPColors.clear();
    vector<cv::Mat> vMP;
    ifstream input_file;
    input_file.open(filePath, ifstream::in);

    int i = 0;
    while(!input_file.eof())
    {
        string s;
        getline(input_file, s);
        //  cout<<s.c_str()<<endl;
        float x , y , z, cam;
        sscanf(s.c_str(),"%f %f %f %f",&x,&y,&z, &cam);

        cv::Mat cvMat(3,1,CV_32F);
        cvMat.at<float>(0)=x;
        cvMat.at<float>(1)=y;
        cvMat.at<float>(2)=z;
        vMP.push_back(cvMat);
        MPColors.push_back(cam);
        i++;
    }
    return vMP;
}




int main(int argc, char** argv)
{
    string root = "/home/sirius/Documents/NRB_SLAM/test_Outside/432969373_No.24582_";
    string KeyFramePosePath     = root + "KeyFrames.txt";
    string FramePosePath = root + "FrameSaved.txt";
    string MapPointPath = root + "MapPoints.txt";
    vector<cv::Mat> mvKeyFrames = LoadKFPosesTcw(KeyFramePosePath);
    vector<int> MPViewedCamera;
    vector<cv::Mat> mvMapPoints = LoadMapPoints(MapPointPath, MPViewedCamera);
    vector<cv::Mat> mvFrames = LoadFramePosesTcw(FramePosePath);

    Viewer viewer;
    viewer.mvRealKFPose = mvKeyFrames;
    viewer.mvRealFramePose = mvFrames;
    viewer.mvRealMPPosition = mvMapPoints;
    viewer.MPViewedCamera = MPViewedCamera;

    std::thread* ptViewer = new thread(&Viewer::Run, viewer);

    while(true) {
        if( kbhit())   //key down
        {
            char c = getchar();
            if(c == KEYCODE_ESC)
            {
                break;
            }
        }

    }

    return 0;
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

cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}
