#ifndef CAMERAS_H 
#define CAMERAS_H 
 
#include <mutex> 
#include <map> 
#include <memory> 
#include <opencv2/core/core.hpp> 
 
using namespace std; 

namespace ORB_SLAM2 
{ 
class Cameras{ 
public: 
 
    Cameras(){} 
 
    Cameras(int _mnCameras): mnCameras(_mnCameras){} 
 
    void setNCameras(const int _mnCameras); 
    void setIntrinsics(const vector<cv::Mat> _mvKs, const vector<cv::Mat> _mvDistCoeffs); 
    void setExtrinsics(const vector<cv::Mat> _mvExtrainsic); 
 
    int getNCameras(); 
    cv::Mat getKi(int i); 
    cv::Mat getDistCoeffsi(int i); 
    cv::Mat getExtrinsici(int i); 
    cv::Mat getExtrinsicAdji(int i);
    cv::Mat getExtrinsicInversei(int i);
    vector<cv::Mat> getK();
    vector<cv::Mat> getDistCoeffs();
 
    int mnCameras; 
    vector<cv::Mat> mvKs; 
    vector<cv::Mat> mvDistCoeffs; 
    vector<cv::Mat> mvExtrinsics; 
    vector<cv::Mat> mvExtrinsicsInverse;
    vector<cv::Mat> mvExtAdj;
 
}; 
} 
#endif //ORB_SLAM2_MULTICAMS_H 
