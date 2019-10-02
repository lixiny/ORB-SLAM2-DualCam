#include "Cameras.h" 
#include <mutex> 
#include <iostream>
 
using namespace std; 
namespace ORB_SLAM2 
{ 
void Cameras::setNCameras(const int _mnCameras){ 
    mnCameras = _mnCameras; 
} 
 
void Cameras::setIntrinsics(const vector<cv::Mat> _mvKs, const vector<cv::Mat> _mvDistCoeffs) { 
    mvKs = _mvKs; 
    mvDistCoeffs = _mvDistCoeffs; 
} 
 
void Cameras::setExtrinsics(const vector<cv::Mat> _mvExtrinsics) { 
    mvExtrinsics = _mvExtrinsics; 

    for(size_t i = 0; i < mvExtrinsics.size(); i++) {
        cv::Mat T = mvExtrinsics[i];
        cv::Mat R = T.rowRange(0,3).colRange(0,3);
        cv::Mat t = T.rowRange(0,3).col(3);


        // Adj
        cv::Mat Adj(6,6,T.type());
        R.copyTo(Adj.rowRange(0,3).colRange(0,3));
        R.copyTo(Adj.rowRange(3,6).colRange(3,6));

        cv::Mat t_hat(3,3,T.type());
        t_hat.at<float>(0,0) = 0; t_hat.at<float>(1,1) = 0;  t_hat.at<float>(2,2) = 0;
        t_hat.at<float>(0,1) = -t.at<float>(2);  t_hat.at<float>(1,0) =  t.at<float>(2);
        t_hat.at<float>(0,2) =  t.at<float>(1);  t_hat.at<float>(2,0) = -t.at<float>(1);
        t_hat.at<float>(1,2) = -t.at<float>(0);  t_hat.at<float>(2,1) =  t.at<float>(0);
        cv::Mat Rt_hat = R * t_hat;
        Rt_hat.copyTo(Adj.rowRange(0,3).colRange(3,6));
//        cout << "Extrinsic:" << i << "\n" << T << endl;
//        cout << "Adj: " << i << "\n" << Adj << endl;
        mvExtAdj.push_back(Adj);

        // Inverse:  Tcs
        cv::Mat Tinv = cv::Mat::eye(4,4,T.type());
        cv::Mat Rinv = R.t();
        cv::Mat tinv = -Rinv * t;
        Rinv.copyTo(Tinv.rowRange(0,3).colRange(0,3));
        tinv.copyTo(Tinv.rowRange(0,3).col(3));
        mvExtrinsicsInverse.push_back(Tinv);


    }
} 
 
int Cameras::getNCameras() { 
    return mnCameras; 
} 
 
cv::Mat Cameras::getKi(int i) { 
    return mvKs[i]; 
} 
cv::Mat Cameras::getDistCoeffsi(int i) { 
    return mvDistCoeffs[i].clone(); 
} 

vector<cv::Mat> Cameras::getK(){
    return mvKs;
}

vector<cv::Mat> Cameras::getDistCoeffs(){
    return mvDistCoeffs;
}
 
cv::Mat Cameras::getExtrinsici(int i) { 
    return mvExtrinsics[i].clone(); 
}

cv::Mat Cameras::getExtrinsicInversei(int i) {
    return mvExtrinsicsInverse[i].clone();
}

cv::Mat Cameras::getExtrinsicAdji(int i) {
    return mvExtAdj[i];
}
} 
