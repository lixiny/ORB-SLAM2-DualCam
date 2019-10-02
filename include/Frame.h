/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAME_H
#define FRAME_H


#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"
#include "ORBVocabulary.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <memory>

using namespace std;

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class ORBextractor;
class System;
class Initializer;
class Cameras;

typedef shared_ptr<MapPoint> MapPointPtr;
typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<System> SystemPtr;
typedef shared_ptr<Initializer> InitializerPtr;
typedef shared_ptr<ORBextractor> ORBextractorPtr;
typedef shared_ptr<Cameras> CamerasPtr;

class Frame : public enable_shared_from_this<Frame>
{
public:
    Frame();
    // Constructor for Monocular cameras.
    Frame(SystemPtr pSystem,
          vector<cv::Mat> &ims,
          const double &timeStamp,
          CamerasPtr pCameras,
          vector<ORBextractorPtr> extractors,
          ORBVocabularyPtr voc,
          bool isMapScaled);

    // Extract ORB on the image. 0 for left image and 1 for right image.
//    void ExtractORB(int flag, const cv::Mat &im);
    void ExtractORB(const int& c, vector<cv::KeyPoint>& vKeys, cv::Mat& Descriptor );

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    cv::Mat GetCameraCenter(int cam = 0);

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPointPtr pMP, float viewingCosLimit, bool bForAllCam);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const int& c, const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const int& c, const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    void SaveImageWithMatches(string preName = "");

    size_t GetGlobalIdxByLocal(const size_t& localIdx, const int& cam);


public:

    const int CAP = 0;
    int mnCams;
    // Vocabulary used for relocalization.

    SystemPtr mpSystem;
    vector<cv::Mat> mvImages;
    ORBVocabularyPtr mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractorPtr mpORBextractor;
    vector<ORBextractorPtr> mvpORBextractor;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    CamerasPtr mpCameras;
    vector<cv::Mat> mvK;
    vector<cv::Mat> mvDistCoef;
    vector<cv::Mat> mvExtrinsics;
    vector<cv::Mat> mvExtAdj;



    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    vector<int> mvN;
    int totalN;

    bool mbMapScaled;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<cv::KeyPoint> mvTotalKeys;
    std::vector<cv::KeyPoint> mvTotalKeysUn;

    std::unordered_map<size_t, int> keypointToCam;
    std::unordered_map<size_t, size_t> keypointToCamLocal;


    std::vector<std::vector<cv::KeyPoint>> mvvkeysTemp;
    std::vector<std::vector<cv::KeyPoint>> mvvkeysUnTemp;


    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.

    vector<DBoW2::BowVector> mvBowVec;
    vector<DBoW2::FeatureVector> mvFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors;
    vector<cv::Mat> mvDescriptors;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPointPtr> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    // std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    std::vector<std::vector<std::vector<std::vector<std::size_t> > > > mvGrids;

    // Camera pose.
    cv::Mat mTcw;
    vector<cv::Mat> mvRelocTsw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFramePtr mpReferenceKF;
    KeyFramePtr mpRelocedKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;


    static std::vector<float> mvfGridElementWidthInv;
    static std::vector<float> mvfGridElementHeightInv;
    static std::vector<float> mvMinX;
    static std::vector<float> mvMaxX;
    static std::vector<float> mvMinY;
    static std::vector<float> mvMaxY;
    static std::vector<float> mvfx;
    static std::vector<float> mvfy;
    static std::vector<float> mvcx;
    static std::vector<float> mvcy;
    static std::vector<float> mvinvfx;
    static std::vector<float> mvinvfy;



    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();
    void UndistortKeyPoints(const int& c);

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);
    void ComputeImageBounds(const int& c);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
