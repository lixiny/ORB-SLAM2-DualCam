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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"
#include "ORBVocabulary.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <mutex>
#include <string>
#include <list>
#include <set>
#include <mutex>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
class KeyFrame;
class Cameras;

typedef shared_ptr<KeyFrameDatabase> KeyFrameDatabasePtr;
typedef shared_ptr<MapPoint> MapPointPtr;
typedef shared_ptr<Frame> FramePtr;
typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<KeyFrame>  KeyFramePtr;
typedef shared_ptr<Cameras> CamerasPtr;


class KeyFrame : public enable_shared_from_this<KeyFrame>
{
public:
    KeyFrame(FramePtr pF, MapPtr pMap, KeyFrameDatabasePtr pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    void SetScale(const double& scale);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter(int cam = 0);
    cv::Mat GetRotation(int cam = 0);
    cv::Mat GetTranslation(int cam = 0);

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFramePtr pKF, const int &weight);
    void EraseConnection(KeyFramePtr pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFramePtr> GetConnectedKeyFrames();
    std::vector<KeyFramePtr > GetVectorCovisibleKeyFrames();
    std::vector<KeyFramePtr> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFramePtr> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFramePtr pKF);

    // Spanning tree functions
    void AddChild(KeyFramePtr pKF);
    void EraseChild(KeyFramePtr pKF);
    void ChangeParent(KeyFramePtr pKF);
    std::set<KeyFramePtr> GetChilds();
    KeyFramePtr GetParent();
    bool hasChild(KeyFramePtr pKF);

    // Loop Edges
    void AddLoopEdge(KeyFramePtr pKF);
    std::set<KeyFramePtr> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(MapPointPtr pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPointPtr pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPointPtr pMP);
    std::set<MapPointPtr> GetMapPoints();
    std::vector<MapPointPtr> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPointPtr GetMapPoint(const size_t &idx);

    // KeyPoint functions
    // 这个函数返回的是在 No.c 相机下 的 keypoint id， 而不是全局的keypoint id
    std::vector<size_t> GetFeaturesInArea(const int& c, const float &x, const float  &y, const float  &r) const;
    size_t GetGlobalIdxByLocal(const size_t& localIdx, const int& cam);
    // Image
    bool IsInImage(const int& c, const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFramePtr pKF1, KeyFramePtr pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:


    FramePtr mpFrame;
    bool mbScaled = false;
    bool mbMapScaled = false;
    bool mbConnectedToSecondMap = false;
    bool mbNotErase = false;
    const int CAP = 0;
    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnTrackSecondMapForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    //    const float fx, fy, cx, cy, invfx, invfy;
    const CamerasPtr mpCameras;
    int mnCams;
    const vector<cv::Mat> mvK;
    const vector<cv::Mat> mvDistCoef;
    const vector<cv::Mat> mvExtrinsics;
    const vector<cv::Mat> mvExtAdj;

    // Number of KeyPoints
    //    const int N;
    int totalN;
    vector<int> mvN;


    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvTotalKeys;
    const std::vector<cv::KeyPoint> mvTotalKeysUn;

    std::unordered_map<size_t, int> keypointToCam;
    std::unordered_map<size_t, size_t> keypointToCamLocal;


    std::vector<std::vector<cv::KeyPoint>> mvvkeysTemp;
    std::vector<std::vector<cv::KeyPoint>> mvvkeysUnTemp;
    const vector<cv::Mat> mvDescriptors;

    //BoW
    vector<DBoW2::BowVector> mvBowVec;
    vector<DBoW2::FeatureVector> mvFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;
    vector<cv::Mat> mvImages;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvInvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const std::vector<float> mvfGridElementWidthInv;
    const std::vector<float> mvfGridElementHeightInv;
    const std::vector<float> mvMinX;
    const std::vector<float> mvMaxX;
    const std::vector<float> mvMinY;
    const std::vector<float> mvMaxY;
    const std::vector<float> mvfx;
    const std::vector<float> mvfy;
    const std::vector<float> mvcx;
    const std::vector<float> mvcy;
    const std::vector<float> mvinvfx;
    const std::vector<float> mvinvfy;


    // Grid over the image to speed up feature matching
    //     std::vector< std::vector <std::vector<size_t> > > mGrid;
    std::vector<std::vector<std::vector<std::vector<std::size_t> > > > mvGrids;
    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    // MapPoints associated to keypoints
    std::vector<MapPointPtr> mvpMapPoints;

    // BoW
    KeyFrameDatabasePtr mpKeyFrameDB;
    ORBVocabularyPtr mpORBvocabulary;



    std::map<KeyFramePtr,int> mConnectedKeyFrameWeights;
    std::vector<KeyFramePtr> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFramePtr mpParent;
    std::set<KeyFramePtr> mspChildrens;
    std::set<KeyFramePtr> mspLoopEdges;

    // Bad flags
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline; // Only for visualization

    MapPtr mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};



} //namespace ORB_SLAM

#endif // KEYFRAME_H
