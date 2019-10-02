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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <mutex>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>

using namespace std;

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;
class MapPoint;

typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<Frame> FramePtr;
typedef shared_ptr<MapPoint> MapPointPtr;


class MapPoint : public enable_shared_from_this<MapPoint>
{
public:
    MapPoint(const cv::Mat &Pos, KeyFramePtr pRefKF, MapPtr pMap);

    void SetWorldPos(const cv::Mat &Pos);
    void SetScale(const double& scale);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFramePtr GetReferenceKeyFrame();

    std::map<KeyFramePtr,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFramePtr pKF,size_t idx);
    void EraseObservation(KeyFramePtr pKF);

    int GetIndexInKeyFrame(KeyFramePtr pKF);
    bool IsInKeyFrame(KeyFramePtr pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPointPtr pMP);
    MapPointPtr GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFramePtr pKF);
    int PredictScale(const float &currentDist, FramePtr pF);
    void SetFirstViewCamera(const int cam);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int mFirstViewCam = -1;
    int nObs;
    bool mbViewdByDifCams = false;

    bool mbScaled = false;

    // Variables used by the tracking
    int mTrackProjCamera;
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnTrackSecondMapForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFramePtr,size_t> mObservations;
     std::map<KeyFramePtr, int> mObservedCameras;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFramePtr mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPointPtr mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     MapPtr mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
