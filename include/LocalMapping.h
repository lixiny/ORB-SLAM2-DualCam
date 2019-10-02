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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <list>
#include <memory>
using namespace std;


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;
class KeyFrame;
class MapPoint;
class KeyFrameDatabase;

typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<MapPoint> MapPointPtr;
typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<LoopClosing> LoopClosingPtr;
typedef shared_ptr<Tracking> TrackingPtr;
typedef shared_ptr<KeyFrameDatabase> KeyFrameDatabasePtr;

class LocalMapping
{
public:
    LocalMapping(MapPtr pMap, KeyFrameDatabasePtr pKFDB);

    void SetLoopCloser(LoopClosingPtr pLoopCloser);

    void SetTracker(TrackingPtr pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFramePtr pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();
    void EmptyLocalMapper();
    void SetScale(const double& scale);

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    long unsigned int mLastCrossCamRelocingKFID = -1;
    long unsigned int mLastCrossCamRelocedKFID = -1;

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void SearchCrossCameras();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFramePtr &pKF1, KeyFramePtr &pKF2, int cam = 0);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);


    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    MapPtr mpMap;
    KeyFrameDatabasePtr mpKeyFrameDB;

    LoopClosingPtr mpLoopCloser;
    TrackingPtr mpTracker;

    std::list<KeyFramePtr> mlNewKeyFrames;

    KeyFramePtr mpCurrentKeyFrame;

    std::list<MapPointPtr> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    bool mbNeedBAUrgent = false;

};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
