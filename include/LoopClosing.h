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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "ORBVocabulary.h"
#include <thread>
#include <mutex>
#include <memory>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class KeyFrame;
class Map;
class MapPoint;
class Sim3Solver;

typedef  shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<KeyFrameDatabase> KeyFrameDatabasePtr;
typedef shared_ptr<Tracking> TrackingPtr;
typedef shared_ptr<LocalMapping> LocalMappingPtr;
typedef shared_ptr<MapPoint> MapPointPtr;
typedef shared_ptr<Sim3Solver> Sim3SolverPtr;

class LoopClosing
{
public:

    typedef pair<set<KeyFramePtr>,int> ConsistentGroup;
    typedef map<KeyFramePtr,g2o::Sim3,std::less<KeyFramePtr>,
        Eigen::aligned_allocator<std::pair<const KeyFramePtr, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(MapPtr pMap, KeyFrameDatabasePtr pDB, ORBVocabularyPtr pVoc);

    void SetTracker(TrackingPtr pTracker);

    void SetLocalMapper(LocalMappingPtr pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFramePtr pKF);

    void RequestReset();

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    int QUERY = 0;
    int RESP = 0;
    bool mbFixSCALE = false;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    MapPtr mpMap;
    TrackingPtr mpTracker;

    KeyFrameDatabasePtr mpKeyFrameDB;
    ORBVocabularyPtr mpORBVocabulary;

    LocalMappingPtr mpLocalMapper;

    std::list<KeyFramePtr> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFramePtr mpCurrentKF;
    KeyFramePtr mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFramePtr> mvpEnoughConsistentCandidates;
    std::vector<KeyFramePtr> mvpCurrentConnectedKFs;
    std::vector<MapPointPtr> mvpCurrentMatchedPoints;
    std::vector<MapPointPtr> mvpLoopMapPoints;
    cv::Mat mScq_w, mScw;
    g2o::Sim3 mg2oScq_w, mg2oScw;


    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;


    int mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
