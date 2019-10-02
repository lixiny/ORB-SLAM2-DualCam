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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H


#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include <memory>
#include "LoopClosing.h"

using namespace std;
namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class KeyFrame;
class MapPoint;
class Frame;
class Map;

typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<MapPoint> MapPointPtr;
typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<LoopClosing> LoopClosingPtr;
typedef shared_ptr<Tracking> TrackingPtr;
typedef shared_ptr<Frame> FramePtr;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFramePtr> &vpKF, const std::vector<MapPointPtr> &vpMP,
                                 unsigned long fixId, int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(MapPtr pMap, int nIterations=5,unsigned long fixId = 0, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static LocalBundleAdjustment(KeyFramePtr pKF, bool *pbStopFlag, MapPtr pMap, size_t fixId);
    int static PoseOptimization(FramePtr pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(MapPtr pMap, KeyFramePtr pLoopKF, KeyFramePtr pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFramePtr, set<KeyFramePtr> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFramePtr pKF1,
                            const int& queryC,
                            KeyFramePtr pKF2,
                            const int& respC,
                            vector<MapPointPtr > &vpMatches1,
                            g2o::Sim3 &g2oS12,
                            const float th2,
                            const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
