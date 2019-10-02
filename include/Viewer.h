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


#ifndef VIEWER_H
#define VIEWER_H

#include <opencv/cv.hpp>
#include <mutex>
#include <memory>


using namespace std;

namespace ORB_SLAM2
{


class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Map;
class MapDrawer;
class KeyFrame;
class Frame;
class MapPoint;
class Tracking;

typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<System> SystemPtr;
typedef shared_ptr<Frame> FramePtr;
typedef shared_ptr<MapPoint> MapPointPtr;
typedef shared_ptr<FrameDrawer> FrameDrawerPtr;
typedef shared_ptr<MapDrawer> MapDrawerPtr;
typedef shared_ptr<Tracking> TrackingPtr;

class Viewer
{
public:
    Viewer(SystemPtr pSystem, FrameDrawerPtr pFrameDrawer, MapDrawerPtr pMapDrawer, TrackingPtr pTracking, const string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

private:

    bool Stop();

    SystemPtr mpSystem;
    FrameDrawerPtr mpFrameDrawer;
    MapDrawerPtr mpMapDrawer;
    TrackingPtr mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

}


#endif // VIEWER_H
	

