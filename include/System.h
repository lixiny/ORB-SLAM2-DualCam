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


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<mutex>
#include<opencv2/core/core.hpp>
#include"ORBVocabulary.h"

using namespace std;

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;
class MapPoint;
class KeyFrameDatabase;
class MapDrawer;
class KeyFrame;

typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<MapPoint> MapPointPtr;
typedef shared_ptr<KeyFrameDatabase> KeyFrameDatabasePtr;
typedef shared_ptr<Viewer> ViewerPtr;
typedef shared_ptr<Tracking> TrackingPtr;
typedef shared_ptr<LocalMapping> LocalMappingPtr;
typedef shared_ptr<LoopClosing> LoopClosingPtr;
typedef shared_ptr<FrameDrawer> FrameDrawerPtr;
typedef shared_ptr<MapDrawer> MapDrawerPtr;

class System : public enable_shared_from_this<System>
{
public:
    // Input sensor
    enum eSensor{
        DUAL=0,
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    void init();
    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackDual(vector<cv::Mat> &ims, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear map)
    void Reset();

    void CheckModeChange();
    void CheckReset();

    void SaveMapPoint(const string &filename);
    void SaveLocalMapPoint(const string& filename);
    void SaveModeChange();
    void SetCompulsoryLost();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();


    void SaveKeyFramePoseTcw(const string &filename);
    void SaveFramePoseTcw(const vector<cv::Mat>& poses, const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPointPtr> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
    bool has_suffix(const std::string &str, const std::string &suffix);


    // Input sensor
    eSensor mSensor;
    int mnCameras;
    bool mbSaveTraj = false;
    size_t mnRecordStartKFId = 0;
    vector<cv::Mat> mvSavedFramePose;
    vector<cv::Mat> mvAllFramePose;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabularyPtr mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabasePtr mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    MapPtr mpMap;

    string mstrSettingsFile;
    bool mbUseViewer;


    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    TrackingPtr mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMappingPtr mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosingPtr mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    ViewerPtr mpViewer;

    FrameDrawerPtr mpFrameDrawer;
    MapDrawerPtr mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPointPtr> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
