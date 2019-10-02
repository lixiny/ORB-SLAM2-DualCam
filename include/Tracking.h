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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "ORBVocabulary.h"
#include <mutex>
#include <string>
#include <list>
#include <memory>

#include <mutex>

namespace ORB_SLAM2
{

class MapDrawer;
class KeyFrameDatabase;
class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class Frame;
class KeyFrame;
class ORBextractor;
class Initializer;
class MapPoint;
class PnPsolver;
class Cameras;

typedef shared_ptr<Map> MapPtr;
typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<Initializer> InitializerPtr;
typedef shared_ptr<System> SystemPtr;
typedef shared_ptr<Frame> FramePtr;
typedef shared_ptr<MapPoint> MapPointPtr;
typedef shared_ptr<ORBextractor> ORBextractorPtr;
typedef shared_ptr<KeyFrameDatabase> KeyFrameDatabasePtr;
typedef shared_ptr<Viewer> ViewerPtr;
typedef shared_ptr<FrameDrawer> FrameDrawerPtr;
typedef shared_ptr<MapDrawer> MapDrawerPtr;
typedef shared_ptr<LocalMapping> LocalMappingPtr;
typedef shared_ptr<LoopClosing> LoopClosingPtr;
typedef shared_ptr<PnPsolver> PnPsolverPtr;
typedef shared_ptr<Cameras> CamerasPtr;


class Tracking : public enable_shared_from_this<Tracking>
{  

public:
    Tracking(SystemPtr pSys,
             ORBVocabularyPtr pVoc,
             FrameDrawerPtr pFrameDrawer,
             MapDrawerPtr pMapDrawer,
             MapPtr pMap,
             KeyFrameDatabasePtr pKFDB,
             const string &strSettingPath,
             const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageDual(vector<cv::Mat>& ims, const double &timestamp);

    void SetLocalMapper(LocalMappingPtr pLocalMapper);
    void SetLoopClosing(LoopClosingPtr pLoopClosing);
    void SetViewer(ViewerPtr pViewer);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);
    unsigned int GetLastKeyFrameID();
    vector<MapPointPtr> GetLocalMapPoints();


public:
    const int CAP = 0;
    const int REC = 1;
    double mReloccScale = 1.0;
    int mnSecondMap = 0;
    const int NUM_SECONDMAP = 8;
    const int NUM_FRAME_IN_SECONDMAP = 5;
    bool mbCompulsoryLost = false;

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        FULL = 3,
        LOST=4
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;
    int mnCameras;
    CamerasPtr mpCameras;

    // Current Frame
    FramePtr mpCurrentFrame;
    cv::Mat mImGray;
    vector<cv::Mat> mvImGrays;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    FramePtr mpInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFramePtr> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;
    bool mbIsMapScaled;
    unsigned int mnFirstScaleKFId = 0;
    vector<KeyFramePtr> mvpKeyFrameForSecondMap;
    vector<FramePtr> mvpFrameForSecondMap;

    void Reset();

    void sleep_ms(unsigned int secs);
    std::mutex mMutexLocalMapPoints;

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();
    void CreateSecondMapMultical(const int& anchorCam, double& scale);
    void AdjustSecondMapMultical(const int& anchorCam, double& scale);

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool FindPartialRelocalCandidate();
    bool Relocalization();
    bool RelocalizationPartialOnCam(const int& c, double& scale);

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames(int thCounter = 0);

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;
    bool mbCurrentRelocSuccess = false;

    //Other Thread Pointers
    LocalMappingPtr mpLocalMapper;
    LoopClosingPtr mpLoopClosing;

    //ORB
    vector<ORBextractorPtr> mvpStandardORBextractor;  //  ncameras
    vector<ORBextractorPtr> mvpIniORBextractor;  // ncameras
    ORBextractorPtr mpStandardORBextractor;
    ORBextractorPtr mpIniORBextractor;

    //BoW
    ORBVocabularyPtr mpORBVocabulary;
    KeyFrameDatabasePtr mpKeyFrameDB;

    // Initalization (only for monocular)
    InitializerPtr mpInitializer;

    //Local Map
    KeyFramePtr mpReferenceKF;
    std::vector<KeyFramePtr> mvpLocalKeyFrames;
    std::vector<MapPointPtr> mvpLocalMapPoints;

    int mnRelocSoldiers;
    vector<double> mvRelocSoldierScale;

    
    // System
    SystemPtr mpSystem;
    
    //Drawers
    ViewerPtr mpViewer;
    FrameDrawerPtr mpFrameDrawer;
    MapDrawerPtr mpMapDrawer;

    //Map
    MapPtr mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    vector<cv::Mat> mvK;   // ncameras
    vector<cv::Mat> mvDistCoef;  // ncameras
    vector<cv::Mat> mvTsol_sys;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFramePtr mpLastKeyFrame;
    FramePtr mpLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    unsigned int mnLastLostRelocFrameId;


    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPointPtr> mlpTemporalPoints;

    int mnReadyForInit = 0;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
