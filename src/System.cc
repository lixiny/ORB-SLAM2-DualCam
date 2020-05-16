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



#include "Converter.h"
#include "Frame.h"
#include "FrameDrawer.h"
#include "Initializer.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "ORBVocabulary.h"
#include "PnPsolver.h"
#include "Sim3Solver.h"
#include "System.h"
#include "Tracking.h"
#include "Viewer.h"
#include "S.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>


namespace ORB_SLAM2
{

System::System(const string &strVocFile,
               const string &strSettingsFile,
               const eSensor sensor,
               const bool bUseViewer)
    : mSensor(sensor),
      mstrSettingsFile(strSettingsFile),
      mbUseViewer(bUseViewer),
      mpViewer(static_cast<ViewerPtr >(NULL)),
      mbReset(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==DUAL)
        cout << "Dual Camera Version" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = make_shared<ORBVocabulary>();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else if(has_suffix(strVocFile, ".bin"))
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    else
        bVocLoad = false;
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = make_shared<KeyFrameDatabase>(mpVocabulary);

    //Create the Map
    mpMap = make_shared<Map>();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = make_shared<FrameDrawer>(mpMap);
    mpMapDrawer = make_shared<MapDrawer>(mpMap, strSettingsFile);

}

void System::init()
{
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker =  make_shared<Tracking>(shared_from_this(), mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, mstrSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = make_shared<LocalMapping>(mpMap, mpKeyFrameDatabase);

    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = make_shared<LoopClosing>(mpMap, mpKeyFrameDatabase, mpVocabulary);

    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(mbUseViewer)
    {
        mpViewer = make_shared<Viewer>(shared_from_this(), mpFrameDrawer,mpMapDrawer,mpTracker,mstrSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}


cv::Mat System::TrackDual(vector<cv::Mat> &ims, const double &timestamp)
{
    if(mSensor!=DUAL)
    {
        cerr << "ERROR: you called TrackDual but input sensor was not set to DUAL." << endl;
        exit(-1);
    }

    CheckModeChange();
    CheckReset();

    cv::Mat Tcw = mpTracker->GrabImageDual(ims,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mpCurrentFrame->mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mpCurrentFrame->mvTotalKeysUn;

    if(! Tcw.empty() && mbSaveTraj) {
        mvSavedFramePose.push_back(Tcw);
    }

    if( ! Tcw.empty()) {
        mvAllFramePose.push_back(Tcw);
    }

    return Tcw;
}

void System::CheckModeChange()
{
    unique_lock<mutex> lock(mMutexMode);
    if(mbActivateLocalizationMode)
    {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while(!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
    }
    if(mbDeactivateLocalizationMode)
    {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
    }
}

void System::CheckReset()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
}
void System::SaveMapPoint(const string &filename)
{
    cout << endl << "Saving map points to " << S::blue << filename << S::endc << " ...";cout.flush();
    vector<MapPointPtr> vpMps = mpMap->GetAllMapPoints();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpMps.size(); i++)
    {
        MapPointPtr pMP = vpMps[i];

        if(!pMP || pMP->isBad())
            continue;
        cv::Mat pos = pMP->GetWorldPos();
        int firstViewdCamera = pMP->mFirstViewCam;
        f << setprecision(6) << pos.at<float>(0,0) <<" "<<pos.at<float>(1,0) <<" "<<pos.at<float>(2,0) << " " << float(firstViewdCamera) << endl;
    }
    f.close();
    cout << "complete!" << endl;

}

void System::SaveLocalMapPoint(const string &filename)
{
    cout << endl << "Saving LOCAL map points to " << S::blue << filename << S::endc << " ...";cout.flush();
    vector<MapPointPtr> vpMps = mpTracker->GetLocalMapPoints();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpMps.size(); i++)
    {
        MapPointPtr pMP = vpMps[i];

        if(!pMP || pMP->isBad())
            continue;
        cv::Mat pos = pMP->GetWorldPos();
        f << setprecision(6) << pos.at<float>(0,0) <<" "<<pos.at<float>(1,0) <<" "<<pos.at<float>(2,0) <<endl;
    }
    f.close();
    cout << "complete!" << endl;

}


void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    cout << S::red << "ShutDown" << S::endc << endl;
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
    cout << "All threads stopped..." << endl;

//    if(mpViewer){
//        pangolin::BindToContext("ORB-SLAM2-DualCam: Map Viewer");
//    }

}


void System::SaveModeChange()
{
    mbSaveTraj = true;
    if(mnRecordStartKFId == 0) {
        mnRecordStartKFId = mpTracker->GetLastKeyFrameID();
    }
    cout << "\nStart recording at KeyFrame #" << mnRecordStartKFId << endl;

}

void System::SetCompulsoryLost()
{
    mpTracker->mbCompulsoryLost = true;
}

void System::SaveKeyFramePoseTcw(const string &filename)
{
    cout << endl << "Saving key frame pose Tcw to " << S::blue << filename << S::endc << " ...";cout.flush();
    vector<KeyFramePtr> vpKFs = mpMap->GetAllKeyFrames();

    map<size_t, KeyFramePtr> mapIdAndKeyFrame;


    for(size_t i = 0; i < vpKFs.size(); i++) {
        KeyFramePtr pKF = vpKFs[i];
        if(!pKF || pKF->isBad()) continue;
        if(pKF->mnId < mnRecordStartKFId) continue;
        mapIdAndKeyFrame[pKF->mnId] = pKF;
    }


    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(auto it = mapIdAndKeyFrame.begin(); it != mapIdAndKeyFrame.end(); it++)
    {
        KeyFramePtr pKF = it->second;

        if(!pKF || pKF->isBad())
            continue;
        // if(pKF->mnId < mpMap->mnStartSaveKFid) continue;
        cv::Mat Rcw = pKF->GetRotation();
        cv::Mat tcw = pKF->GetTranslation();
        vector<float> qcw = Converter::toQuaternion(Rcw);
        //x, y, z, qx, qy, qz, qw, id
        f << setprecision(6)
          << tcw.at<float>(0) << " " << tcw.at<float>(1) << " " << tcw.at<float>(2) << " "
          << qcw[0] << " " << qcw[1] << " " << qcw[2] << " " << qcw[3] << " " << pKF->mnId <<endl;

    }

    // 保存最后一个Frame的POse
    cv::Mat latestPose = mvAllFramePose[mvAllFramePose.size()-1];
    cv::Mat Rcw = latestPose.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = latestPose.rowRange(0,3).col(3);
    vector<float> qcw = Converter::toQuaternion(Rcw);
    f << setprecision(6)
      << tcw.at<float>(0) << " " << tcw.at<float>(1) << " " << tcw.at<float>(2) << " "
      << qcw[0] << " " << qcw[1] << " " << qcw[2] << " " << qcw[3] << endl;

    f.close();
    cout << " completed!" << endl;

}


void System::SaveFramePoseTcw(const vector<cv::Mat>& poses, const string &filename)
{
    cout << endl << "Saving frame pose Tcw to " << S::blue << filename << S::endc << " ...";cout.flush();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<poses.size(); i++)
    {
        cv::Mat pose = poses[i];
        cv::Mat Rcw = pose.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = pose.rowRange(0,3).col(3);
        vector<float> qcw = Converter::toQuaternion(Rcw);
        // x, y, z, qx, qy, qz, qw
        f << setprecision(6)
          << tcw.at<float>(0) << " " << tcw.at<float>(1) << " " << tcw.at<float>(2) << " "
          << qcw[0] << " " << qcw[1] << " " << qcw[2] << " " << qcw[3] << endl;

    }
    f.close();
    cout << " completed!" << endl;

}


int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPointPtr > System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

bool System::has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}


} //namespace ORB_SLAM
