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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

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
#include "Cameras.h"
#include "S.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(
        SystemPtr pSys,
        ORBVocabularyPtr  pVoc,
        FrameDrawerPtr pFrameDrawer,
        MapDrawerPtr pMapDrawer,
        MapPtr pMap,
        KeyFrameDatabasePtr  pKFDB,
        const string &strSettingPath,
        const int sensor)
    :
      mState(NO_IMAGES_YET),
      mSensor(sensor),
      mbOnlyTracking(false),
      mbVO(false),
      mpORBVocabulary(pVoc),
      mpKeyFrameDB(pKFDB),
      mpInitializer(static_cast<InitializerPtr >(NULL)),
      mpSystem(pSys),
      mpViewer(NULL),
      mpFrameDrawer(pFrameDrawer),
      mpMapDrawer(pMapDrawer),
      mpMap(pMap),
      mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mnCameras = fSettings["nCameras"];
    mpFrameDrawer->setNCameras(mnCameras);
    mpKeyFrameDB->setNCams(mnCameras);
    mpSystem->mnCameras = mnCameras;
    mvK.resize(mnCameras);
    mvDistCoef.resize(mnCameras);
    mvTsol_sys.resize(mnCameras);
    mvpStandardORBextractor.resize(mnCameras);
    mvpIniORBextractor.resize(mnCameras);
    mpCameras = make_shared<Cameras>(mnCameras);
    mnRelocSoldiers = 0;

    // 相机内外参数赋值
    for(int i = 0; i < mnCameras; i++) {
        string cameraName = "Camera";
        string cameraNamei = cameraName.append(to_string(i));
        float fxi = fSettings[string (cameraNamei + ".fx")];
        float fyi = fSettings[string (cameraNamei + ".fy")];
        float cxi = fSettings[string (cameraNamei + ".cx")];
        float cyi = fSettings[string (cameraNamei + ".cy")];

        cv::Mat Ki = cv::Mat::eye(3,3,CV_32F);
        Ki.at<float>(0,0) = fxi;
        Ki.at<float>(1,1) = fyi;
        Ki.at<float>(0,2) = cxi;
        Ki.at<float>(1,2) = cyi;
        mvK[i] = Ki.clone();

        cv::Mat DistCoefi(4,1,CV_32F);
        DistCoefi.at<float>(0) = fSettings[string (cameraNamei + ".k1")];
        DistCoefi.at<float>(1) = fSettings[string (cameraNamei + ".k2")];
        DistCoefi.at<float>(2) = fSettings[string (cameraNamei + ".p1")];
        DistCoefi.at<float>(3) = fSettings[string (cameraNamei + ".p2")];
        const float k3i = fSettings[string (cameraNamei + ".k3")];
        if(k3i!=0)
        {
            DistCoefi.resize(5);
            DistCoefi.at<float>(4) = k3i;
        }
        mvDistCoef[i] = DistCoefi.clone();

        cout << endl << " ##### Camera[" << i <<  "] Parameters: #####" << endl;
        cout << "- fx: " << fxi << endl;
        cout << "- fy: " << fyi << endl;
        cout << "- cx: " << cxi << endl;
        cout << "- cy: " << cyi << endl;
        cout << "- k1: " << DistCoefi.at<float>(0) << endl;
        cout << "- k2: " << DistCoefi.at<float>(1) << endl;
        if(DistCoefi.rows==5)
            cout << "- k3: " << DistCoefi.at<float>(4) << endl;
        cout << "- p1: " << DistCoefi.at<float>(2) << endl;
        cout << "- p2: " << DistCoefi.at<float>(3) << endl;

        if(i == CAP) {
            auto Tidentity = Eigen::Isometry3d::Identity();
            mvTsol_sys[CAP] = Converter::toCvMat(Tidentity.matrix());
            continue;
        }
        float qwi = fSettings[string (cameraNamei + ".qw")];
        float qxi = fSettings[string (cameraNamei + ".qx")];
        float qyi = fSettings[string (cameraNamei + ".qy")];
        float qzi = fSettings[string (cameraNamei + ".qz")];
        float txi = fSettings[string (cameraNamei + ".tx")];
        float tyi = fSettings[string (cameraNamei + ".ty")];
        float tzi = fSettings[string (cameraNamei + ".tz")];

        Eigen::Isometry3d Tsci = Eigen::Isometry3d::Identity();
        Eigen::Vector3d tsci (txi, tyi, tzi);
        Eigen::Quaterniond qsci;
        qsci.x() = qxi;
        qsci.y() = qyi;
        qsci.z() = qzi;
        qsci.w() = qwi;

        Tsci.rotate(qsci);
        Tsci.pretranslate(tsci);

        mvTsol_sys[i] = Converter::toCvMat(Tsci.matrix());
    }

    mpCameras->setIntrinsics(mvK, mvDistCoef);
    mpCameras->setExtrinsics(mvTsol_sys);
    mpMapDrawer->SetCameras(mpCameras);

    mvK[CAP].copyTo(mK);
    mvDistCoef[CAP].copyTo(mDistCoef);

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    for(int i = 0; i < mnCameras; i++) {
        mvpStandardORBextractor[i] =  make_shared<ORBextractor> (1.3 * nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST) ;
        mvpIniORBextractor[i] =  make_shared<ORBextractor>( 2 * nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    }
    mpStandardORBextractor = mvpStandardORBextractor[CAP];
    mpIniORBextractor = mvpIniORBextractor[CAP];

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;


}

void Tracking::SetLocalMapper(LocalMappingPtr pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosingPtr pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(ViewerPtr pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageDual(vector<cv::Mat>& ims, const double &timestamp)
{

    mvImGrays = ims;

    // converte to Gray
    for(int c = 0; c < mnCameras; c++) {
        if(mvImGrays[c].channels()==3)
        {
            if(mbRGB)
                cvtColor(mvImGrays[c],mvImGrays[c],CV_RGB2GRAY);
            else
                cvtColor(mvImGrays[c],mvImGrays[c],CV_BGR2GRAY);
        }
        else if(mvImGrays[c].channels()==4)
        {
            if(mbRGB)
                cvtColor(mvImGrays[c],mvImGrays[c],CV_RGBA2GRAY);
            else
                cvtColor(mvImGrays[c],mvImGrays[c],CV_BGRA2GRAY);
        }
        // equalizeHist(mvImGrays[c], mvImGrays[c]);
    }
    mImGray = mvImGrays[CAP];

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mpCurrentFrame = make_shared<Frame>(mpSystem, mvImGrays,timestamp, mpCameras, mvpIniORBextractor, mpORBVocabulary, mbIsMapScaled);
    else
        mpCurrentFrame = make_shared<Frame>(mpSystem, mvImGrays,timestamp, mpCameras, mvpStandardORBextractor,mpORBVocabulary, mbIsMapScaled);


    // cout << "frameId: " << mpCurrentFrame->mnId << endl;
    Track();
    return mpCurrentFrame->mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;
    if(mbCompulsoryLost) mState = LOST;

    {
        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        switch (mState) {

        case NOT_INITIALIZED: {
            MonocularInitialization();

            mpFrameDrawer->Update(shared_from_this());

            if(mState!=OK)
                return;
            break;
        }
        case OK:
        case FULL:
        case LOST:{
            bool bOK;
            // if(mpCurrentFrame->mnId >= 500 && mpCurrentFrame->mnId % 307 == 0 && mState == FULL) mState = LOST;  //  test Reloc
            if(mState==OK || mState == FULL) {
                CheckReplacedInLastFrame();
                if(mVelocity.empty() || mpCurrentFrame->mnId<mnLastLostRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            } else {
                bOK = Relocalization();
            }

            mpCurrentFrame->mpReferenceKF = mpReferenceKF;
            if(bOK) bOK = TrackLocalMap();


            if( ! bOK) {
                mState = LOST;
            } else {
                if(mbIsMapScaled && mnSecondMap == NUM_SECONDMAP) {
                    mState = FULL;
                } else {
                    mState = OK;
                    bool relocSuccess = FindPartialRelocalCandidate();
                    if(relocSuccess) {
                        break;
                    }

                }
            }

            mpFrameDrawer->Update(shared_from_this());
            mpMapDrawer->SetCurrentCameraPose(mpCurrentFrame->mTcw);



            // If tracking were good, check if we insert a keyframe
            if(bOK){
                // Update motion model
                if(!mpLastFrame->mTcw.empty())
                {
                    cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                    mpLastFrame->GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                    mpLastFrame->GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                    mVelocity = mpCurrentFrame->mTcw * LastTwc;
                }
                else
                    mVelocity = cv::Mat();

                // Clean VO matches
                for(int i=0; i<mpCurrentFrame->totalN; i++)
                {
                    int cam = mpCurrentFrame->keypointToCam[i];
                    if(cam != CAP &&  mpCurrentFrame->mnId == mnLastRelocFrameId) continue;
                    auto pMP = mpCurrentFrame->mvpMapPoints[i];
                    if(! pMP ) continue;
                    if(pMP->Observations() >= 1) continue;
                    mpCurrentFrame->mvbOutlier[i] = false;
                    mpCurrentFrame->mvpMapPoints[i] = static_cast<MapPointPtr >(NULL);
                }

                for(int i=0; i<mpCurrentFrame->totalN;i++)
                {
                    int cam = mpCurrentFrame->keypointToCam[i];
                    if(cam != CAP &&  mpCurrentFrame->mnId == mnLastRelocFrameId) continue;
                    auto pMP = mpCurrentFrame->mvpMapPoints[i];
                    if( !pMP) continue;
                    if( ! mpCurrentFrame->mvbOutlier[i]) continue;
                    mpCurrentFrame->mvpMapPoints[i]=static_cast<MapPointPtr >(NULL);
                }

#ifdef DEBUG
                if(mState == FULL) {
                    const vector<MapPointPtr > vpMapPointsKF = mpCurrentFrame->mvpMapPoints;
                    unordered_map<int, int> CamMPs;
                    int totalMP = 0;
                    for(size_t i = 0; i < vpMapPointsKF.size(); i++){
                        auto pMP = vpMapPointsKF[i];
                        if(!pMP || pMP->isBad()) continue;
                        int camid = mpCurrentFrame->keypointToCam[i];
                        CamMPs[camid]++;
                        totalMP++;
                    }
                    cout << S::green << "## Aft Track Local Map and Opt" << mpCurrentFrame->mnId << " has " << totalMP << " MPs " << endl;
                    for(auto cam : CamMPs) cout << "\t Fcam " << cam.first << " has " << cam.second << " MPs ";
                    cout << S::endc << endl;
                    // mpCurrentFrame->SaveImageWithMatches("AftTrackFin_");
                }
#endif


                // Check if we need to insert a new keyframe
                if(NeedNewKeyFrame())
                    CreateNewKeyFrame();

            }

            // Reset if the camera get lost soon after initialization
            if(mState==LOST){
                if(mpMap->KeyFramesInMap()<=5){
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }


            if(!mpCurrentFrame->mpReferenceKF)
                mpCurrentFrame->mpReferenceKF = mpReferenceKF;

            mpLastFrame = mpCurrentFrame;

            break;
        }

        case SYSTEM_NOT_READY: break;
        default: break;
        }
    }

    if(mState == OK) {

        // 第一种情况
        if(mnRelocSoldiers == NUM_FRAME_IN_SECONDMAP)
        {
            // 取得3帧之间的scale平均值
            double avg = 0;
            //sort(mvRelocSoldierScale.begin(), mvRelocSoldierScale.end());
            //auto itfirst = mvRelocSoldierScale.begin() + 1;
            //auto itend = mvRelocSoldierScale.end() - 1;
            //vector<double> insideThree(itfirst, itend);

            for(int i = 0; i < (int)mvRelocSoldierScale.size(); i++) {
                //cout << insideThree[i] << " ";cout.flush();
                avg+=mvRelocSoldierScale[i];
            }
            cout << endl;
            avg /= mvRelocSoldierScale.size();
            mReloccScale = avg;

            CreateSecondMapMultical(REC, mReloccScale);
        } // 第二种情况
        else if(mbIsMapScaled && mnLastRelocFrameId == mpCurrentFrame->mnId)
        {
            AdjustSecondMapMultical(REC, mReloccScale);
        }

    }


    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mpCurrentFrame->mTcw.empty())
    {
        cv::Mat Tcr = mpCurrentFrame->mTcw*mpCurrentFrame->mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mpCurrentFrame->mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}


bool Tracking::FindPartialRelocalCandidate()
{
    if(!mbIsMapScaled && mpCurrentFrame->mnId <= mnLastRelocFrameId + 30) return false;
    if(mbIsMapScaled && mpCurrentFrame->mnId <= mnLastRelocFrameId + 50) return false;

    if(mpMap->KeyFramesInMap() >= 10) {
        double scale = 1.0;
        bool relocC = RelocalizationPartialOnCam(REC, scale);
        if(! relocC) return false;
        if(mbIsMapScaled) {
            mnLastRelocFrameId = mpCurrentFrame->mnId;
            mReloccScale = scale;
            return true;
        }

        mnRelocSoldiers++;
        mvRelocSoldierScale.push_back(scale);
        mvpFrameForSecondMap.push_back(mpCurrentFrame);
        mnLastRelocFrameId = mpCurrentFrame->mnId;

        if( mnRelocSoldiers == NUM_FRAME_IN_SECONDMAP) return true;
    }

    return false;
}

void Tracking::AdjustSecondMapMultical(const int& camS, double& scale)
{
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    KeyFramePtr pKF = make_shared<KeyFrame>(mpCurrentFrame, mpMap, mpKeyFrameDB);
    pKF->ComputeBoW();
    for(size_t i = 0 ; i < mpCurrentFrame->mvpMapPoints.size(); i++) {
        auto pMP =  mpCurrentFrame->mvpMapPoints[i];
        if( !pMP ) continue;
        if(pMP->isBad()) continue;
        int camid = mpCurrentFrame->keypointToCam[i];
        if(camid != 0) cout<<"*";cout.flush();
        pMP->AddObservation(pKF, i);
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
    }

    /*
    ORBmatcher matcher;
    auto allMapPoints = mpMap->GetAllMapPoints();
    set<MapPointPtr> spAlreadyFoundMPs = pKF->GetMapPoints();
    int matches = matcher.SearchByProjection(pKF,
                                             allMapPoints,
                                             spAlreadyFoundMPs,
                                             3, 64);
    cout << "additional match in new scale: "<<  matches << endl;
    */

    pKF->UpdateConnections();
    pKF->mbConnectedToSecondMap = true;
    mpCurrentFrame->mpRelocedKF->UpdateConnections();
    mpCurrentFrame->mpRelocedKF->mbConnectedToSecondMap = true;

    mpMap->AddKeyFrame(pKF);
    mpLocalMapper->InsertKeyFrame(pKF);
    mnFirstScaleKFId = pKF->mnId;


    mpReferenceKF = pKF;
    mpCurrentFrame->mpReferenceKF = pKF;
    mnLastKeyFrameId = mpCurrentFrame->mnId;
    mpLastKeyFrame = pKF;

    mnSecondMap++;
    return;

    /*
    auto allKeyFrames = mpMap->GetAllKeyFrames();
    auto allMapPoints = mpMap->GetAllMapPoints();

    int ndiscardKF = 0;
    int ndiscardMP = 0;

    // scale 所有[地图Map]中关键帧, 并且 SetBadFlag 那些不在局部地图中的关键帧
    for(auto itKF = allKeyFrames.begin(); itKF != allKeyFrames.end(); itKF++) {
        KeyFramePtr pKF = *itKF;
        if(!pKF || pKF->isBad()) continue;
        if(pKF->mbScaled) continue;
        pKF->SetScale(scale);
        pKF->mbScaled = true;
        pKF->mbConnectedToSecondMap = true;
    }

    // cale 所有地图中的3D点， 并且 SetBadFlag 那些不在局部地图中的地图点
    for(auto itMP = allMapPoints.begin(); itMP != allMapPoints.end(); itMP++) {
        MapPointPtr pMP = *itMP;
        if(!pMP || pMP->isBad()) continue;
        if(pMP->mbScaled) continue;
        pMP->SetScale(scale);
        pMP->mbScaled = true;
        pMP->UpdateNormalAndDepth();
    }

    ORBmatcher matcher;
    set<MapPointPtr> spAlreadyFoundMPs = pKF->GetMapPoints();
    int matches = matcher.SearchByProjection(pKF,
                                             allMapPoints,
                                             spAlreadyFoundMPs,
                                             3, 64);
    cout << "additional match in new scale: "<<  matches << endl;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    mpMap->SetReferenceKeyFrames(mvpLocalKeyFrames);
    mpCurrentFrame->SetPose(pKF->GetPose());
    mpMapDrawer->SetCurrentCameraPose(mpCurrentFrame->mTcw);
    // mpMapDrawer->SetScale(scale);

    mpFrameDrawer->Update(shared_from_this());
    mVelocity = cv::Mat();

    mnLastKeyFrameId = pKF->mnId;
    mpLastKeyFrame = pKF;
    mpReferenceKF = pKF;
    mpCurrentFrame->mpReferenceKF = pKF;
    mpLastFrame = mpCurrentFrame;

    mpLocalMapper->Release();
    mpLocalMapper->SetScale(scale);
    mpLocalMapper->InsertKeyFrame(pKF);
    mnSecondMap++;

    for(auto pKF : mpMap->GetAllKeyFrames()) pKF->mbScaled = false;
    for(auto pMP : mpMap->GetAllMapPoints()) pMP->mbScaled = false;
    */

}
void Tracking::CreateSecondMapMultical(const int& camS, double& scale)
{

    // scale = 3.0;
    cout << S::yellow << __FUNCTION__ << S::endc << endl;
    mpLocalMapper->RequestStop();
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // 清空localmapping中的帧
    mpLocalMapper->EmptyLocalMapper();

    cout << S::blue << "LocalMapping is stopped. Start scaling map: " << scale << endl;
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    // #######################################
    mvpLocalKeyFrames.clear();
    mvpLocalMapPoints.clear();
    // #######################################

    int  nCrossCamMP = 0;

    cout << "Current Frame id: " << mpCurrentFrame->mnId << endl;
    int nFrames = mvpFrameForSecondMap.size();
    for(int i = 0; i < nFrames; i++) {
        auto pF = mvpFrameForSecondMap[i];
        KeyFramePtr pKF = make_shared<KeyFrame>(pF, mpMap, mpKeyFrameDB);
        pKF->ComputeBoW();

        for(size_t i = 0 ; i < pF->mvpMapPoints.size(); i++) {
            auto pMP =  pF->mvpMapPoints[i];
            if( !pMP ) continue;
            if(pMP->isBad()) continue;
            int camid = pF->keypointToCam[i];
            if(camid != 0) cout<<"*";cout.flush();
            pMP->AddObservation(pKF, i);
            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();
        }
        pKF->UpdateConnections();
        pKF->mbConnectedToSecondMap = true;
        pF->mpRelocedKF->UpdateConnections();
        pF->mpRelocedKF->mbConnectedToSecondMap = true;

        mvpKeyFrameForSecondMap.push_back(pKF);
        mvpLocalKeyFrames.push_back(pKF);
        // 保证mvpLocalKeyFrame 中没有重复
        if(pF->mpRelocedKF->mnTrackSecondMapForFrame != mpCurrentFrame->mnId)
            mvpLocalKeyFrames.push_back(pF->mpRelocedKF);

        pKF->mnTrackSecondMapForFrame = mpCurrentFrame->mnId;
        pF->mpRelocedKF->mnTrackSecondMapForFrame = mpCurrentFrame->mnId;
        mpMap->AddKeyFrame(pKF);

        // 拿到pF的所有共识的关键帧
        map<KeyFramePtr ,int> keyframeCounter;
        for(int i=0; i<pF->totalN; i++)
        {
            MapPointPtr  pMP = pF->mvpMapPoints[i];
            if( ! pMP) continue;
            if( pMP->isBad()) {
                pF->mvpMapPoints[i]=NULL;
                continue;
            }
            //int camid = mpCurrentFrame->keypointToCam[i];
            const map<KeyFramePtr ,size_t> observations = pMP->GetObservations();
            for(map<KeyFramePtr ,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
            {
                if(it->first->mnId == pKF->mnId) continue; // 去掉当前的KF， 因为自己和自己共识一定最高
                keyframeCounter[it->first]++;
            }

        }

        // 6.1 accquire pKFreloced max covis
        int max = 0;
        KeyFramePtr  pKFmax= static_cast<KeyFramePtr >(NULL);
        for(map<KeyFramePtr ,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
        {
            KeyFramePtr  pKF = it->first;
            if(pKF->isBad()) continue;
            if(it->second>max)
            {
                max=it->second;
                pKFmax=pKF;
            }
        }

        // 所有有共识的关键帧都添加到localKeyFrame中
        for(map<KeyFramePtr ,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
        {
            KeyFramePtr  pKF = it->first;
            int counter = it->second;
            if(pKF->isBad()) continue;
            // 保证 mvpLocalKeyFrame 中没有重复
            if(pKF->mnTrackSecondMapForFrame == mpCurrentFrame->mnId) continue;
            if(counter < 0.6 * max && !pKF->mbConnectedToSecondMap) continue;

            // cout << "kfid: " << it->first->mnId << "   covis #" << it->second << " is added to the local map" << endl;
            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackSecondMapForFrame = mpCurrentFrame->mnId;
        }

        cout << endl;
        cout << "processing frame id: " << pF->mnId << endl;
        cout << "relocing KF id: " << pKF->mnId << " ( constructed from processing frame) " << endl;
        cout << "reloced KF id " << pF->mpRelocedKF->mnId << endl;

        map<KeyFramePtr ,int> keyframeCounter2;
        auto vpMapPoints = pF->mpRelocedKF->GetMapPointMatches();
        for(int i=0; i<int(vpMapPoints.size()); i++)
        {
            MapPointPtr  pMP = vpMapPoints[i];
            if( ! pMP) continue;
            if( pMP->isBad()) {
                pF->mvpMapPoints[i]=NULL;
                continue;
            }
            //int camid = mpCurrentFrame->keypointToCam[i];
            const map<KeyFramePtr ,size_t> observations = pMP->GetObservations();
            for(map<KeyFramePtr ,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
            {
                if(it->first->mnId == pF->mpRelocedKF->mnId) continue; // 去掉当前的KF， 因为自己和自己共识一定最高
                keyframeCounter2[it->first]++;
            }

        }

        int max2 = 0;
        for(map<KeyFramePtr ,int>::const_iterator it=keyframeCounter2.begin(), itEnd=keyframeCounter2.end(); it!=itEnd; it++)
        {
            KeyFramePtr  pKF = it->first;
            if(pKF->isBad()) continue;
            if(it->second>max2)
            {
                max2=it->second;
            }
        }

        // 所有有共识的关键帧都添加到localKeyFrame中
        for(map<KeyFramePtr ,int>::const_iterator it=keyframeCounter2.begin(), itEnd=keyframeCounter2.end(); it!=itEnd; it++)
        {
            KeyFramePtr  pKF = it->first;
            int counter = it->second;
            if(pKF->isBad()) continue;
            // 保证 mvpLocalKeyFrame 中没有重复
            if(pKF->mnTrackSecondMapForFrame == mpCurrentFrame->mnId) continue;
            if(counter < 0.6 * max && !pKF->mbConnectedToSecondMap)  continue;

            // cout << "kfid: " << it->first->mnId << "   covis #" << it->second << " is added to the local map" << endl;
            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackSecondMapForFrame = mpCurrentFrame->mnId;
        }

    }

    // 将 mvpLocalKeyFrames中 所有的帧看到的地图点加入到 mvpLocalMapPoints 中
    for(auto itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFramePtr  pKF = *itKF;
        vector<MapPointPtr > vpMPs = pKF->GetMapPointMatches();

        for(auto itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPointPtr  pMP = *itMP;
            if(!pMP) continue;
            if(pMP->isBad()) continue;
            if(pMP->mnTrackSecondMapForFrame == mpCurrentFrame->mnId) continue;
            mvpLocalMapPoints.push_back(pMP);
            pMP->mnTrackSecondMapForFrame = mpCurrentFrame->mnId;

        }
    }

    auto allKeyFrames = mpMap->GetAllKeyFrames();
    auto allMapPoints = mpMap->GetAllMapPoints();

    int ndiscardKF = 0;
    int ndiscardMP = 0;

    // scale 所有[地图Map]中关键帧, 并且 SetBadFlag 那些不在局部地图中的关键帧
    for(auto itKF = allKeyFrames.begin(); itKF != allKeyFrames.end(); itKF++) {
        KeyFramePtr pKF = *itKF;
        if(!pKF || pKF->isBad()) continue;
        if(pKF->mnTrackSecondMapForFrame != mpCurrentFrame->mnId) {
            cout << "-";cout.flush();
            pKF->SetBadFlag();
            ndiscardKF++;
            continue;
        }
        if(pKF->mbScaled) continue;
        pKF->SetScale(scale);
        pKF->mbScaled = true;
        pKF->mbConnectedToSecondMap = true;
    }

    // cale 所有地图中的3D点， 并且 SetBadFlag 那些不在局部地图中的地图点
    for(auto itMP = allMapPoints.begin(); itMP != allMapPoints.end(); itMP++) {
        MapPointPtr pMP = *itMP;
        if(!pMP || pMP->isBad()) continue;
        if(pMP->mnTrackSecondMapForFrame != mpCurrentFrame->mnId) {
            pMP->SetBadFlag();
            ndiscardMP++;
            continue;
        }
        if(pMP->mbScaled) continue;
        pMP->SetScale(scale);
        pMP->mbScaled = true;
        pMP->UpdateNormalAndDepth();
    }

    cout << S::blue << "Scaling map finished! " << S::endc << endl;
    cout << S::blue << "discard " << ndiscardKF << " KFs " << ndiscardMP << " MPs" << S::endc << endl;
    // cout << "current KFt after scale " << pKFrelocC->GetCameraCenter() << "\n\n";cout.flush();

    //################################################################
    // 10. 在新的Scale下做一次Search
    KeyFramePtr pKFLastInSecondMap = mvpKeyFrameForSecondMap.back();
    KeyFramePtr pKFFirstInSecondMap = mvpKeyFrameForSecondMap.front();

    ORBmatcher matcher;
    for(auto itKF = mvpKeyFrameForSecondMap.begin(); itKF != mvpKeyFrameForSecondMap.end(); itKF++) {
        auto pKF = *itKF;
        set<MapPointPtr> spAlreadyFoundMPs = pKF->GetMapPoints();
        int matches = matcher.SearchByProjection(pKF,
                                                 mvpLocalMapPoints,
                                                 spAlreadyFoundMPs,
                                                 3, 64);
        cout << "additional match in new scale: "<<  matches << endl;
    }

    Optimizer::GlobalBundleAdjustemnt(mpMap, 10, pKFLastInSecondMap->mnId);

    //################################################################
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    mpMap->SetReferenceKeyFrames(mvpLocalKeyFrames);
    mpCurrentFrame->SetPose(pKFLastInSecondMap->GetPose());
    mpMapDrawer->SetCurrentCameraPose(mpCurrentFrame->mTcw);
    // mpMapDrawer->SetScale(scale);

    mpFrameDrawer->Update(shared_from_this());
    mVelocity = cv::Mat();


    mnLastKeyFrameId = mpCurrentFrame->mnId;
    mpLastKeyFrame = pKFLastInSecondMap;
    mpReferenceKF = pKFLastInSecondMap;
    mpCurrentFrame->mpReferenceKF = pKFLastInSecondMap;
    mpLastFrame = mpCurrentFrame;

    if( ! mbIsMapScaled){
        mnFirstScaleKFId = pKFFirstInSecondMap->mnId;
        mbIsMapScaled = true;
    }
    mpLocalMapper->Release();
    mpLocalMapper->SetScale(scale);
    for(size_t i = 0; i < mvpKeyFrameForSecondMap.size(); i++) {
        auto pKF = mvpKeyFrameForSecondMap[i];
        pKF->mbMapScaled = true;
        mpLocalMapper->InsertKeyFrame(pKF);
    }
    mpLocalMapper->mLastCrossCamRelocingKFID = pKFLastInSecondMap->mnId;

    mnSecondMap += NUM_FRAME_IN_SECONDMAP;

    for(auto pKF : mpMap->GetAllKeyFrames()) pKF->mbScaled = false;
    for(auto pMP : mpMap->GetAllMapPoints()) pMP->mbScaled = false;
    mvpFrameForSecondMap.clear();
    mvpKeyFrameForSecondMap.clear();
    mvRelocSoldierScale.clear();
    mnRelocSoldiers = 0;


}


unsigned int Tracking::GetLastKeyFrameID()
{
    return mpLastKeyFrame->mnId;
}




bool Tracking::RelocalizationPartialOnCam(const int& camS, double& scale)
{

    mpCurrentFrame->ComputeBoW();

    // 在 keyframeDatabase 中的所有Cap相机内 寻找当前的 mpCurrentFrame 的c相机的重定位候选
    vector<KeyFramePtr > vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidatesForCam(mpCurrentFrame, camS, CAP);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolverPtr> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPointPtr > > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFramePtr pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            // 把 mpCurrentFrame 在第c个相机上的观测 在 pKF 的第CAP个相机的观测中寻找匹配
            int nmatches = matcher.SearchByBoWCrossCam(mpCurrentFrame, camS, pKF, CAP, vvpMapPointMatches[i]);
            if(nmatches<50)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                // cout << S::lblue << " enough matches: " << nmatches << " ";
                PnPsolverPtr  pSolver = make_shared<PnPsolver>(*mpCurrentFrame,vvpMapPointMatches[i], camS);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;

            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    // 构造一个零时的Frame
    FramePtr pinF = make_shared<Frame>(); // innner Frame of mpCurrentFrame at camera c
    pinF->mvTotalKeysUn = mpCurrentFrame->mvvkeysUnTemp[camS];
    pinF->mvvkeysUnTemp = mpCurrentFrame->mvvkeysUnTemp;
    pinF->mvN.resize(mpCurrentFrame->mvN.size(), 0);

    int nkpinF = mpCurrentFrame->mvN[camS];
    pinF->totalN = nkpinF;
    pinF->mvpMapPoints.resize(nkpinF, static_cast<MapPointPtr>(NULL));
    pinF->mvInvLevelSigma2 = mpCurrentFrame->mvInvLevelSigma2;
    pinF->mvInvScaleFactors = mpCurrentFrame->mvInvScaleFactors;
    pinF->mvDescriptors = mpCurrentFrame->mvDescriptors;
    pinF->mvScaleFactors = mpCurrentFrame->mvScaleFactors;
    pinF->mnScaleLevels = mpCurrentFrame->mnScaleLevels;
    pinF->mvGrids = mpCurrentFrame->mvGrids;
    pinF->mvExtrinsics = mpCurrentFrame->mvExtrinsics;
    pinF->mvExtAdj = mpCurrentFrame->mvExtAdj;
    for(int i = 0; i < int(pinF->mvExtrinsics.size()); i++) {
        auto Tidentity = Eigen::Isometry3d::Identity();;
        pinF->mvExtrinsics[i] = Converter::toCvMat(Tidentity.matrix());
        pinF->mvExtAdj[i] = mpCurrentFrame->mvExtAdj[0];
    }

    for(int k = 0; k < nkpinF; k++) {
        pinF->keypointToCam[k] = camS;
        pinF->keypointToCamLocal[k] = k;
    }
    pinF->mvbOutlier.resize(nkpinF, true);


    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolverPtr  pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {

                Tcw.copyTo(pinF->mTcw);
                set<MapPointPtr > sFound;

                const int np = vbInliers.size();  // = 第c个相机特征点的 特征点数目

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        pinF->mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else{
                        if(pinF->mvpMapPoints[j]){
                            pinF->mvpMapPoints[j]=static_cast<MapPointPtr>(NULL);
                        }
                    }

                }

                int nGood = Optimizer::PoseOptimization(pinF);

                // cout << S::lblue << __FUNCTION__ <<  " nGoods: " << nGood <<" ";cout.flush();

                if(nGood<10)
                    continue;

                for(int io =0; io<pinF->totalN; io++)
                    if(pinF->mvbOutlier[io])
                        if(pinF->mvpMapPoints[io])
                            pinF->mvpMapPoints[io]=static_cast<MapPointPtr>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again

                if(nGood<60)
                {
                    int nadditional =matcher2.SearchByProjectionOnCam(pinF, camS, vpCandidateKFs[i], sFound,10,100);

                    if(nadditional+nGood>=60)
                    {
                        nGood = Optimizer::PoseOptimization(pinF);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<60)
                        {
                            sFound.clear();
                            for(int ip =0; ip<pinF->totalN; ip++)
                                if(pinF->mvpMapPoints[ip])
                                    sFound.insert(pinF->mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjectionOnCam(pinF, camS, vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=70)
                            {
                                nGood = Optimizer::PoseOptimization(pinF);

                                for(int io =0; io<pinF->totalN; io++)
                                    if(pinF->mvbOutlier[io])
                                        if(pinF->mvpMapPoints[io])
                                            pinF->mvpMapPoints[io]=static_cast<MapPointPtr>(NULL);
                            }
                        }
                    }
                }

                // cout << S::endc << " ";
                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=70)
                {
                    /**
                    int nm = vvpMapPointMatches[i].size();
                    vector<pair<cv::KeyPoint, cv::KeyPoint>> kpCorresp;
                    int nFound = 0;
                    for(int j = 0; j < nm; j++) {
                        auto pMP = pinF->mvpMapPoints[j];
                        if ( ! pMP) continue;
                        nFound++;
                        auto obsvs = pMP->GetObservations();
                        for(auto it = obsvs.begin(); it!= obsvs.end(); it++) {
                            auto pKFobs = it->first;
                            if (pKFobs != vpCandidateKFs[i]) continue;
                            if (pKFobs == vpCandidateKFs[i]) {
                                int index = it->second;
                                cv::KeyPoint kpKF = pKFobs->mvTotalKeysUn[index];
                                int indexFatC = j;
                                cv::KeyPoint kpFatC = mpCurrentFrame->mvvkeysUnTemp[c][indexFatC];
                                kpCorresp.push_back(make_pair(kpFatC, kpKF));
                                break;
                            }
                        }
                    }
                    vector<cv::Mat> imgs;
                    imgs.push_back(mpCurrentFrame->mvImages[c]);
                    imgs.push_back(vpCandidateKFs[i]->mvImages[0]);
                    auto imsMerge = Converter::jointImage(imgs);
                    cv::cvtColor(imsMerge, imsMerge, CV_GRAY2RGB);
                    cv::RNG rng(time(0));
                    for(auto it = kpCorresp.begin(); it != kpCorresp.end(); it++) {
                        cv::KeyPoint kpF = it->first;
                        cv::Point Fp = cv::Point(kpF.pt.x, kpF.pt.y);
                        cv::KeyPoint kpKF = it->second;
                        cv::Point KFp = cv::Point(640 + kpKF.pt.x, kpKF.pt.y);
                        auto color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
                        cv::circle(imsMerge, Fp, 3, color);
                        cv::circle(imsMerge, KFp, 3, color);
                        cv::line(imsMerge, Fp, KFp, color);
                    }

                    cv::imwrite("queryF" + to_string(mpCurrentFrame->mnId) + "_responseKF" + to_string(vpCandidateKFs[i]->mnId)+".jpg", imsMerge);
                    */

                    cout << S::purple << __FUNCTION__ <<  "middle nGood :" << nGood; cout.flush();
                    ORBmatcher matcher3(0.8,true);
                    auto vpNeighKF = vpCandidateKFs[i]->GetConnectedKeyFrames();
                    int nadditional2 = 0;
                    for(auto pKF : vpNeighKF){
                        if(!pKF || pKF->isBad()) continue;
                        sFound.clear();
                        for(int ip =0; ip<pinF->totalN; ip++)
                            if(pinF->mvpMapPoints[ip])
                                sFound.insert(pinF->mvpMapPoints[ip]);
                        nadditional2 += matcher3.SearchByProjectionOnCam(pinF, camS, pKF, sFound, 3, 64);
                    }

                    nGood = Optimizer::PoseOptimization(pinF);
                    cout << S::red << " FINAL nGood" << nGood <<  S::endc;
                    bMatch = true;
                    mpCurrentFrame->mpRelocedKF = vpCandidateKFs[i];
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        scale = 1;
        return false;
    }
    else
    {
        cout << S::green <<  " Success ";
        int count = 0;
        for(int i = 0; i < pinF->totalN; i++) {
            if(pinF->mvbOutlier[i]) continue;
            if( ! pinF->mvpMapPoints[i]) continue;
            int localKpIdx = i;
            int globalKpIdx = 0;
            for(int ic = 0; ic < camS; ic++) globalKpIdx += mpCurrentFrame->mvN[ic];
            globalKpIdx += localKpIdx;
            if(localKpIdx !=  (int)mpCurrentFrame->keypointToCamLocal[globalKpIdx]) exit(-1);
            // assert();
            mpCurrentFrame->mvpMapPoints[globalKpIdx] = pinF->mvpMapPoints[i];
            count++;
        }
        cout << count << " MapPoints reloced " << S::endc << endl;

        mpMapDrawer->SetCurrentCameraPose(mpCurrentFrame->mTcw);
        mpMapDrawer->SetRelocCameraPose(pinF->mTcw);

        auto centerS = pinF->GetCameraCenter();
        auto centerC = mpCurrentFrame->GetCameraCenter();
        cv::Mat deltaCenter = centerS - centerC;
        Eigen::Vector3d egtsc_map(deltaCenter.at<float>(0,0), deltaCenter.at<float>(1,0), deltaCenter.at<float>(2,0));

        //        auto egtsc_map = egtc_w - egts_w;
        double dotInMap = pow(egtsc_map.dot(egtsc_map), 0.5);
        cout << S::purple << "Extrinsic length in map: " << dotInMap << S::endc << endl;
        auto Tsc = mpCameras->getExtrinsici(camS);
        auto egTsc = Converter::toMatrix4d(Tsc);
        Eigen::Vector3d egtsc(egTsc(0,3), egTsc(1,3), egTsc(2,3));
        double dotInWorld = pow(egtsc.dot(egtsc), 0.5);
        cout << S::purple << "Extrinsic in world" << egtsc.transpose() << S::endc << endl;
        cout << S::purple << "Extrinsic length in world: " << dotInWorld << "m" <<  S::endc << endl;
        scale = dotInWorld / dotInMap;
        cout << S::purple << "scale" << scale << S::endc << endl;
        return true;
    }

}

bool Tracking::Relocalization()
{
    if(mbCompulsoryLost) return false;

    // Compute Bag of Words Vector
    mpCurrentFrame->ComputeBoW();
    cout << S::red << __FUNCTION__ << " at Frame# " << mpCurrentFrame->mnId << S::endc << endl;

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFramePtr > vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidatesForCam(mpCurrentFrame, CAP, CAP);

    if(vpCandidateKFs.empty())
        return false;

    int nKFs = vpCandidateKFs.size();

    vector<KeyFramePtr> vpRemainCandidateKFs;
    int nWorkingCams = 0;
    if( mbIsMapScaled) {
        nWorkingCams = mnCameras;
        for(int i = 0; i < nKFs; i++) {
            auto pKF = vpCandidateKFs[i];
            if(pKF->mbMapScaled) vpRemainCandidateKFs.push_back(pKF);
        }
    } else {
        nWorkingCams = 1;
        vpRemainCandidateKFs = vpCandidateKFs;
    }

    if (vpRemainCandidateKFs.empty()) return false;
    nKFs = vpRemainCandidateKFs.size();

    cout << "\t Candidates: " << nKFs << endl;

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolverPtr> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<  vector<MapPointPtr>   > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFramePtr pKF = vpCandidateKFs[i];

        if(!pKF || pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            // 先只搜索CAP
            int nmatches = matcher.SearchByBoWCrossCam(mpCurrentFrame, CAP, pKF, CAP, vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolverPtr  pSolver = make_shared<PnPsolver>(*mpCurrentFrame,vvpMapPointMatches[i], CAP);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolverPtr  pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mpCurrentFrame->mTcw);

                set<MapPointPtr > sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        size_t globalkpId = mpCurrentFrame->GetGlobalIdxByLocal(j, CAP);
                        mpCurrentFrame->mvpMapPoints[globalkpId]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else{
                        if(mpCurrentFrame->mvpMapPoints[j]){
                            mpCurrentFrame->mvpMapPoints[j]=static_cast<MapPointPtr>(NULL);
                        }
                    }

                }

                int nGood = Optimizer::PoseOptimization(mpCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mpCurrentFrame->totalN; io++)
                    if(mpCurrentFrame->mvbOutlier[io])
                        if(mpCurrentFrame->mvpMapPoints[io])
                            mpCurrentFrame->mvpMapPoints[io]=static_cast<MapPointPtr>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                int nAdditional = 0;
                for(int ic = 0; ic < mnCameras; ic++) {
                    if(ic != 0  && ! mbIsMapScaled) continue;
                    int add =matcher2.SearchByProjectionOnCam(mpCurrentFrame,
                                                              ic,
                                                              vpCandidateKFs[i],
                                                              sFound,
                                                              10,
                                                              100);
                    nAdditional += add;
                }

                cout << S::red << "\tfirst Search by Proj add: " << nAdditional << " MPs" << S::endc << endl;


                if(nAdditional+nGood >= 60)
                {
                    nGood = Optimizer::PoseOptimization(mpCurrentFrame);

                    // If many inliers but still not enough, search by projection again in a narrower window
                    // the camera has been already optimized with many points
                    if(nGood> 30 && nGood< nWorkingCams * 60)
                    {
                        int nAdditional = 0;
                        for(int ic = 0; ic < mnCameras; ic++) {
                            if(ic != 0  && ! mbIsMapScaled) continue;
                            sFound.clear();
                            for(int ip = 0; ip < mpCurrentFrame->mvN[ic]; ip++) {
                                size_t globalip = mpCurrentFrame->GetGlobalIdxByLocal(ip, ic);
                                auto pMP = mpCurrentFrame->mvpMapPoints[globalip];
                                if( !pMP ) continue;
                                sFound.insert(pMP);
                            }
                            int add = matcher2.SearchByProjectionOnCam(mpCurrentFrame,
                                                                       ic,
                                                                       vpCandidateKFs[i],
                                                                       sFound,
                                                                       3,
                                                                       64);
                            nAdditional += add;
                        }

                        cout << S::red << "\tSecond Search by Proj add: " << nAdditional << " MPs" << S::endc << endl;

                        // Final optimization
                        if(nGood+nAdditional >= 70)
                        {
                            nGood = Optimizer::PoseOptimization(mpCurrentFrame);

                            for(int io =0; io<mpCurrentFrame->totalN; io++)
                                if(mpCurrentFrame->mvbOutlier[io])
                                    if(mpCurrentFrame->mvpMapPoints[io])
                                        mpCurrentFrame->mvpMapPoints[io]=static_cast<MapPointPtr>(NULL);

                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood >= 70)
                {
#ifdef DEBUG
                    unordered_map<int, int> CamMPs;
                    auto vpMPsCurt = mpCurrentFrame->mvpMapPoints;
                    int totalMP = 0;
                    for(size_t i = 0; i < vpMPsCurt.size(); i++){
                        auto pMP = vpMPsCurt[i];
                        if(!pMP || pMP->isBad()) continue;
                        int camid = mpCurrentFrame->keypointToCam[i];
                        CamMPs[camid]++;
                        totalMP++;
                    }
                    cout << S::red << "\tAfter Reloc Curt F" << mpCurrentFrame->mnId << " has " << totalMP << " MPs " << S::endc;cout.flush();
                    for(auto cam : CamMPs) cout << "cam " << cam.first << " has " << cam.second << " MPs ";
                    cout << endl;
#endif
                    // mpReferenceKF = vpCandidateKFs[i];
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastLostRelocFrameId = mpCurrentFrame->mnId;
        return true;
    }

}

void Tracking::sleep_ms(unsigned int secs) {


    struct timeval tval;

    tval.tv_sec=secs/1000;

    tval.tv_usec=(secs*1000)%1000000;

    select(0,NULL,NULL,NULL,&tval);


}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mpLastFrame->totalN; i++)
    {
        MapPointPtr  pMP = mpLastFrame->mvpMapPoints[i];

        if(pMP)
        {
            MapPointPtr  pRep = pMP->GetReplaced();
            if(pRep)
            {
                mpLastFrame->mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // cout << S::blue << __FUNCTION__ << endl;
    // Compute Bag of Words vector
    mpCurrentFrame->ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPointPtr > vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpCurrentFrame,
                                       mpReferenceKF,
                                       vpMapPointMatches,
                                       mbIsMapScaled);

    if(nmatches<15)
        return false;

    mpCurrentFrame->mvpMapPoints = vpMapPointMatches;

    // if(mState == FULL)
    //     mpCurrentFrame->SaveImageWithMatches("AftSechBow_");

    mpCurrentFrame->SetPose(mpReferenceKF->GetPose());

    Optimizer::PoseOptimization(mpCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    int discard = 0;

    for(int i =0; i<mpCurrentFrame->totalN; i++)
    {
        if(mpCurrentFrame->mvpMapPoints[i])
        {
            if(mpCurrentFrame->mvbOutlier[i])
            {
                MapPointPtr  pMP = mpCurrentFrame->mvpMapPoints[i];

                mpCurrentFrame->mvpMapPoints[i]=static_cast<MapPointPtr >(NULL);
                mpCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mpCurrentFrame->mnId;
                nmatches--;
                discard++;
            }
            else if(mpCurrentFrame->mvpMapPoints[i]->Observations()>0)
            {
                nmatchesMap++;
            }

        }
    }

#ifdef DEBUG
    if(mState == FULL){
        const vector<MapPointPtr > vpMapPointsKF = mpCurrentFrame->mvpMapPoints;
        unordered_map<int, int> CamMPs;
        int totalMP = 0;
        for(size_t i = 0; i < vpMapPointsKF.size(); i++){
            auto pMP = vpMapPointsKF[i];
            if(!pMP || pMP->isBad()) continue;
            int camid = mpCurrentFrame->keypointToCam[i];
            CamMPs[camid]++;
            totalMP++;
        }
        cout << S::purple << "\tAft Sech Bow and Opt, F" << mpCurrentFrame->mnId << " has " << totalMP << " MPs " << endl;
        for(auto cam : CamMPs) cout << "\t Fcam " << cam.first << " has " << cam.second << " MPs ";
        cout << S::endc << endl;
        // mpCurrentFrame->SaveImageWithMatches("AftSechBow&Opt_");
    }
#endif
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFramePtr  pRef = mpLastFrame->mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mpLastFrame->SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mpLastFrame->mnId || mSensor==System::DUAL || !mbOnlyTracking)
        return;

}

bool Tracking::TrackWithMotionModel()
{
    // if(mState == FULL) cout << S::blue << __FUNCTION__ << endl;
    ORBmatcher matcher(0.8,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mpCurrentFrame->SetPose(mVelocity * mpLastFrame->mTcw);
    //    mpMapDrawer->SetCurrentCameraPose(mpCurrentFrame->mTcw);

    fill(mpCurrentFrame->mvpMapPoints.begin(),
         mpCurrentFrame->mvpMapPoints.end(),
         static_cast<MapPointPtr >(NULL));

    // Project points seen in previous frame
    int th=7;
    int nmatches = matcher.SearchByProjection(mpCurrentFrame,
                                              mpLastFrame,
                                              th,
                                              mbIsMapScaled);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mpCurrentFrame->mvpMapPoints.begin(),
             mpCurrentFrame->mvpMapPoints.end(),
             static_cast<MapPointPtr >(NULL));
        nmatches = matcher.SearchByProjection(mpCurrentFrame,
                                              mpLastFrame,
                                              2*th,
                                              mbIsMapScaled);
    }

    if(nmatches<20)
        return false;

    // if(mState == FULL) {
    //     mpCurrentFrame->SaveImageWithMatches("AftProj_");
    // }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(mpCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mpCurrentFrame->totalN; i++)
    {
        if(mpCurrentFrame->mvpMapPoints[i])
        {
            if(mpCurrentFrame->mvbOutlier[i])
            {
                MapPointPtr  pMP = mpCurrentFrame->mvpMapPoints[i];

                mpCurrentFrame->mvpMapPoints[i]=static_cast<MapPointPtr >(NULL);
                mpCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mpCurrentFrame->mnId;
                nmatches--;
            }
            else if(mpCurrentFrame->mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

#ifdef DEBUG
    if(mState == FULL) {
        unordered_map<int, int> CamMPs;
        auto vpMPsCurt = mpCurrentFrame->mvpMapPoints;
        int totalMP = 0;
        for(size_t i = 0; i < vpMPsCurt.size(); i++){
            auto pMP = vpMPsCurt[i];
            if(!pMP || pMP->isBad()) continue;
            int camid = mpCurrentFrame->keypointToCam[i];
            CamMPs[camid]++;
            totalMP++;
        }
        cout << S::green << "\tAfter Proj and Opt Curt F" << mpCurrentFrame->mnId << " has " << totalMP << " MPs " << S::endc;cout.flush();
        for(auto cam : CamMPs) cout << "cam " << cam.first << " has " << cam.second << " MPs ";
        cout << endl;
        // mpCurrentFrame->SaveImageWithMatches("AftProj&Opt_");
    }
#endif

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    // if(mState == FULL) cout << S::blue << __FUNCTION__ << S::endc << endl;
    UpdateLocalMap();
    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(mpCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mpCurrentFrame->totalN; i++)
    {
        if(mpCurrentFrame->mvpMapPoints[i])
        {
            if(!mpCurrentFrame->mvbOutlier[i])
            {
                mpCurrentFrame->mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mpCurrentFrame->mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mpCurrentFrame->mnId<mnLastLostRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    if(!mbIsMapScaled && mpCurrentFrame->mnId == mnLastRelocFrameId) return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mpCurrentFrame->mnId<mnLastLostRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::DUAL)
        thRefRatio = 0.95f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mpCurrentFrame->mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mpCurrentFrame->mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::DUAL && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::DUAL)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFramePtr  pKF = make_shared<KeyFrame>(mpCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mpCurrentFrame->mpReferenceKF = pKF;


    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mpCurrentFrame->mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(auto vit=mpCurrentFrame->mvpMapPoints.begin(), vend=mpCurrentFrame->mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPointPtr  pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPointPtr >(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mpCurrentFrame->mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(auto vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPointPtr  pMP = *vit;
        if(pMP->mnLastFrameSeen == mpCurrentFrame->mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mpCurrentFrame->isInFrustum(pMP, 0.5, mbIsMapScaled))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

#ifdef DEBUG
    auto vpMP = mpCurrentFrame->mvpMapPoints;
    unordered_map<int, int> CamMPs;
    for(size_t i = 0; i < vpMP.size(); i++ ) {
        auto pMP = vpMP[i];
        if(!pMP || pMP->isBad()) continue;
        int c = mpCurrentFrame->keypointToCam[i];
        CamMPs[c]++;
    }

    if(mState == FULL) {
        cout << "\tbefore search in local map: ";cout.flush();
        for(auto cam : CamMPs) cout << "cam " << cam.first << " has " << cam.second << " MPs ";cout.flush();
        cout << endl;
    }
#endif

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mpCurrentFrame->mnId<mnLastLostRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mpCurrentFrame,mvpLocalMapPoints,th);
    }

#ifdef DEBUG
    vpMP = mpCurrentFrame->mvpMapPoints;
    CamMPs.clear();
    for(size_t i = 0; i < vpMP.size(); i++ ) {
        auto pMP = vpMP[i];
        if(!pMP || pMP->isBad()) continue;
        int c = mpCurrentFrame->keypointToCam[i];
        CamMPs[c]++;

    }

    if(mState == FULL) {
        cout << "\tafter search in local map: ";cout.flush();
        for(auto cam : CamMPs) cout << "cam " << cam.first << " has " << cam.second << " MPs ";cout.flush();
        cout << endl;
    }

    // if(mState == FULL) {
    //     mpCurrentFrame->SaveImageWithMatches("aftSechLocMap_");
    // }
#endif

}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    mpMap->SetReferenceKeyFrames(mvpLocalKeyFrames);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{

    unique_lock<mutex> lock(mMutexLocalMapPoints);
    mvpLocalMapPoints.clear();

    for(vector<KeyFramePtr >::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFramePtr  pKF = *itKF;
        const vector<MapPointPtr > vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPointPtr >::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPointPtr  pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mpCurrentFrame->mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
            }
        }
    }
}

vector<MapPointPtr> Tracking::GetLocalMapPoints()
{
    unique_lock<mutex> lock(mMutexLocalMapPoints);
    return mvpLocalMapPoints;
}


void Tracking::UpdateLocalKeyFrames(int thCounter)
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFramePtr ,int> keyframeCounter;
    for(int i=0; i<mpCurrentFrame->totalN; i++)
    {
        if(mpCurrentFrame->mvpMapPoints[i])
        {
            MapPointPtr  pMP = mpCurrentFrame->mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFramePtr ,size_t> observations = pMP->GetObservations();
                for(map<KeyFramePtr ,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mpCurrentFrame->mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFramePtr  pKFmax= static_cast<KeyFramePtr >(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFramePtr ,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFramePtr  pKF = it->first;
        int counter = it->second;

        if(pKF->isBad()) continue;
        if(thCounter != 0 && counter < thCounter) continue;
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mpCurrentFrame->mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFramePtr >::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFramePtr  pKF = *itKF;

        const vector<KeyFramePtr > vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFramePtr >::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFramePtr  pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mpCurrentFrame->mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
                    break;
                }
            }
        }

        const set<KeyFramePtr > spChilds = pKF->GetChilds();
        for(set<KeyFramePtr >::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFramePtr  pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mpCurrentFrame->mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
                    break;
                }
            }
        }

        KeyFramePtr  pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mpCurrentFrame->mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mpCurrentFrame->mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mpCurrentFrame->mpReferenceKF = mpReferenceKF;
    }
}


void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        //        delete mpInitializer;
        mpInitializer = static_cast<InitializerPtr >(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mnRelocSoldiers = 0;
    mvRelocSoldierScale.clear();
    mReloccScale = 1.0;
    mbIsMapScaled = false;
    mnLastRelocFrameId = 0;
    mnLastLostRelocFrameId = 0;
    mnFirstScaleKFId = 0;
    mnSecondMap = 0;

    if(mpViewer)
        mpViewer->Release();
}


void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        if(mpCurrentFrame->mvvkeysTemp[CAP].size() < 100) return;
        // Set Reference Frame

        mpInitialFrame = mpCurrentFrame;
        mpLastFrame = mpCurrentFrame;
        mvbPrevMatched.resize(mpCurrentFrame->mvTotalKeysUn.size());
        for(size_t i=0; i<mpCurrentFrame->mvTotalKeysUn.size(); i++)
            mvbPrevMatched[i]=mpCurrentFrame->mvTotalKeysUn[i].pt;

        mpInitializer =  make_shared<Initializer>(*mpCurrentFrame,1.0,200);
        mnReadyForInit = 1;
        fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
        return;

    }
    else
    {
        // Try to initialize
        if( (int)mpCurrentFrame->mvvkeysTemp[CAP].size() < 100 ) {
            mpInitializer = static_cast<InitializerPtr>(NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            mnReadyForInit = 0;
            return;
        }
        mnReadyForInit = 2;
    }

    if(mnReadyForInit == 2){
        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mpInitialFrame,mpCurrentFrame,
                                                       mvbPrevMatched,mvIniMatches,100);

        // cout << "nmatches " << nmatches << endl;
        // Check if there are enough correspondences
        if(nmatches<100)
        {
            // delete mpInitializer;
            mpInitializer = static_cast<InitializerPtr>(NULL);
            mnReadyForInit = 0;
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
        bool initSuc = mpInitializer->Initialize(*mpCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated);
        if( ! initSuc) return;

        for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
        {
            if(mvIniMatches[i]>=0 && !vbTriangulated[i])
            {
                mvIniMatches[i]=-1;
                nmatches--;
            }
        }

        // Set Frame Poses
        // 成功初始化后的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
        mpInitialFrame->SetPose(cv::Mat::eye(4,4,CV_32F));
        cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(Tcw.rowRange(0,3).col(3));
        mpCurrentFrame->SetPose(Tcw);

        //将三角化得到的3D点包装成MapPoints
        // Initialize函数会得到mvIniP3D，
        // mvIniP3D是cv::Point3f类型的一个容器，是个存放3D点的临时变量，
        // CreateInitialMapMonocular将3D点包装成MapPoint类型存入KeyFrame和Map中
        CreateInitialMapMonocular();
    }
}

void Tracking::CreateInitialMapMonocular()
{

    // Create KeyFrames
    KeyFramePtr  pKFini = make_shared<KeyFrame>(mpInitialFrame,mpMap,mpKeyFrameDB);
    KeyFramePtr  pKFcur = make_shared<KeyFrame>(mpCurrentFrame,mpMap,mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();
    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPointPtr  pMP = make_shared<MapPoint>(worldPos,pKFcur,mpMap);


        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);
        pMP->SetFirstViewCamera(CAP);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mpCurrentFrame->mvpMapPoints[mvIniMatches[i]] = pMP;
        mpCurrentFrame->mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }


    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << S::green <<  "\n\t\t *** New Map created with " << mpMap->MapPointsInMap() << " points *** \n\n" << S::endc << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);


    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << S::red << "Wrong initialization, reseting..." << S::endc << endl;
        Reset();
        return;
    }


    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPointPtr > vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPointPtr  pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);


    mpCurrentFrame->SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mpCurrentFrame->mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mpCurrentFrame->mpReferenceKF = pKFcur;

    mpLastFrame = mpCurrentFrame;

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);
    mState=OK;

}




} //namespace ORB_SLAM
