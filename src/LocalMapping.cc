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
#include "Cameras.h"
#include "S.h"

#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(MapPtr pMap, KeyFrameDatabasePtr pKFDB):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap), mpKeyFrameDB(pKFDB),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

void LocalMapping::SetLoopCloser(LoopClosingPtr  pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(TrackingPtr pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;


    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
                SearchCrossCameras();
            }

            mbAbortBA = false;

            if (( mbNeedBAUrgent || !CheckNewKeyFrames()) && !stopRequested()) {
                // Local BA
                if(mpMap->KeyFramesInMap()>= 2) {
                    size_t fixId = 0;
                    if(mpTracker->mbIsMapScaled)
                        fixId = mpTracker->mnFirstScaleKFId;
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap, fixId);
                }
                mbNeedBAUrgent = false;
                // Check redundant local Keyframes
                KeyFrameCulling();
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(4000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(4000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFramePtr pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}

void LocalMapping::EmptyLocalMapper()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    for(list<KeyFramePtr>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++){
        auto pKF = *lit;
        pKF = static_cast<KeyFramePtr>(NULL);
    }

    mlNewKeyFrames.clear();

    cout << "Local Mapping CLEAR" << endl;
}

bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    //    if(mlNewKeyFrames.size()!= 0) cout << __FUNCTION__ << " " << mlNewKeyFrames.size() << endl;
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPointPtr > vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPointPtr  pMP = vpMapPointMatches[i];
        if(! pMP || pMP->isBad()) continue;

        if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
        {
            pMP->AddObservation(mpCurrentKeyFrame, i);
            pMP->UpdateNormalAndDepth();
            pMP->ComputeDistinctiveDescriptors();
        }
        else // this can only happen for new stereo points inserted by the Tracking
        {
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    if( ! mpMap->IsKeyFrameInMap(mpCurrentKeyFrame))
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPointPtr >::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;


    int nThObs = 2;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPointPtr  pMP = *lit;
        if(pMP->isBad())
        {
            // 步骤1：已经是坏点的MapPoints直接从检查链表中删除
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            // 跟踪到该MapPoint的Frame数相比预计可观测到该MapPoint的Frame数的比例需大于25%
            // IncreaseFound / IncreaseVisible < 25%，注意不一定是关键帧。
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            // VI-B 条件2：从该点建立开始，到现在已经过了不小于2个关键帧
            // 但是观测到该点的关键帧数却不超过cnThObs帧，那么该点检验不合格
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            // 步骤4：从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点
            // 因此没有SetBadFlag()，仅从队列中删除，放弃继续对该MapPoint的检测
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}


void LocalMapping::SetScale(const double& scale)
{
    vector<KeyFramePtr> vpScaledKF;
    vector<MapPointPtr> vpScaledMP;

    for(auto itMP = mlpRecentAddedMapPoints.begin(); itMP != mlpRecentAddedMapPoints.end(); itMP++)
    {
        MapPointPtr pMP = *itMP;
        if(!pMP || pMP->isBad()) continue;
        if(pMP->mbScaled) continue;
        pMP->SetScale(scale);
        pMP->UpdateNormalAndDepth();
        pMP->mbScaled = true;
    }

    for(auto itKF = mlNewKeyFrames.begin(); itKF != mlNewKeyFrames.end(); itKF++)
    {
        KeyFramePtr pKF = *itKF;
        if(!pKF || pKF->isBad()) continue;
        if(pKF->mbScaled) continue;
        pKF->SetScale(scale);
        pKF->mbScaled = true;
    }

    for(auto pKF : vpScaledKF) pKF->mbScaled = false;
    for(auto pMP : vpScaledMP) pMP->mbScaled = false;

}


void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 20;
    const vector<KeyFramePtr > vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);


    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFramePtr  pKF2 = vpNeighKFs[i];

        for(int ic = 0; ic < mpTracker->mnCameras; ic++){

            // Check first that baseline is not too short
            cv::Mat Ocw1 = mpCurrentKeyFrame->GetCameraCenter(ic);
            cv::Mat Ocw2 = pKF2->GetCameraCenter(ic);
            cv::Mat vBaseline = Ocw2-Ocw1;
            const float baseline = cv::norm(vBaseline);

            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;

            // CAP 相机始终是一直在生成3D点的
            bool case1 = (ic == 0);
            // 当前相机不是Cap同时系统尺度已经回复了，并且当前相机冲定位成功了
            bool case2 = ic != 0 && mpTracker->mbIsMapScaled;

            int nnew=0;

            if(case1 || case2) {
                // Compute Fundamental Matrix
                cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2, ic);
                // Search matches that fullfil epipolar constraint
                // 步骤5：通过极线约束限制匹配时的搜索范围，进行特征点匹配
                vector< pair<size_t,size_t> > vMatchedIndices;
                matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,ic);

                cv::Mat Osw1 = mpCurrentKeyFrame->GetCameraCenter(ic);
                cv::Mat Osw2 = pKF2->GetCameraCenter(ic);

                cv::Mat Rsw1 = mpCurrentKeyFrame->GetRotation(ic);
                cv::Mat Rws1 = Rsw1.t();
                cv::Mat tsw1 = mpCurrentKeyFrame->GetTranslation(ic);
                cv::Mat Tsw1(3,4,CV_32F);
                Rsw1.copyTo(Tsw1.colRange(0,3));
                tsw1.copyTo(Tsw1.col(3));

                const float &fx1 = mpCurrentKeyFrame->mvfx[ic];
                const float &fy1 = mpCurrentKeyFrame->mvfy[ic];
                const float &cx1 = mpCurrentKeyFrame->mvcx[ic];
                const float &cy1 = mpCurrentKeyFrame->mvcy[ic];
                const float &invfx1 = mpCurrentKeyFrame->mvinvfx[ic];
                const float &invfy1 = mpCurrentKeyFrame->mvinvfy[ic];

                cv::Mat Rsw2 = pKF2->GetRotation(ic);
                cv::Mat Rws2 = Rsw2.t();
                cv::Mat tsw2 = pKF2->GetTranslation(ic);
                cv::Mat Tsw2(3,4,CV_32F);
                Rsw2.copyTo(Tsw2.colRange(0,3));
                tsw2.copyTo(Tsw2.col(3));

                const float &fx2 = pKF2->mvfx[ic];
                const float &fy2 = pKF2->mvfy[ic];
                const float &cx2 = pKF2->mvcx[ic];
                const float &cy2 = pKF2->mvcy[ic];
                const float &invfx2 = pKF2->mvinvfx[ic];
                const float &invfy2 = pKF2->mvinvfy[ic];
                const int nmatches = vMatchedIndices.size();

                vector<bool> vbNewMPCreated(nmatches, false);
                for(int ikp = 0; ikp < nmatches; ikp++) {
                    const int &idx1 = vMatchedIndices[ikp].first;
                    const int &idx2 = vMatchedIndices[ikp].second;
                    const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvTotalKeysUn[idx1];
                    const cv::KeyPoint &kp2 = pKF2->mvTotalKeysUn[idx2];

                    // 步骤6.2：利用匹配点反投影得到视差角
                    // 特征点反投影 // 反投影到归一化平面
                    cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
                    cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

                    cv::Mat ray1 = Rws1*xn1;
                    cv::Mat ray2 = Rws2*xn2;

                    const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

                    cv::Mat x3D;
                    if(cosParallaxRays>0 && cosParallaxRays<0.9998) {
                        cv::Mat A(4,4,CV_32F);
                        A.row(0) = xn1.at<float>(0)*Tsw1.row(2)-Tsw1.row(0);
                        A.row(1) = xn1.at<float>(1)*Tsw1.row(2)-Tsw1.row(1);
                        A.row(2) = xn2.at<float>(0)*Tsw2.row(2)-Tsw2.row(0);
                        A.row(3) = xn2.at<float>(1)*Tsw2.row(2)-Tsw2.row(1);

                        cv::Mat w,u,vt;
                        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                        x3D = vt.row(3).t();

                        if(x3D.at<float>(3)==0) continue;

                        // Euclidean coordinates
                        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
                    } else {
                        continue;
                    }
                    cv::Mat x3Dt = x3D.t();
                    float z1 = Rsw1.row(2).dot(x3Dt)+tsw1.at<float>(2);
                    if(z1<=0) continue;
                    float z2 = Rsw2.row(2).dot(x3Dt)+tsw2.at<float>(2);
                    if(z2<=0) continue;

                    //Check reprojection error in first keyframe
                    // 步骤6.6：计算3D点在当前关键帧下的重投影误差
                    const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                    const float x1 = Rsw1.row(0).dot(x3Dt)+tsw1.at<float>(0);
                    const float y1 = Rsw1.row(1).dot(x3Dt)+tsw1.at<float>(1);
                    const float invz1 = 1.0/z1;

                    float u1 = fx1*x1*invz1+cx1;
                    float v1 = fy1*y1*invz1+cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                        continue;

                    const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                    const float x2 = Rsw2.row(0).dot(x3Dt)+tsw2.at<float>(0);
                    const float y2 = Rsw2.row(1).dot(x3Dt)+tsw2.at<float>(1);
                    const float invz2 = 1.0/z2;

                    float u2 = fx2*x2*invz2+cx2;
                    float v2 = fy2*y2*invz2+cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                        continue;
                    //Check scale consistency
                    cv::Mat normal1 = x3D-Osw1;
                    float dist1 = cv::norm(normal1);

                    cv::Mat normal2 = x3D-Osw2;
                    float dist2 = cv::norm(normal2);

                    if(dist1==0 || dist2==0)
                        continue;

                    const float ratioDist = dist2/dist1;
                    const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]
                            / pKF2->mvScaleFactors[kp2.octave];
                    if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                        continue;

                    // Triangulation is succesfull
                    MapPointPtr  pMP = make_shared<MapPoint>(x3D,mpCurrentKeyFrame,mpMap);
                    pMP->AddObservation(mpCurrentKeyFrame,idx1);
                    pMP->SetFirstViewCamera(ic);
                    pMP->AddObservation(pKF2,idx2);

                    mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
                    pKF2->AddMapPoint(pMP,idx2);
                    pMP->ComputeDistinctiveDescriptors();

                    pMP->UpdateNormalAndDepth();

                    mpMap->AddMapPoint(pMP);
                    mlpRecentAddedMapPoints.push_back(pMP);
                    vbNewMPCreated[ikp] = true;

                    nnew++;
                }

                if(ic != 0 && nnew >= 30) {
                    vector<cv::Mat> imgs;
                    imgs.push_back(mpCurrentKeyFrame->mvImages[ic]);
                    imgs.push_back(pKF2->mvImages[ic]);
                    auto img = Converter::jointImage(imgs);
                    cv::cvtColor(img, img, CV_GRAY2RGB);
                    cv::RNG rng(time(0));

                    for(auto it = vMatchedIndices.begin(); it != vMatchedIndices.end(); it++) {
                        size_t globalidx1 = it->first;
                        size_t globalidx2 = it->second;

                        const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvTotalKeysUn[globalidx1];
                        const cv::KeyPoint &kp2 = pKF2->mvTotalKeysUn[globalidx2];

                        cv::Point KF1p = cv::Point(kp1.pt.x, kp1.pt.y);
                        cv::Point KF2p = cv::Point(640 + kp2.pt.x, kp2.pt.y);
                        auto color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
                        cv::circle(img, KF1p, 3, color);
                        cv::circle(img, KF2p, 3, color);
                        cv::line(img, KF1p, KF2p, color);
                    }

                    //cv::imwrite("queryKF" + to_string(mpCurrentKeyFrame->mnId) + "cam" + to_string(ic)
                    //    + "_responseKF" + to_string(pKF2->mnId) + "cam" + to_string(ic)
                    //    +".jpg", img);
                    //cout << "KFid" << mpCurrentKeyFrame->mnId << " cam" << ic << " added " << nnew << " MPs" << endl;
                }
            }
        }

    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 20;
    const vector<KeyFramePtr > vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFramePtr > vpTargetKFs;
    for(vector<KeyFramePtr >::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFramePtr  pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFramePtr > vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFramePtr >::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFramePtr  pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPointPtr > vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFramePtr >::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFramePtr  pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPointPtr > vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFramePtr >::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFramePtr  pKFi = *vitKF;

        vector<MapPointPtr > vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPointPtr >::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPointPtr  pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPointPtr  pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}


void LocalMapping::SearchCrossCameras()
{
    // 地图还没还原尺度
    if( ! mpTracker->mbIsMapScaled ) return;
    // 距离上次结果哦还没有过去5个关键帧
    if(mpCurrentKeyFrame->mnId <= mLastCrossCamRelocingKFID + 5) return;

    auto pCurtF = mpCurrentKeyFrame->mpFrame;
    pCurtF->ComputeBoW();
    // 0 相机在 1 相机的地图中搜寻重定位

    auto spConnectedKF = mpCurrentKeyFrame->GetConnectedKeyFrames();

    vector<KeyFramePtr> vpCandidateKFs =
            mpKeyFrameDB->DetectRelocalizationCandidatesForCam(pCurtF, 0, 1);
    if(vpCandidateKFs.empty()) return;
    int nKFs = vpCandidateKFs.size();

    vector<KeyFramePtr> vpRemainKFs;
    for(int i=0; i < nKFs; i++) {
        auto pKF = vpCandidateKFs[i];
        if(!pKF || pKF->isBad()) continue;
        if(!pKF->mbMapScaled)continue;
        if(!spConnectedKF.count(pKF)) continue;
        vpRemainKFs.push_back(pKF);
    }
    if(vpRemainKFs.empty()) return;
    nKFs = vpRemainKFs.size();



    ORBmatcher matcher(0.75,true);
    vector<PnPsolverPtr> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPointPtr > > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;
    for(int i = 0; i < nKFs; i++ ) {
        auto pKF = vpRemainKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            // 把 pCurtF 在第c个相机上的观测 在 pKF 的第CAP个相机的观测中寻找匹配
            int nmatches = matcher.SearchByBoWCrossCam(pCurtF, 0, pKF, 1, vvpMapPointMatches[i]);
            if(nmatches<50)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                // cout << S::lblue << " enough matches: " << nmatches << " ";
                PnPsolverPtr  pSolver = make_shared<PnPsolver>(*pCurtF, vvpMapPointMatches[i], 0);
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

    // ############### 构造一个inner Frame 用来搜索更多的匹配 ###################
    FramePtr pinF = make_shared<Frame>(); // innner Frame of pF at camera 0
    pinF->mvTotalKeysUn = pCurtF->mvvkeysUnTemp[0];
    pinF->mvvkeysUnTemp = pCurtF->mvvkeysUnTemp;
    pinF->mvN.resize(pCurtF->mvN.size(), 0);

    int nkpinF = pCurtF->mvN[0];
    pinF->totalN = nkpinF;
    pinF->mvpMapPoints.resize(nkpinF, static_cast<MapPointPtr>(NULL));
    pinF->mvInvLevelSigma2 = pCurtF->mvInvLevelSigma2;
    pinF->mvInvScaleFactors = pCurtF->mvInvScaleFactors;
    pinF->mvDescriptors = pCurtF->mvDescriptors;
    pinF->mvScaleFactors = pCurtF->mvScaleFactors;
    pinF->mnScaleLevels = pCurtF->mnScaleLevels;
    pinF->mvGrids = pCurtF->mvGrids;
    pinF->mvExtrinsics = pCurtF->mvExtrinsics;
    pinF->mvExtAdj = pCurtF->mvExtAdj;
    for(int i = 0; i < int(pinF->mvExtrinsics.size()); i++) {
        auto Tidentity = Eigen::Isometry3d::Identity();;
        pinF->mvExtrinsics[i] = Converter::toCvMat(Tidentity.matrix());
        pinF->mvExtAdj[i] = pCurtF->mvExtAdj[0];
    }
    for(int k = 0; k < nkpinF; k++) {
        pinF->keypointToCam[k] = 0;
        pinF->keypointToCamLocal[k] = k;
    }
    pinF->mvbOutlier.resize(nkpinF, true);
    // ############### end 构造一个inner Frame 用来搜索更多的匹配 ###################

    KeyFramePtr pKFsuccess;
    while(nCandidates>0 && !bMatch) {
        for(int i = 0; i < nKFs; i++) {
            if( vbDiscarded[i]) continue;
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            auto pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
            if(bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            if(! Tcw.empty()) {
                Tcw.copyTo(pinF->mTcw);
                set<MapPointPtr > sFound;
                const int np = vbInliers.size();  // = 第0个相机特征点的 特征点数目
                for(int j=0; j<np; j++){
                    if(vbInliers[j]){
                        pinF->mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }else{
                        if(pinF->mvpMapPoints[j]){
                            pinF->mvpMapPoints[j]=static_cast<MapPointPtr>(NULL);
                        }
                    }
                }
                int nGood = Optimizer::PoseOptimization(pinF);
                if(nGood < 10) continue;
                for(int io =0; io<pinF->totalN; io++)
                    if(pinF->mvbOutlier[io])
                        if(pinF->mvpMapPoints[io])
                            pinF->mvpMapPoints[io]=static_cast<MapPointPtr>(NULL);
                if(nGood<60)
                {
                    int nadditional =matcher2.SearchByProjectionOnCam(pinF,
                                                                      0,
                                                                      vpRemainKFs[i],
                                                                      sFound,10,100);

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
                            nadditional =matcher2.SearchByProjectionOnCam(pinF,
                                                                          0,
                                                                          vpRemainKFs[i],
                                                                          sFound,3,64);

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
                if(nGood>=70){
                    bMatch = true;
                    pKFsuccess = vpRemainKFs[i];
                    break;
                }
            }
        }
    }

    if( ! bMatch) return;

    int replaced = 0;
    int newadded = 0;
    for(int i = 0; i < pinF->totalN; i++) {
        if(pinF->mvbOutlier[i]) continue;
        auto pMP1 = pinF->mvpMapPoints[i];
        if( ! pMP1 || pMP1->isBad()) continue;
        size_t localKpIdx = i;
        size_t globalKpIdx = mpCurrentKeyFrame->GetGlobalIdxByLocal(localKpIdx, 0);
        auto pMP2 = mpCurrentKeyFrame->GetMapPoint(globalKpIdx);
        if( ! pMP2 || pMP2->isBad()) {
            pMP1->AddObservation(mpCurrentKeyFrame, globalKpIdx);
            mpCurrentKeyFrame->AddMapPoint(pMP1, globalKpIdx);
            newadded++;
        } else {
            pMP2->Replace(pMP1);
            replaced++;
        }

    }

    cout << "LocalMapping SearchCrossCameras replaced: " << replaced << " added: " << newadded << endl;
    // Update KeyFrame
    mpCurrentKeyFrame->UpdateConnections();

    // Update MPs
    auto vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        auto  pMP = vpMapPointMatches[i];
        if(! pMP ||  pMP->isBad()) continue;
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
    }

    mbNeedBAUrgent = true;
    mLastCrossCamRelocingKFID = mpCurrentKeyFrame->mnId;
    mLastCrossCamRelocedKFID = pKFsuccess->mnId;

#ifdef DEBUG2
    /**
    vector<cv::Mat> imgs;
    cv::Mat img;
    for(int i = 0; i < nKFs; i++) {
        if(vbDiscarded[i]) continue;
        imgs.clear();
        imgs.push_back(mpCurrentKeyFrame->mvImages[0]);
        imgs.push_back(vpRemainKFs[i]->mvImages[1]);
        img = Converter::jointImage(imgs);
        cv::imwrite("lOCALMAPPING_query"+to_string(mpCurrentKeyFrame->mnId)+"cam0"+
                    " resp"+to_string(vpRemainKFs[i]->mnId)+"cam1.jpg", img);

    }
    */
#endif
    return;
}

cv::Mat LocalMapping::ComputeF12(KeyFramePtr &pKF1, KeyFramePtr &pKF2, int cs)
{

    Eigen::Matrix4d egTcscc = Converter::toMatrix4d(mpTracker->mpCameras->getExtrinsici(cs));

    // 获得KF1相机cs到世界的R，t
    cv::Mat T1ccw = pKF1->GetPose();
    Eigen::Matrix4d egT1ccw = Converter::toMatrix4d(T1ccw);
    Eigen::Matrix4d egT1csw = egTcscc * egT1ccw;
    Eigen::Isometry3d eigT1csw(egT1csw);

    Eigen::Matrix3d egR1csw = eigT1csw.rotation();
    Eigen::Vector3d egt1csw = eigT1csw.translation();

    //    cout << "T1ccw \n" << T1ccw << "\n-----------------------\n\n";
    //    cout << "egT1csw \n" << egT1csw << "\n-----------------------\n\n";
    //    cout << "egR1csw \n" << egR1csw << "\n-----------------------\n\n";
    //    cout << "egt1csw \n" << egt1csw << "\n-----------------------\n\n";

    cv::Mat R1csw = Converter::toCvMat(egR1csw);
    cv::Mat t1csw = Converter::toCvMat(egt1csw);

    // 获得KF2相机cs到世界的R，t
    cv::Mat T2ccw = pKF2->GetPose();
    Eigen::Matrix4d egT2ccw = Converter::toMatrix4d(T2ccw);
    Eigen::Matrix4d egT2csw = egTcscc * egT2ccw;
    Eigen::Isometry3d eigT2csw(egT2csw);
    Eigen::Matrix3d egR2csw = eigT2csw.rotation();
    Eigen::Vector3d egt2csw = eigT2csw.translation();

    cv::Mat R2csw = Converter::toCvMat(egR2csw);
    cv::Mat t2csw = Converter::toCvMat(egt2csw);

    cv::Mat R1cs_2cs = R1csw * R2csw.t();
    cv::Mat t1cs_2cs = -R1csw * R2csw.t() * t2csw + t1csw;
    cv::Mat t1cs_2cs_x = SkewSymmetricMatrix(t1cs_2cs);
    const cv::Mat &K1 = pKF1->mvK[cs];
    const cv::Mat &K2 = pKF2->mvK[cs];

    cv::Mat ret = K1.t().inv() * t1cs_2cs_x * R1cs_2cs * K2.inv();
    return ret;

    /*
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &KK1 = pKF1->mvK[cs];
    const cv::Mat &KK2 = pKF2->mvK[cs];

    cv::Mat ret2 = KK1.t().inv()*t12x*R12*KK2.inv();
    cout << "ret2 \n" << ret2 << "\n-----------------------\n\n";
    return KK1.t().inv()*t12x*R12*KK2.inv();
    */

}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFramePtr>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++){
        auto pKF = *lit;
        pKF = static_cast<KeyFramePtr>(NULL);
    }

    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points

    vector<KeyFramePtr > vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFramePtr >::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFramePtr  pKF = *vit;
        if(pKF->mbConnectedToSecondMap) continue;

        const vector<MapPointPtr > vpMapPoints = pKF->GetMapPointMatches();

        const int thObs = 3;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPointPtr  pMP = vpMapPoints[i];
            if(! pMP || pMP->isBad()) continue;
            nMPs++;
            if(pMP->Observations()>thObs)
            {
                const int &scaleLevel = pKF->mvTotalKeysUn[i].octave;
                const map<KeyFramePtr , size_t> observations = pMP->GetObservations();
                int nObs=0;
                for(map<KeyFramePtr , size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                {
                    KeyFramePtr  pKFi = mit->first;
                    if(pKFi==pKF)
                        continue;
                    const int &scaleLeveli = pKFi->mvTotalKeysUn[mit->second].octave;

                    if(scaleLeveli<=scaleLevel+1)
                    {
                        nObs++;
                        if(nObs>=thObs)
                            break;
                    }
                }
                if(nObs>=thObs)
                {
                    nRedundantObservations++;
                }
            }
        }

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<
            0,                -v.at<float>(2),  v.at<float>(1),
            v.at<float>(2),   0,                -v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),   0               );
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
        mLastCrossCamRelocingKFID = -1;
        mLastCrossCamRelocedKFID = -1;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
