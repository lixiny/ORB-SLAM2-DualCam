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
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(
    FramePtr pF,
    MapPtr pMap,
    KeyFrameDatabasePtr pKFDB
):
    mpFrame(pF),
    mbMapScaled(pF->mbMapScaled),
    mnFrameId(pF->mnId),
    mTimeStamp(pF->mTimeStamp),
    mnGridCols(FRAME_GRID_COLS),
    mnGridRows(FRAME_GRID_ROWS),
    mnTrackReferenceForFrame(0),
    mnFuseTargetForKF(0),
    mnBALocalForKF(0),
    mnBAFixedForKF(0),
    mnLoopQuery(0),
    mnLoopWords(0),
    mnRelocQuery(0),
    mnRelocWords(0),
    mnBAGlobalForKF(0),
    
    mpCameras(pF->mpCameras),
    mvK(pF->mvK),
    mvDistCoef(pF->mvDistCoef),
    mvExtrinsics(pF->mvExtrinsics),
    mvExtAdj(pF->mvExtAdj),

    totalN(pF->totalN),
    mvN(pF->mvN),

    mvTotalKeys(pF->mvTotalKeys),
    mvTotalKeysUn(pF->mvTotalKeysUn),
    keypointToCam(pF->keypointToCam),
    keypointToCamLocal(pF->keypointToCamLocal),
    mvvkeysTemp(pF->mvvkeysTemp),
    mvvkeysUnTemp(pF->mvvkeysUnTemp),
    mvDescriptors(pF->mvDescriptors),

    mvImages(pF->mvImages),

    mnScaleLevels(pF->mnScaleLevels),
    mfScaleFactor(pF->mfScaleFactor),
    mfLogScaleFactor(pF->mfLogScaleFactor),
    mvScaleFactors(pF->mvScaleFactors),
    mvInvScaleFactors(pF->mvInvScaleFactors),
    mvLevelSigma2(pF->mvLevelSigma2),
    mvInvLevelSigma2(pF->mvInvLevelSigma2),

    mvfGridElementWidthInv(pF->mvfGridElementWidthInv),
    mvfGridElementHeightInv(pF->mvfGridElementHeightInv),

    mvMinX(pF->mvMinX),
     mvMaxX(pF->mvMaxX),
    mvMinY(pF->mvMinY),
    mvMaxY(pF->mvMaxY),
    
    mvfx(pF->mvfx),
    mvfy(pF->mvfy),
    mvcx(pF->mvcx),
    mvcy(pF->mvcy),
    mvinvfx(pF->mvinvfx),
    mvinvfy(pF->mvinvfy),

    mvpMapPoints(pF->mvpMapPoints),
    mpKeyFrameDB(pKFDB),
    mpORBvocabulary(pF->mpORBvocabulary),
    mbFirstConnection(true),
    mpParent(NULL),
    mbToBeErased(false),
    mbBad(false),
    
    mpMap(pMap)

{
    mnId=nNextId++;
    mnCams = mpCameras->getNCameras();
    mvGrids.resize(mnCams);
    mvBowVec.resize(mnCams);
    mvFeatVec.resize(mnCams);

    for(int c = 0; c < mnCams; c++) {
        mvGrids[c].resize(mnGridCols);
        for(int i = 0; i < mnGridCols; i++) {
            mvGrids[c][i].resize(mnGridRows);
            for (int j = 0; j < mnGridRows; j++)
                mvGrids[c][i][j] = pF->mvGrids[c][i][j];
        }
    }

    ComputeBoW();
    SetPose(pF->mTcw);
}

void KeyFrame::ComputeBoW()
{
    for(int c = 0; c < mnCams; c++) {
        if(mvBowVec[c].empty() || mvFeatVec[c].empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mvDescriptors[c]);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpORBvocabulary->transform(vCurrentDesc,mvBowVec[c],mvFeatVec[c],4);
        }
    }

}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
}


void KeyFrame::SetScale(const double& scale)
{
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat Ttemp = Tcw.clone();
    Ttemp.rowRange(0,3).col(3) *= scale;
    Ttemp.copyTo(Tcw);

    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));

}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter(int cam)
{
    unique_lock<mutex> lock(mMutexPose);
    if(cam == 0){
        return Ow.clone();
    } else {
        cv::Mat ext = mvExtrinsics[cam];
        cv::Mat Tsw = ext * Tcw;
        cv::Mat Rsw = Tsw.rowRange(0,3).colRange(0,3);
        cv::Mat tsw = Tsw.rowRange(0,3).col(3);
        cv::Mat Rws = Rsw.t();
        cv::Mat Osw = -Rws * tsw;
        return Osw.clone();
    }

}

cv::Mat KeyFrame::GetRotation(int cam)
{
    unique_lock<mutex> lock(mMutexPose);
    if(cam == 0) {
        return Tcw.rowRange(0,3).colRange(0,3).clone();
    }else{
        cv::Mat ext = mvExtrinsics[cam];
        cv::Mat Tsw = ext * Tcw;
        return Tsw.rowRange(0,3).colRange(0,3).clone();
    }

}

cv::Mat KeyFrame::GetTranslation(int cam)
{
    unique_lock<mutex> lock(mMutexPose);
    if(cam == 0) {
        return Tcw.rowRange(0,3).col(3).clone();
    } else {
        cv::Mat ext = mvExtrinsics[cam];
        cv::Mat Tsw = ext * Tcw;
        return Tsw.rowRange(0,3).col(3).clone();
    }

}


size_t KeyFrame::GetGlobalIdxByLocal(const size_t& localIdx, const int& cam)
{
    size_t globalKpId = 0;
    for(int ic = 0; ic < cam; ic++) globalKpId += mvN[ic];
    globalKpId += localIdx;
    return globalKpId;
}

void KeyFrame::AddConnection(KeyFramePtr pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFramePtr > > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFramePtr ,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFramePtr > lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFramePtr >(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}

set<KeyFramePtr > KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFramePtr > s;
    for(map<KeyFramePtr ,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFramePtr > KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFramePtr > KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFramePtr >(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFramePtr > KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFramePtr >();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFramePtr >();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFramePtr >(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFramePtr pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPointPtr pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPointPtr >(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPointPtr  pMP)
{
    int idx = pMP->GetIndexInKeyFrame(shared_from_this());
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPointPtr >(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPointPtr  pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPointPtr > KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPointPtr > s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPointPtr  pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<totalN; i++)
    {
        MapPointPtr  pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPointPtr > KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPointPtr  KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFramePtr ,int> KFcounter;

    vector<MapPointPtr > vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPointPtr >::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPointPtr  pMP = *vit;
        if(!pMP) continue;
        if(pMP->isBad()) continue;

        map<KeyFramePtr ,size_t> observations = pMP->GetObservations();

        for(map<KeyFramePtr ,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFramePtr  pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFramePtr > > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFramePtr ,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(shared_from_this(),mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(shared_from_this(),nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFramePtr > lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFramePtr >(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(shared_from_this());
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFramePtr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFramePtr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFramePtr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(shared_from_this());
}

set<KeyFramePtr > KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFramePtr  KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFramePtr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFramePtr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFramePtr > KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{

    {
        unique_lock<mutex> lock(mMutexConnections);

        if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFramePtr ,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(shared_from_this());

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(shared_from_this());
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFramePtr > sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);

        if(sParentCandidates.size() != 0) {
            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            // 如果这个关键帧有自己的孩子关键帧，告诉这些子关键帧，它们的父关键帧不行了，赶紧找新的父关键帧
            while(!mspChildrens.empty())
            {
                bool bContinue = false;

                int max = -1;
                KeyFramePtr  pC;
                KeyFramePtr  pP;

                for(set<KeyFramePtr >::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
                {
                    KeyFramePtr  pKF = *sit;
                    if(pKF->isBad())
                        continue;

                    // Check if a parent candidate is connected to the keyframe
                    // 子关键帧遍历每一个与它相连的关键帧（共视关键帧）
                    vector<KeyFramePtr > vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                    {
                        for(set<KeyFramePtr >::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                        {
                            if(vpConnected[i]->mnId == (*spcit)->mnId)
                            {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if(w>max)
                                {
                                    pC = pKF;
                                    pP = vpConnected[i];
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if(bContinue)
                {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                }
                else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            if(!mspChildrens.empty()) {
                for(set<KeyFramePtr >::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
                {
                    (*sit)->ChangeParent(mpParent);
                }
            }

            mpParent->EraseChild(shared_from_this());
            mTcp = Tcw*mpParent->GetPoseInverse();
            mbBad = true;
        } else { // 当前要Erase的节点没有父节点。
            // 需要从他的所有子节点中找到一个作为新的父节点，其他的子节点更新其父亲
            KeyFramePtr pP;
            int maxGrandChild = 0;
            for(auto sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
                auto pKF = *sit;
                int numGrandChild= pKF->mspChildrens.size();
                if(numGrandChild > maxGrandChild){
                    maxGrandChild = numGrandChild;
                    pP = pKF;
                }
            }

            for(auto sit = mspChildrens.begin(); sit != mspChildrens.end(); sit++) {
                auto pKF = *sit;
                if(pKF->mnId == pP->mnId) {
                    pKF->mpParent = static_cast<KeyFramePtr>(NULL);
                    pKF->mTcp = cv::Mat::eye(4,4,CV_32F);
                } else {
                    pKF->ChangeParent(pP);
                }

            }
            mbBad = true;

        }


    }

    mpMap->EraseKeyFrame(shared_from_this());
    mpKeyFrameDB->erase(shared_from_this());

    // cout << "SetbadFlag end" << endl;
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFramePtr  pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const int& c, const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(mvN[c]);

    const int nMinCellX = max(0,(int)floor((x-mvMinX[c]-r)*mvfGridElementWidthInv[c]));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mvMinX[c]+r)*mvfGridElementWidthInv[c]));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mvMinY[c]-r)*mvfGridElementHeightInv[c]));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mvMinY[c]+r)*mvfGridElementHeightInv[c]));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mvGrids[c][ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvTotalKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const int& c, const float &x, const float &y) const
{
    return (x>=mvMinX[c] && x<mvMaxX[c] && y>=mvMinY[c] && y<mvMaxY[c]);
}


float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPointPtr > vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(mvpMapPoints.size());
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(unsigned i=0; i< mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            MapPointPtr  pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            z = abs(z);
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

} //namespace ORB_SLAM
