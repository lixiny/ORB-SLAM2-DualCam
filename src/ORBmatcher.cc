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

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include<stdint-gcc.h>
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

using namespace std;

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}


bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const KeyFramePtr  pKF2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr<3.84*pKF2->mvLevelSigma2[kp2.octave];
}


/** F在KF上通过Bow寻找匹配
 * @brief ORBmatcher::SearchByBoW
 * @param pF target, 需要被匹配的Frame
 * @param pKF source，被搜寻的KeyFrame
 * @param vpMapPointMatches 匹配后的结果，对应F的 mvTotalKeysUn
 * @param vbSoldierReloc 由对应的Soldier相机是否重定位成功决定是否在 Soldier上寻找
 * @return 匹配数
 */
int ORBmatcher::SearchByBoW(FramePtr pF,
                            KeyFramePtr  pKF,
                            vector<MapPointPtr > &vpMapPointMatches,
                            bool bMapScaled)
{
    const vector<MapPointPtr > vpMapPointsKF = pKF->GetMapPointMatches();

#ifdef DEBUG
    unordered_map<int, int> CamMPs;
    int totalMP = 0;
    for(size_t i = 0; i < vpMapPointsKF.size(); i++){
        auto pMP = vpMapPointsKF[i];
        if(!pMP || pMP->isBad()) continue;
        int camid = pKF->keypointToCam[i];
        CamMPs[camid]++;
        totalMP++;
    }
    cout << S::green << "\tF" << pF->mnId << "'s  RefKF" << pKF->mnId << " has " << totalMP << " MPs " << endl;;
    for(auto cam : CamMPs) cout << "\tKFcam " << cam.first << " has " << cam.second << " MPs ";
    cout << S::endc << endl;
#endif

    vpMapPointMatches = vector<MapPointPtr >(pF->totalN, static_cast<MapPointPtr >(NULL));

    int nmatches = 0;
    int nCams = pKF->mnCams;
    for(int ic = 0; ic < nCams; ic++) {
        if( ic != 0 && ! bMapScaled) continue;
        auto vpInnerMPMatches = vector<MapPointPtr>(pF->mvN[ic], static_cast<MapPointPtr >(NULL));

        int nmatch= SearchByBoWCrossCam(pF, ic, pKF, ic, vpInnerMPMatches);

        // cout << "\tFcam" << ic << " matches: " << nmatch;cout.flush();
        nmatches += nmatch;

        for(size_t ii = 0; ii < vpInnerMPMatches.size(); ii++) {
            int localkpIdF = ii;
            auto pMP = vpInnerMPMatches[ii];
            if(!pMP || pMP->isBad()) continue;
            int globalkpidF = pF->GetGlobalIdxByLocal(localkpIdF, ic);
            vpMapPointMatches[globalkpidF] = pMP;
        }
    }

    // cout << endl;
    return nmatches;
}



/** F的cF相机 在 KF的cKF相机 上通过Bow寻找匹配
 * @brief ORBmatcher::SearchByBoWCrossCam
 * 将pF的第cF个相机上的特征 在 pKF的第cKF个相机上寻找匹配
 * @param pF target, 需要被匹配的Frame
 * @param cF camera at F, 需要被匹配的Frame的相机
 * @param pKF source，被搜寻的KeyFrame
 * @param cKF camera at KF, 被搜寻的KeyFrame的相机
 * @param vpMapPointMatches 匹配结果， 对应 pF在cF相机下的 mvvKeysUnTemp[cF]
 * @return 匹配数
 */
int ORBmatcher::SearchByBoWCrossCam(FramePtr pF,
                                    const int& cF,
                                    KeyFramePtr pKF,
                                    const int& cKF,
                                    vector<MapPointPtr> &vpMapPointMatches)
{
    const vector<MapPointPtr > vpMapPointsKF = pKF->GetMapPointMatches();

    vpMapPointMatches = vector<MapPointPtr >(pF->mvN[cF],static_cast<MapPointPtr >(NULL));

    const DBoW2::FeatureVector &vFeatVecF = pF->mvFeatVec[cF];
    const DBoW2::FeatureVector &vFeatVecKF = pKF->mvFeatVec[cKF];


    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = vFeatVecF.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = vFeatVecF.end();

    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            const vector<unsigned int> vIndicesKFlocal = KFit->second;
            const vector<unsigned int> vIndicesFlocal = Fit->second;

            for(size_t iKF=0; iKF<vIndicesKFlocal.size(); iKF++)
            {
                const unsigned int IdxKFlocal = vIndicesKFlocal[iKF];
                unsigned int IdxKFglobal = pKF->GetGlobalIdxByLocal(IdxKFlocal, cKF);

                MapPointPtr  pMP = vpMapPointsKF[IdxKFglobal];

                if(!pMP)  continue;
                if(pMP->isBad())  continue;

                const cv::Mat &dKF= pKF->mvDescriptors[cKF].row(IdxKFlocal);

                int bestDist1=256;
                int bestIdxFlocal =-1 ;
                int bestDist2=256;

                for(size_t iF=0; iF<vIndicesFlocal.size(); iF++)
                {
                    const unsigned int idxFlocal = vIndicesFlocal[iF];
                    // 如果已经找到过匹配的MapPoint
                    if(vpMapPointMatches[idxFlocal]) continue;

                    const cv::Mat &dF = pF->mvDescriptors[cF].row(idxFlocal);
                    const int dist =  DescriptorDistance(dKF,dF);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxFlocal=idxFlocal;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1<=TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMapPointMatches[bestIdxFlocal] = pMP;

                        const cv::KeyPoint &kp = pKF->mvvkeysUnTemp[cKF][IdxKFlocal];

                        if(mbCheckOrientation)
                        {
                            float rot = kp.angle - pF->mvvkeysUnTemp[cF][bestIdxFlocal].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxFlocal);
                        }
                        nmatches++;
                    }
                }

            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = vFeatVecF.lower_bound(KFit->first);
        }
    }


    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=static_cast<MapPointPtr >(NULL);
                nmatches--;
            }
        }
    }
    // cout << "End ! SearchByBoW(KeyFramePtr  pKF,FramePtr pF, vector<MapPointPtr > &vpMapPointMatches)" << endl;

    return nmatches;
}


int ORBmatcher::SearchByBoWCrossCam(KeyFramePtr pKF1,
                                    const int& c1,
                                    KeyFramePtr pKF2,
                                    const int& c2,
                                    std::vector<MapPointPtr> &vpMatches12)
{
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvvkeysUnTemp[c1];
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mvFeatVec[c1];
    const vector<MapPointPtr> vpMapPoints1 = pKF1->GetMapPointMatches();
    const cv::Mat &Descriptors1 = pKF1->mvDescriptors[c1];

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvvkeysUnTemp[c2];
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mvFeatVec[c2];
    const vector<MapPointPtr> vpMapPoints2 = pKF2->GetMapPointMatches();
    const cv::Mat &Descriptors2 = pKF2->mvDescriptors[c2];

    vpMatches12 = vector<MapPointPtr>(pKF1->mvN[c1],static_cast<MapPointPtr>(NULL));
    vector<bool> vbMatched2(pKF2->mvN[c2],false);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator kf1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator kf2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator kf1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator kf2end = vFeatVec2.end();

    while(kf1it != kf1end && kf2it != kf2end)
    {
        if(kf1it->first == kf2it->first)
        {
            for(size_t i1 = 0, iend1=kf1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1local = kf1it->second[i1];
                size_t idx1global = pKF1->GetGlobalIdxByLocal(idx1local, c1);
                auto pMP1 = vpMapPoints1[idx1global];
                if(!pMP1 || pMP1->isBad()) continue;
                const cv::Mat& d1 = Descriptors1.row(idx1local);
                int bestDist1=256;
                int bestIdx2local =-1 ;
                int bestDist2=256;

                for(size_t i2 = 0, i2end = kf2it->second.size(); i2 < i2end; i2++)
                {
                    const size_t idx2local = kf2it->second[i2];
                    size_t idx2global = pKF2->GetGlobalIdxByLocal(idx2local, c2);
                    auto pMP2 = vpMapPoints2[idx2global];
                    if(vbMatched2[idx2local] || !pMP2 || pMP2->isBad()) continue;
                    const cv::Mat& d2 = Descriptors2.row(idx2local);
                    int dist = DescriptorDistance(d1,d2);
                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdx2local=idx2local;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1 < TH_LOW) {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2)) {
                        size_t bestIdx2global = pKF2->GetGlobalIdxByLocal(bestIdx2local, c2);
                        vpMatches12[idx1local] = vpMapPoints2[bestIdx2global];
                        vbMatched2[bestIdx2local] = true;

                        if(mbCheckOrientation) {
                            float rot = vKeysUn1[idx1local].angle-vKeysUn2[bestIdx2local].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1local);
                        }
                        nmatches++;
                    }
                }
            }
            kf1it++;
            kf2it++;
        } else if (kf1it->first < kf2it->first){
            kf1it = vFeatVec1.lower_bound(kf2it->first);
        } else {
            kf2it = vFeatVec2.lower_bound(kf1it->first);
        }
    }

    if(mbCheckOrientation) {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=static_cast<MapPointPtr>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;


}

int ORBmatcher::SearchByProjection(KeyFramePtr  pKF,
                                   const int & query,
                                   cv::Mat Scq_w,
                                   const vector<MapPointPtr > &vpPoints,
                                   vector<MapPointPtr > &vpMatched,
                                   int th)
{
    // cout << "SearchByProjection" << endl;
    // Get Calibration Parameters for later projection
    const float &fx = pKF->mvfx[query];
    const float &fy = pKF->mvfy[query];
    const float &cx = pKF->mvcx[query];
    const float &cy = pKF->mvcy[query];

    // Decompose Scw
    cv::Mat sRcqw = Scq_w.rowRange(0,3).colRange(0,3);
    const float scqw = sqrt(sRcqw.row(0).dot(sRcqw.row(0)));
    cv::Mat Rcqw = sRcqw/scqw;
    cv::Mat tcqw = Scq_w.rowRange(0,3).col(3)/scqw;
    cv::Mat Ocqw = -Rcqw.t()*tcqw;

    // Set of MapPoints already found in the KeyFrame
    set<MapPointPtr > spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPointPtr >(NULL));

    int nmatches=0;

    // For each Candidate MapPoint Project and Match
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPointPtr  pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        cv::Mat p3Dcq = Rcqw*p3Dw+tcqw;

        // Depth must be positive
        if(p3Dcq.at<float>(2)<0.0)
            continue;

        // Project into Image
        const float invz = 1/p3Dcq.at<float>(2);
        const float x = p3Dcq.at<float>(0)*invz;
        const float y = p3Dcq.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(query,u,v))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ocqw;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndicesLocal = pKF->GetFeaturesInArea(query,u,v,radius);

        if(vIndicesLocal.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndicesLocal.begin(), vend=vIndicesLocal.end(); vit!=vend; vit++)
        {
            const size_t idxLocal = *vit;
            if(vpMatched[idxLocal])
                continue;

            const int &kpLevel= pKF->mvvkeysUnTemp[query][idxLocal].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mvDescriptors[query].row(idxLocal);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idxLocal;
            }
        }

        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    cout << S::blue << "LOOP CLOSING: " << __FUNCTION__ << " nmatches: " << nmatches << S::endc<< endl;
    return nmatches;
}


int ORBmatcher::SearchByProjection(FramePtr pF,
                                   const vector<MapPointPtr > &vpMapPoints,
                                   const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPointPtr  pMP = vpMapPoints[iMP];
        if(!pMP) continue;
        if(!pMP->mbTrackInView) continue;
        if(pMP->isBad()) continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;

        // 这个VIndices 是kp在 mTrackProjCamera local 下的
        const vector<size_t> vIndices =
                pF->GetFeaturesInArea(pMP->mTrackProjCamera,
                                      pMP->mTrackProjX,
                                      pMP->mTrackProjY,
                                      r * pF->mvScaleFactors[nPredictedLevel],
                                      nPredictedLevel-1,nPredictedLevel+1);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestglobalIdx =-1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {

            const size_t localIdxF = *vit;
            size_t globalIdxF = pF->GetGlobalIdxByLocal(localIdxF, pMP->mTrackProjCamera);

            // 如果这个globalIdxF 这里已经有匹配的MP 了，就跳过
            if(pF->mvpMapPoints[globalIdxF])
                if(pF->mvpMapPoints[globalIdxF]->Observations()>0)
                    continue;

            const cv::Mat &d = pF->mvDescriptors[pMP->mTrackProjCamera].row(localIdxF);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = pF->mvvkeysUnTemp[pMP->mTrackProjCamera][localIdxF].octave;
                bestglobalIdx=globalIdxF;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = pF->mvvkeysUnTemp[pMP->mTrackProjCamera][localIdxF].octave;
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            pF->mvpMapPoints[bestglobalIdx]=pMP;
            nmatches++;
        }
    }

    return nmatches;
}


/** current Frame 在 last frame 上寻找匹配
 * @brief ORBmatcher::SearchByProjection
 * @param pCurrentFrame
 * @param pLastFrame
 * @param th
 * @return
 */
int ORBmatcher::SearchByProjection(FramePtr pCurrentFrame,
                                   const FramePtr pLastFrame,
                                   const float th,
                                   bool bMapScaled)
{

#ifdef DEBUG
    if(bMapScaled) {
        unordered_map<int, int> CamMPs;
        auto vpMPsLast = pLastFrame->mvpMapPoints;
        int totalMP = 0;
        for(size_t i = 0; i < vpMPsLast.size(); i++){
            auto pMP = vpMPsLast[i];
            if(!pMP || pMP->isBad()) continue;
            int camid = pLastFrame->keypointToCam[i];
            CamMPs[camid]++;
            totalMP++;
        }
        cout << S::green << "\tLastF" << pLastFrame->mnId << " has " << totalMP << " MPs " << S::endc;cout.flush();
        for(auto cam : CamMPs) cout << "cam " << cam.first << " has " << cam.second << " MPs ";
        cout << endl;
    }
#endif

    int nmatches = 0;
    int nCams = pCurrentFrame->mnCams;
    for(int ic=0; ic<nCams; ic++)
    {
        if( ic != 0 && ! bMapScaled) continue;
        int nmatch = SearchByProjectionOnCam(pCurrentFrame, ic, pLastFrame, th);
        if(nmatch <= 20) {
            nmatches = nmatch;
            break;
        }
        nmatches += nmatch;
    }

#ifdef DEBUG
    if(bMapScaled) {
        unordered_map<int, int> CamMPs;
        auto vpMPsCurt = pCurrentFrame->mvpMapPoints;
        int totalMP = 0;
        for(size_t i = 0; i < vpMPsCurt.size(); i++){
            auto pMP = vpMPsCurt[i];
            if(!pMP || pMP->isBad()) continue;
            int camid = pLastFrame->keypointToCam[i];
            CamMPs[camid]++;
            totalMP++;
        }
        cout << S::green << "\tAfter Projection Curt F" << pCurrentFrame->mnId << " has " << totalMP << " MPs " << S::endc;cout.flush();
        for(auto cam : CamMPs) cout << "cam " << cam.first << " has " << cam.second << " MPs ";
        cout << endl;
    }
#endif

    return nmatches;
}


int ORBmatcher::SearchByProjection(KeyFramePtr pKF,
                                   const std::vector<MapPointPtr> &vpMapPoints,
                                   const set<MapPointPtr > &sAlreadyFound,
                                   const float th ,
                                   const int ORBdist)
{
    int nmatches = 0;
    int newAdd = 0;
    int replaced = 0;
    const cv::Mat Tcw = pKF->GetPose();
    for (int s = 0; s < pKF->mnCams; s++) {
        const cv::Mat Tsc = pKF->mvExtrinsics[s];
        const cv::Mat Tsw = Tsc * Tcw;

        const cv::Mat Rsw = Tsw.rowRange(0,3).colRange(0,3);
        const cv::Mat tsw = Tsw.rowRange(0,3).col(3);
        const cv::Mat Osw = -Rsw.t()*tsw;

        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++){
            MapPointPtr  pMP = vpMapPoints[i];
            if( ! pMP) continue;
            if( pMP->isBad()) continue;
            if( sAlreadyFound.count(pMP)) continue;

            //Project
            cv::Mat x3Dw = pMP->GetWorldPos();
            cv::Mat x3Ds = Rsw*x3Dw+tsw;

            const float xs = x3Ds.at<float>(0);
            const float ys = x3Ds.at<float>(1);
            const float invzs = 1.0/x3Ds.at<float>(2);

            const float u = pKF->mvfx[s]*xs*invzs+pKF->mvcx[s];
            const float v = pKF->mvfy[s]*ys*invzs+pKF->mvcy[s];

            if(u<pKF->mvMinX[s] || u>pKF->mvMaxX[s])
                continue;
            if(v<pKF->mvMinY[s] || v>pKF->mvMaxY[s])
                continue;

            cv::Mat PO = x3Dw-Osw;
            float dist3D = cv::norm(PO);

            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();

            // Depth must be inside the scale pyramid of the image
            if(dist3D<minDistance || dist3D>maxDistance)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
            const vector<size_t> vIndicesInCam = pKF->GetFeaturesInArea(s,u,v,radius);

            if(vIndicesInCam.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdxglobal = -1;
            MapPointPtr pMPexist;
            for(vector<size_t>::const_iterator vit=vIndicesInCam.begin(), vend=vIndicesInCam.end(); vit!=vend; vit++)
            {
                const size_t localKpIdx = *vit;
                size_t globalKpIdx = pKF->GetGlobalIdxByLocal(localKpIdx, s);

                const int &kpLevel= pKF->mvTotalKeysUn[globalKpIdx].octave;
                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel+1) continue;

                const cv::Mat &d = pKF->mvDescriptors[s].row(localKpIdx);

                const int dist = DescriptorDistance(dMP,d);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxglobal = globalKpIdx;
                    pMPexist = pKF->GetMapPoint(globalKpIdx);

                }
            }

            if(bestDist<=ORBdist)
            {
                if( ! pMPexist || pMPexist->isBad()) {
                    pKF->AddMapPoint(pMP, bestIdxglobal);
                    pKF->UpdateConnections();

                    pMP->AddObservation(pKF, bestIdxglobal);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                    newAdd++;
                } else {
                    pMPexist->Replace(pMP);
                    replaced++;
                }
                nmatches++;
            }

        }
    }
    return nmatches;

}

/** 通过投影在某个相机上寻找匹配
 * @brief ORBmatcher::SearchByProjectionOnCam
 * 将pKF上的所有MapPoint投影到pF的第s个相机上寻找匹配
 * @param pF target，被投影的Frame
 * @param cF soldier camera，被投影的Frame的第cF个相机
 * @param pKF source，用于投影的KeyFrame
 * @param sAlreadyFound
 * @param th
 * @param ORBdist
 * @return 匹配数
 */
int ORBmatcher::SearchByProjectionOnCam(FramePtr pF,
                                        const int& query,
                                        KeyFramePtr pKF,
                                        const set<MapPointPtr > &sAlreadyFound,
                                        const float th ,
                                        const int ORBdist)
{
    // cout << "SearchByProjection" << endl;
    int nmatches = 0;

    const cv::Mat Tsc = pF->mvExtrinsics[query];
    const cv::Mat Tcw = pF->mTcw;
    const cv::Mat Tsw = Tsc * Tcw;

    //    const cv::Mat Rcw = pCurrentFrame->mTcw.rowRange(0,3).colRange(0,3);
    //    const cv::Mat tcw = pCurrentFrame->mTcw.rowRange(0,3).col(3);
    //    const cv::Mat Ow = -Rcw.t()*tcw;

    const cv::Mat Rsw = Tsw.rowRange(0,3).colRange(0,3);
    const cv::Mat tsw = Tsw.rowRange(0,3).col(3);
    const cv::Mat Osw = -Rsw.t()*tsw;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const vector<MapPointPtr > vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPointPtr  pMP = vpMPs[i];
        if( ! pMP) continue;
        if( pMP->isBad()) continue;
        if( sAlreadyFound.count(pMP)) continue;

        //Project
        cv::Mat x3Dw = pMP->GetWorldPos();
        cv::Mat x3Ds = Rsw*x3Dw+tsw;

        const float xs = x3Ds.at<float>(0);
        const float ys = x3Ds.at<float>(1);
        const float invzs = 1.0/x3Ds.at<float>(2);

        const float u = pF->mvfx[query]*xs*invzs+pF->mvcx[query];
        const float v = pF->mvfy[query]*ys*invzs+pF->mvcy[query];

        if(u<pF->mvMinX[query] || u>pF->mvMaxX[query])
            continue;
        if(v<pF->mvMinY[query] || v>pF->mvMaxY[query])
            continue;

        // Compute predicted scale level
        cv::Mat PO = x3Dw-Osw;
        float dist3D = cv::norm(PO);

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist3D,pF);

        // Search in a window
        const float radius = th*pF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndicesFrame = pF->GetFeaturesInArea(query, u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

        if(vIndicesFrame.empty())
            continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestglobalIdx = -1;

        for(vector<size_t>::const_iterator vit=vIndicesFrame.begin(); vit!=vIndicesFrame.end(); vit++)
        {
            const size_t localKpIdx = *vit;
            size_t globalKpIdx = pF->GetGlobalIdxByLocal(localKpIdx, query);
            if(pF->mvpMapPoints[globalKpIdx]) //已经拥有了匹配
                continue;

            const cv::Mat &d = pF->mvDescriptors[query].row(localKpIdx);

            const int dist = DescriptorDistance(dMP,d);

            if(dist<bestDist)
            {
                bestDist=dist;
                // bestIdx记录的是全局的idx ！！！
                bestglobalIdx=globalKpIdx;
            }
        }

        if(bestDist<=ORBdist)
        {
            pF->mvpMapPoints[bestglobalIdx]=pMP;
            nmatches++;

            if(mbCheckOrientation)
            {
                float rot = pKF->mvTotalKeysUn[i].angle - pF->mvTotalKeysUn[bestglobalIdx].angle;
                if(rot<0.0)
                    rot+=360.0f;
                int bin = round(rot*factor);
                if(bin==HISTO_LENGTH)
                    bin=0;
                assert(bin>=0 && bin<HISTO_LENGTH);
                rotHist[bin].push_back(bestglobalIdx);
            }
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    pF->mvpMapPoints[rotHist[i][j]]=NULL;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}


int ORBmatcher::SearchByProjectionOnCam(FramePtr pFcurt,
                                        const int& query,
                                        FramePtr pFlast,
                                        const float th)
{

    int nmatches = 0;

    const cv::Mat Tsc = pFcurt->mvExtrinsics[query];
    const cv::Mat Tcw = pFcurt->mTcw;
    const cv::Mat Tsw = Tsc * Tcw;

    const cv::Mat Rsw = Tsw.rowRange(0,3).colRange(0,3);
    const cv::Mat tsw = Tsw.rowRange(0,3).col(3);
    const cv::Mat Osw = -Rsw.t()*tsw; //  s 相机光心

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const vector<MapPointPtr> vplastMPs = pFlast->mvpMapPoints;

    vector<cv::Mat> imgs;
    imgs.push_back(pFcurt->mvImages[query]);
    imgs.push_back(pFlast->mvImages[query]);

    auto img = Converter::jointImage(imgs);
    cv::cvtColor(img, img, CV_GRAY2RGB);
    cv::RNG rng(time(0));


    for(size_t i=0, iend=vplastMPs.size(); i<iend; i++) {
        // 1. 说明lastFrame观测到MP的相机和 currentFrame待投影到的相机不同
        // 通常情况下我们认为两帧之间的运动不会差别太大。
        // 如果MP在lastFrame上是被第i个相机观测到， 那么在currentFrame也应该在i相机上投影
        if( pFlast->keypointToCam[i] != query) continue;

        MapPointPtr pMP = vplastMPs[i];
        if( ! pMP) continue;
        if( pMP->isBad()) continue;

        cv::Mat x3Dw = pMP->GetWorldPos();
        cv::Mat x3Ds = Rsw*x3Dw+tsw;

        const float xs = x3Ds.at<float>(0);
        const float ys = x3Ds.at<float>(1);
        const float zs = x3Ds.at<float>(2);
        if(zs < 0) continue;
        const float invzs = 1.0/x3Ds.at<float>(2);

        const float u = pFcurt->mvfx[query]*xs*invzs+pFcurt->mvcx[query];
        const float v = pFcurt->mvfy[query]*ys*invzs+pFcurt->mvcy[query];

        if(u<pFcurt->mvMinX[query] || u>pFcurt->mvMaxX[query])
            continue;
        if(v<pFcurt->mvMinY[query] || v>pFcurt->mvMaxY[query])
            continue;

        auto kplast = pFlast->mvTotalKeysUn[i];
        auto x = kplast.pt.x;
        auto y = kplast.pt.y;

        cv::Point Fcurtp = cv::Point(u, v);
        cv::Point Flastp = cv::Point(640 + x, y);

        auto color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
        cv::circle(img, Fcurtp, 2, color);
        cv::circle(img, Flastp, 2, color);
        cv::line(img, Fcurtp, Flastp, color);

        int nLastOctave = pFlast->mvTotalKeysUn[i].octave;

        // Search in a window. Size depends on scale
        float radius = th * pFcurt->mvScaleFactors[nLastOctave];
        cv::circle(img, Fcurtp, radius, color);
        // Search in a window

        const vector<size_t> vIndices = pFcurt->GetFeaturesInArea(query, u, v, radius, nLastOctave-1, nLastOctave+1);
        if(vIndices.empty())
            continue;

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestglobalIdx = -1;

        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t localKpIdx = *vit;
            size_t globalKpIdx = pFcurt->GetGlobalIdxByLocal(localKpIdx, query);

            if(pFcurt->mvpMapPoints[globalKpIdx])
                if(pFcurt->mvpMapPoints[globalKpIdx]->Observations()>0)
                    continue;

            const cv::Mat &d = pFcurt->mvDescriptors[query].row(localKpIdx);

            const int dist = DescriptorDistance(dMP,d);

            if(dist<bestDist)
            {
                bestDist=dist;
                bestglobalIdx = globalKpIdx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            pFcurt->mvpMapPoints[bestglobalIdx] = pMP;
            nmatches++;

            // cout << "-";cout.flush();

            if(mbCheckOrientation)
            {
                float rot = pFlast->mvTotalKeysUn[i].angle - pFcurt->mvTotalKeysUn[bestglobalIdx].angle;
                if(rot<0.0)
                    rot+=360.0f;
                int bin = round(rot*factor);
                if(bin==HISTO_LENGTH)
                    bin=0;
                assert(bin>=0 && bin<HISTO_LENGTH);
                rotHist[bin].push_back(bestglobalIdx);
            }
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    pFcurt->mvpMapPoints[rotHist[i][j]]=static_cast<MapPointPtr>(NULL);
                    nmatches--;
                }
            }
        }
    }

    //    if(s!=0) {
    //        cout << "\tafter search cam" << s << " remains " << nmatches << endl;
    //        cv::imwrite("MotionMode_curtF" + to_string(pFcurt->mnId) + "cam" + to_string(s)
    //                    + "_lastF" + to_string(pFlast->mnId) + "cam" + to_string(s)
    //                    +".jpg", img);
    //    }

    return nmatches;


}



int ORBmatcher::SearchForInitialization(FramePtr pF1,
                                        FramePtr pF2,
                                        vector<cv::Point2f> &vbPrevMatched,
                                        vector<int> &vnMatches12,
                                        int windowSize)
{
    // cout << "SearchForInitialization" << endl;
    int nmatches=0;
    int skips = 0;
    vnMatches12 = vector<int>(pF1->mvTotalKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(pF2->mvTotalKeysUn.size(),INT_MAX);
    vector<int> vnMatches21(pF2->mvTotalKeysUn.size(),-1);

    auto kpmap1 = pF1->keypointToCam; // kpmap 记录了 kp 到 对应的 camera 的映射
    auto kpmap2 = pF2->keypointToCam;
    for(size_t globalKpIdx1=0, iend1=pF1->mvTotalKeysUn.size(); globalKpIdx1<iend1; globalKpIdx1++)
    {
        if( kpmap1[globalKpIdx1] != CAP ) {
            skips++;
            continue;
        }// 我们只要来自0号相机的kp
        cv::KeyPoint kp1 = pF1->mvTotalKeysUn[globalKpIdx1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        // 这个index 是在CAP local下的
        vector<size_t> vIndices2 = pF2->GetFeaturesInArea(CAP, vbPrevMatched[globalKpIdx1].x,vbPrevMatched[globalKpIdx1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        int despIdx = pF1->keypointToCamLocal.find(globalKpIdx1)->second;
        cv::Mat d1 = pF1->mvDescriptors[CAP].row(despIdx);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t localKpIdx2 = *vit;
            size_t globalKpIdx2 = pF2->GetGlobalIdxByLocal(localKpIdx2, CAP);

            int camid = kpmap2[globalKpIdx2];
            if(camid != CAP) exit(-99); //  make a assertion
            int descIdx2 = pF2->keypointToCamLocal.find(globalKpIdx2)->second;
            cv::Mat d2 = pF2->mvDescriptors[camid].row(descIdx2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[globalKpIdx2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=globalKpIdx2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[globalKpIdx1]=bestIdx2;
                vnMatches21[bestIdx2]=globalKpIdx1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = pF1->mvTotalKeysUn[globalKpIdx1].angle - pF2->mvTotalKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(globalKpIdx1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=pF2->mvTotalKeysUn[vnMatches12[i1]].pt;

    //cout << "skip : " << skips << "because not in CAP camera" << endl;
    return nmatches;
}

int ORBmatcher::SearchForTriangulation(KeyFramePtr pKF1, KeyFramePtr pKF2, cv::Mat F12,
                                       vector<pair<size_t, size_t> > &vMatchedPairs, int camS)
{
    // cout << "SearchForTriangulation" << endl;
    const DBoW2::FeatureVector &vFeatVecKF1atCamS = pKF1->mvFeatVec[camS];
    const DBoW2::FeatureVector &vFeatVecKF2atCamS = pKF2->mvFeatVec[camS];

    //Compute epipole in second image
    // 计算KF1的相机中心在KF2图像平面的坐标，即极点坐标
    cv::Mat C1sw = pKF1->GetCameraCenter(camS);
    cv::Mat R2sw = pKF2->GetRotation(camS);
    cv::Mat t2sw = pKF2->GetTranslation(camS);
    cv::Mat C1s2s = R2sw*C1sw+t2sw;
    const float invz = 1.0f/C1s2s.at<float>(2);
    const float ex =pKF2->mvfx[camS]*C1s2s.at<float>(0)*invz+pKF2->mvcx[camS];
    const float ey =pKF2->mvfy[camS]*C1s2s.at<float>(1)*invz+pKF2->mvcy[camS];

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node

    int nmatches=0;
    vector<bool> vbMatched2(pKF2->mvN[camS],false);
    vector<int> vMatches12(pKF1->mvN[camS],-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVecKF1atCamS.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVecKF2atCamS.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVecKF1atCamS.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVecKF2atCamS.end();

    while(f1it!=f1end && f2it!=f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t localidxKF1 = f1it->second[i1];  // 这个index是在KF1相机S下local的
                size_t globalidxKF1 = pKF1->GetGlobalIdxByLocal(localidxKF1, camS);

                MapPointPtr  pMP1 = pKF1->GetMapPoint(globalidxKF1);

                // If there is already a MapPoint skip
                if(pMP1)
                    continue;

                const cv::KeyPoint &kp1 = pKF1->mvvkeysUnTemp[camS][localidxKF1];
                int cam1 = (pKF1->keypointToCam)[globalidxKF1];
                if (cam1 != camS) {
                    cout << "Error asseration in " << __FUNCTION__ << " cam1 == camS" << endl;
                    exit(-1);
                }
                const cv::Mat &d1 = pKF1->mvDescriptors[camS].row(localidxKF1);

                int bestDist = TH_LOW;
                int bestlocalIdx2 = -1;

                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    size_t localidxKF2 = f2it->second[i2];
                    size_t globalidxKF2 = pKF2->GetGlobalIdxByLocal(localidxKF2, camS);

                    MapPointPtr  pMP2 = pKF2->GetMapPoint(globalidxKF2);

                    // If we have already matched or there is a MapPoint skip
                    if(vbMatched2[localidxKF2] || pMP2)
                        continue;

                    int cam2 = (pKF2->keypointToCam)[globalidxKF2];
                    if (cam2 != camS) {
                        cout << "Error asseration in " << __FUNCTION__ << " cam1 == camS" << endl;
                        exit(-1);
                    }
                    const cv::Mat &d2 = pKF2->mvDescriptors[camS].row(localidxKF2);

                    const int dist = DescriptorDistance(d1,d2);

                    if(dist>TH_LOW || dist>bestDist)
                        continue;

                    const cv::KeyPoint &kp2 = pKF2->mvvkeysUnTemp[camS][localidxKF2];

                    const float distex = ex-kp2.pt.x;
                    const float distey = ey-kp2.pt.y;
                    if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                        continue;


                    if(CheckDistEpipolarLine(kp1,kp2,F12,pKF2))
                    {
                        bestlocalIdx2 = localidxKF2;
                        bestDist = dist;
                    }
                }

                if(bestlocalIdx2>=0)
                {
                    const cv::KeyPoint &kp2 = pKF2->mvvkeysUnTemp[camS][bestlocalIdx2];
                    vMatches12[localidxKF1]=bestlocalIdx2;
                    vbMatched2[bestlocalIdx2]=true;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = kp1.angle-kp2.angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(localidxKF1);
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVecKF1atCamS.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVecKF2atCamS.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }

    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0) continue;
        size_t globalidxKF1 = 0;
        size_t globalidxKF2 = 0;
        for(int c = 0; c < camS; c++) {
            globalidxKF1 += pKF1->mvN[c];
            globalidxKF2 += pKF2->mvN[c];
        }
        globalidxKF1 += i;
        globalidxKF2 += vMatches12[i];

        vMatchedPairs.push_back(make_pair(globalidxKF1,globalidxKF2));
    }

    return nmatches;
}



int ORBmatcher::Fuse(KeyFramePtr pKF, const vector<MapPointPtr > &vpMapPoints, const float th)
{
    // cout << "Fuse" << endl;
    int nCams = pKF->mnCams;
    int nFused=0;
    const int nMPs = vpMapPoints.size();
    for(int ic = 0; ic < nCams; ic++) {
        if(ic != 0 && ! pKF->mbMapScaled) continue;
        cv::Mat Rsw = pKF->GetRotation(ic);
        cv::Mat tsw = pKF->GetTranslation(ic);

        const float &fx = pKF->mvfx[ic];
        const float &fy = pKF->mvfy[ic];
        const float &cx = pKF->mvcx[ic];
        const float &cy = pKF->mvcy[ic];
        cv::Mat Os = pKF->GetCameraCenter(ic);

        for(int i = 0; i < nMPs; i++) {
            auto pMP = vpMapPoints[i];
            if(!pMP) continue;
            if(pMP->isBad() || pMP->IsInKeyFrame(pKF)) continue;
            cv::Mat p3Dw = pMP->GetWorldPos();
            cv::Mat p3Ds = Rsw * p3Dw + tsw;
            // Depth must be positive
            if(p3Ds.at<float>(2)<0.0f)  continue;
            const float invz = 1/p3Ds.at<float>(2);
            const float x = p3Ds.at<float>(0)*invz;
            const float y = p3Ds.at<float>(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF->IsInImage(ic,u,v)) continue;
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw-Os;
            const float dist3D = cv::norm(PO);

            // Depth must be inside the scale pyramid of the image
            if(dist3D<minDistance || dist3D>maxDistance )
                continue;

            // Viewing angle must be less than 60 deg
            cv::Mat Pn = pMP->GetNormal();

            if(PO.dot(Pn)<0.5*dist3D)
                continue;

            int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

            // Search in a radius
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndicesLocal = pKF->GetFeaturesInArea(ic,u,v,radius);

            if(vIndicesLocal.empty())
                continue;

            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdxglobal = -1;
            for(vector<size_t>::const_iterator vit=vIndicesLocal.begin(), vend=vIndicesLocal.end(); vit!=vend; vit++)
            {
                const size_t idxLocal = *vit;
                size_t idxglobal = pKF->GetGlobalIdxByLocal(idxLocal, ic);
                size_t local = pKF->keypointToCamLocal[idxglobal];
                if (local != idxLocal) exit(-1);

                const cv::KeyPoint &kp = pKF->mvvkeysUnTemp[ic][idxLocal];

                const int &kpLevel= kp.octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float e2 = ex*ex+ey*ey;

                if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                    continue;

                const cv::Mat &dKF = pKF->mvDescriptors[ic].row(idxLocal);

                const int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxglobal = idxglobal;
                }
            }

            // If there is already a MapPoint replace otherwise add new measurement
            if(bestDist<=TH_LOW)
            {
                MapPointPtr  pMPinKF = pKF->GetMapPoint(bestIdxglobal);

                if(pMPinKF)
                {
                    if(!pMPinKF->isBad())
                    {
                        if(pMPinKF->Observations()>pMP->Observations())
                            pMP->Replace(pMPinKF);
                        else
                            pMPinKF->Replace(pMP);
                    }
                }
                else
                {

                    pMP->AddObservation(pKF,bestIdxglobal);
                    pKF->AddMapPoint(pMP,bestIdxglobal);
                }
                nFused++;
            }

        }
    }

    return nFused;
}



int ORBmatcher::Fuse(KeyFramePtr pKF, cv::Mat Scw, const vector<MapPointPtr > &vpPoints, float th, vector<MapPointPtr > &vpReplacePoint)
{
    // cout << "Fuse" << endl;
    // Get Calibration Parameters for later projection

    // Set of MapPoints already found in the KeyFrame

    const int nCams = pKF->mnCams;
    vector<cv::Mat> vRsw(nCams);
    vector<cv::Mat> vtsw(nCams);
    vector<cv::Mat> vOsw(nCams);


    // for each camera, Decompose Scw and store.
    for(int ic = 0; ic < nCams; ic++) {
        // Decompose Scw
        cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
        cv::Mat Rcw = sRcw/scw;
        cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;

        cv::Mat Tcw = cv::Mat::eye(4,4,Rcw.type());
        Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(Tcw.rowRange(0,3).col(3));

        cv::Mat Tsc = pKF->mpCameras->getExtrinsici(ic);
        cv::Mat Tsw = Tsc * Tcw;

        cv::Mat Rsw = Tsw.rowRange(0,3).colRange(0,3);
        cv::Mat tsw = Tsw.rowRange(0,3).col(3);
        cv::Mat Osw = -Rsw.t()*tsw;

        vRsw[ic] = Rsw.clone();
        vtsw[ic] = tsw.clone();
        vOsw[ic] = Osw.clone();
    }



    const set<MapPointPtr > spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPointPtr  pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        for(int ic = 0; ic < nCams; ic++) {
            const float &fx = pKF->mvfx[ic];
            const float &fy = pKF->mvfy[ic];
            const float &cx = pKF->mvcx[ic];
            const float &cy = pKF->mvcy[ic];

            // Transform into Camera Coords.
            cv::Mat p3Ds = vRsw[ic] * p3Dw + vtsw[ic];
            // Depth must be positive
            if(p3Ds.at<float>(2)<0.0f)
                continue;
            // Project into Image
            const float invz = 1.0/p3Ds.at<float>(2);
            const float x = p3Ds.at<float>(0)*invz;
            const float y = p3Ds.at<float>(1)*invz;

            const float u = fx*x+cx;
            const float v = fy*y+cy;

            // Point must be inside the image
            if(!pKF->IsInImage(ic,u,v))
                continue;

            // Depth must be inside the scale pyramid of the image
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            cv::Mat PO = p3Dw - vOsw[ic];

            const float dist3D = cv::norm(PO);

            if(dist3D<minDistance || dist3D>maxDistance)
                continue;

            cv::Mat Pn = pMP->GetNormal();

            // Viewing angle must be less than 60 deg
            if(PO.dot(Pn)<0.5*dist3D)
                continue;
            // Compute predicted scale level
            const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);
            // Search in a radius
            const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

            const vector<size_t> vIndicesLoc = pKF->GetFeaturesInArea(ic,u,v,radius);
            if(vIndicesLoc.empty())
                continue;

            // Match to the most similar keypoint in the radius
            const cv::Mat dMP = pMP->GetDescriptor();
            int bestDist = INT_MAX;
            int bestIdxGlb = -1;
            for(vector<size_t>::const_iterator vit=vIndicesLoc.begin(); vit!=vIndicesLoc.end(); vit++)
            {
                const size_t idxLoc = *vit;
                const int &kpLevel = pKF->mvvkeysUnTemp[ic][idxLoc].octave;

                if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                    continue;

                const cv::Mat &dKF = pKF->mvDescriptors[ic].row(idxLoc);

                int dist = DescriptorDistance(dMP,dKF);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxGlb = pKF->GetGlobalIdxByLocal(idxLoc,ic);
                }
            }
            // If there is already a MapPoint replace otherwise add new measurement
            if(bestDist<=TH_LOW)
            {
                MapPointPtr  pMPinKF = pKF->GetMapPoint(bestIdxGlb);
                if(pMPinKF)
                {
                    if(!pMPinKF->isBad())
                        vpReplacePoint[iMP] = pMPinKF;
                }
                else
                {
                    pMP->AddObservation(pKF,bestIdxGlb);
                    pKF->AddMapPoint(pMP,bestIdxGlb);
                }
                nFused++;
            }
        }
    }

    return nFused;
}



// 通过Sim3变换，确定pKF1 c1相机的特征点在 pKF2 c2 相机中的大致区域，同理，确定pKF2 c2 的特征点在pKF1 c1 中的大致区域
// 在该区域内通过描述子进行匹配捕获pKF1和pKF2之前漏匹配的特征点，更新vpMatches12（之前使用SearchByBoW进行特征点匹配时会有漏匹配）
// pKF1 是 闭环query帧(mpCurrentKF), pKF2 是闭环候选帧
int ORBmatcher::SearchBySim3CrossCam(KeyFramePtr pKF1,
                                     const int& cam1,
                                     KeyFramePtr pKF2,
                                     const int& cam2,
                                     vector<MapPointPtr > &vpMatches12,
                                     const float &s12,
                                     const cv::Mat &R12,
                                     const cv::Mat &t12,
                                     const float th)
{
    const float &fx1 = pKF1->mvfx[cam1];
    const float &fy1 = pKF1->mvfy[cam1];
    const float &cx1 = pKF1->mvcx[cam1];
    const float &cy1 = pKF1->mvcy[cam1];

    const float &fx2 = pKF2->mvfx[cam2];
    const float &fy2 = pKF2->mvfy[cam2];
    const float &cx2 = pKF2->mvcx[cam2];
    const float &cy2 = pKF2->mvcy[cam2];

    // Camera 1 from world
    // 从world到KF1 cam1的变换
    cv::Mat R1sw = pKF1->GetRotation(cam1);
    cv::Mat t1sw = pKF1->GetTranslation(cam1);

    //Camera 2 from world
    // 从world到KF2 cam2的变换
    cv::Mat R2sw = pKF2->GetRotation(cam2);
    cv::Mat t2sw = pKF2->GetTranslation(cam2);

    //Transformation between cameras
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();
    cv::Mat t21 = -sR21*t12;


    const vector<MapPointPtr > vpMapPoints1Total = pKF1->GetMapPointMatches();
    vector<MapPointPtr> vpMapPoints1;
    vpMapPoints1.resize(pKF1->mvN[cam1], static_cast<MapPointPtr>(NULL));
    int id1 = 0;
    for(size_t i1 = 0; i1 < vpMapPoints1Total.size(); i1++) {
        if(pKF1->keypointToCam[i1] != cam1) continue;
        vpMapPoints1[id1] = (vpMapPoints1Total[i1]);
        id1++;
    }
    const int N1 = vpMapPoints1.size();


    const vector<MapPointPtr > vpMapPoints2Total = pKF2->GetMapPointMatches();
    vector<MapPointPtr> vpMapPoints2;
    vpMapPoints2.resize(pKF2->mvN[cam2], static_cast<MapPointPtr>(NULL));
    int id2 = 0;
    for(size_t i2 = 0; i2 < vpMapPoints2Total.size(); i2++) {
        if(pKF2->keypointToCam[i2] != cam2) continue;
        vpMapPoints2[id2] = vpMapPoints2Total[i2];
        id2++;
    }
    const int N2 = vpMapPoints2.size();
    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);


    // 步骤2：用vpMatches12更新vbAlreadyMatched1和vbAlreadyMatched2
    for(int i=0; i<N1; i++)  // N1 是 pKF1 cam1 下的kp数量， 对应的id是local 的
    {
        MapPointPtr  pMP = vpMatches12[i];
        if(!pMP || pMP->isBad()) continue;

        vbAlreadyMatched1[i]=true;  // 该特征点已经判断过
        int idx2global = pMP->GetIndexInKeyFrame(pKF2);
        if(idx2global < 0) continue;
        int idx2local = (int)pKF2->keypointToCamLocal[idx2global];
        if(idx2local>=0 && idx2local<N2)
            vbAlreadyMatched2[idx2local]=true;

    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // Transform from KF1 to KF2 and search
    for(int i1=0; i1<N1; i1++)
    {
        MapPointPtr  pMP = vpMapPoints1[i1];

        if(!pMP || pMP->isBad() || vbAlreadyMatched1[i1])
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Ds1 = R1sw * p3Dw + t1sw;
        cv::Mat p3Ds2 = sR21 * p3Ds1 + t21;

        // Depth must be positive
        if(p3Ds2.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Ds2.at<float>(2);
        const float x = p3Ds2.at<float>(0)*invz;
        const float y = p3Ds2.at<float>(1)*invz;

        const float u = fx2 * x + cx2;
        const float v = fy2 * y + cy2;

        // Point must be inside the image
        if(!pKF2->IsInImage(cam2,u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Ds2);

        // Depth must be inside the scale invariance region
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);

        // Search in a radius
        const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices2Local = pKF2->GetFeaturesInArea(cam2,u,v,radius);

        if(vIndices2Local.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx2local = -1;
        for(vector<size_t>::const_iterator vit=vIndices2Local.begin(), vend=vIndices2Local.end(); vit!=vend; vit++)
        {
            const size_t idx2local = *vit;

            const cv::KeyPoint &kp = pKF2->mvvkeysUnTemp[cam2][idx2local];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF2->mvDescriptors[cam2].row(idx2local);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx2local = idx2local;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch1[i1]=bestIdx2local;
        }
    }

    // Transform from KF2 to KF2 and search
    for(int i2=0; i2<N2; i2++)
    {
        MapPointPtr  pMP = vpMapPoints2[i2];

        if(!pMP || pMP->isBad() || vbAlreadyMatched2[i2])  continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Ds2 = R2sw * p3Dw + t2sw;
        cv::Mat p3Ds1 = sR12 * p3Ds2 + t12;

        // Depth must be positive
        if(p3Ds1.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Ds1.at<float>(2);
        const float x = p3Ds1.at<float>(0)*invz;
        const float y = p3Ds1.at<float>(1)*invz;

        const float u = fx1 * x + cx1;
        const float v = fy1 * y + cy1;

        // Point must be inside the image
        if(!pKF1->IsInImage(cam1,u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Ds1);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices1Local = pKF1->GetFeaturesInArea(cam1, u,v,radius);

        if(vIndices1Local.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx1local = -1;
        for(vector<size_t>::const_iterator vit=vIndices1Local.begin(), vend=vIndices1Local.end(); vit!=vend; vit++)
        {
            const size_t idx1local = *vit;

            const cv::KeyPoint &kp = pKF1->mvvkeysUnTemp[cam1][idx1local];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF1->mvDescriptors[cam1].row(idx1local);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx1local = idx1local;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch2[i2]=bestIdx1local;
        }
    }

    // Check agreement
    int nFound = 0;

    for(int i1 = 0; i1 < N1; i1++)
    {
        int idx2local = vnMatch1[i1];

        if(idx2local >= 0)
        {
            int idx1local = vnMatch2[idx2local];
            if(idx1local == i1)
            {
                vpMatches12[idx1local] = vpMapPoints2[idx2local];
                nFound++;
            }
        }
    }

    return nFound;
}



void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace ORB_SLAM
