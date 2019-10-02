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
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
vector<float> Frame::mvMinX, Frame::mvMinY, Frame::mvMaxX, Frame::mvMaxY;
vector<float> Frame::mvfGridElementHeightInv, Frame::mvfGridElementWidthInv;
vector<float> Frame::mvfx, Frame::mvfy, Frame::mvcx, Frame::mvcy, Frame::mvinvfx, Frame::mvinvfy;

Frame::Frame()
{}

/** //Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}
*/

Frame::Frame(SystemPtr pSystem,
             vector<cv::Mat> &ims,
             const double &timeStamp,
             CamerasPtr pCameras,
             vector<ORBextractorPtr> extractors,
             ORBVocabularyPtr voc,
             bool isMapScaled
             )
    : mpSystem(pSystem),
      mvImages(ims),
      mpORBvocabulary(voc),
      mvpORBextractor(extractors),
      mTimeStamp(timeStamp),
      mpCameras(pCameras),
      mbMapScaled(isMapScaled)
{
    // Frame ID
    mnId=nNextId++;
    int nCameras = mpCameras->getNCameras();
    mnCams = nCameras;
    mvK = mpCameras->getK();
    mvDistCoef = mpCameras->getDistCoeffs();
    mvExtrinsics.resize(nCameras);
    mvExtAdj.resize(nCameras);
    mvDescriptors.resize(nCameras);
    mvBowVec.resize(nCameras);
    mvFeatVec.resize(nCameras);
    mvvkeysTemp.resize(nCameras);
    mvvkeysUnTemp.resize(nCameras);
    mvN.resize(nCameras);
    mvRelocTsw.resize(nCameras);
    mvGrids = std::vector<std::vector<std::vector<std::vector<size_t> > > >(nCameras);
    totalN = 0;

    mpORBextractor = mvpORBextractor[CAP];
    // Scale Level Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractor->GetScaleFactors();
    mvInvScaleFactors = mpORBextractor->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    cv::Mat imGray = mvImages[CAP];
    if(mbInitialComputations) {
        mvMinX.resize(nCameras);
        mvMaxX.resize(nCameras);
        mvMinY.resize(nCameras);
        mvMaxY.resize(nCameras);
        mvfGridElementWidthInv.resize(nCameras);
        mvfGridElementHeightInv.resize(nCameras);
        mvfx.resize(nCameras);
        mvfy.resize(nCameras);
        mvcx.resize(nCameras);
        mvcy.resize(nCameras);
        mvinvfx.resize(nCameras);
        mvinvfy.resize(nCameras);

    }

    for(int c= 0; c < nCameras; c++) {
        mvExtrinsics[c] = mpCameras->getExtrinsici(c).clone();
        mvExtAdj[c] = mpCameras->getExtrinsicAdji(c).clone();
        ExtractORB(c, mvvkeysTemp[c], mvDescriptors[c]);
        // cout << "for frame at image" << c << " keypoint number:" << mvvkeysTemp[c].size() << endl;
        mvN[c] = mvvkeysTemp[c].size();
        if( ! mvvkeysTemp[c].empty()) {
            UndistortKeyPoints(c);
        }

        if(mbInitialComputations)
        {

            ComputeImageBounds(c);

            mvfGridElementWidthInv[c]=static_cast<float>(FRAME_GRID_COLS)
                    /static_cast<float>(mvMaxX[c]-mvMinX[c]);
            mvfGridElementHeightInv[c]=static_cast<float>(FRAME_GRID_ROWS)
                    /static_cast<float>(mvMaxY[c]-mvMinY[c]);

            auto K = mpCameras->getKi(c);

            mvfx[c] = K.at<float>(0,0);
            mvfy[c] = K.at<float>(1,1);
            mvcx[c] = K.at<float>(0,2);
            mvcy[c] = K.at<float>(1,2);
            mvinvfx[c] = 1.0f/mvfx[c];
            mvinvfy[c] = 1.0f/mvfy[c];

        }

        mvGrids[c] = std::vector<std::vector<std::vector<size_t> > >(FRAME_GRID_COLS);
        for (unsigned int j = 0; j < FRAME_GRID_COLS; ++j)
            mvGrids[c][j] = std::vector<std::vector<size_t> >(FRAME_GRID_ROWS);
    }

    mbInitialComputations=false;

    int currPtIdx = 0;
    for(int c = 0; c < nCameras; c++) {
        totalN += mvN[c];
        for(size_t i = 0; i < mvvkeysUnTemp[c].size(); i++) {
            mvTotalKeys.push_back(mvvkeysTemp[c][i]);
            mvTotalKeysUn.push_back(mvvkeysUnTemp[c][i]);
            keypointToCam[currPtIdx] = c;
            keypointToCamLocal[currPtIdx] = i;
            cv::KeyPoint& kp = mvvkeysUnTemp[c][i];

            //assign features to the grid
            int nGridPosX, nGridPosY;
            if(PosInGrid(c, kp,nGridPosX,nGridPosY)){
                mvGrids[c][nGridPosX][nGridPosY].push_back(i);
            }
            currPtIdx++;
        }
    }

    mvpMapPoints = vector<MapPointPtr >(totalN,static_cast<MapPointPtr >(NULL));
    mvbOutlier = vector<bool>(totalN,false);

}


/**
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);
}*/

void Frame::ExtractORB(const int& c, vector<cv::KeyPoint>& vKeys, cv::Mat& Descriptor )
{
    (*(mvpORBextractor[c]))(mvImages[c], cv::Mat(), vKeys, Descriptor );
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}


cv::Mat Frame::GetCameraCenter(int cam) {
    if(cam == 0){
        return mOw.clone();
    } else {
        cv::Mat ext = mvExtrinsics[cam];
        cv::Mat Tsw = ext * mTcw;
        cv::Mat Rsw = Tsw.rowRange(0,3).colRange(0,3);
        cv::Mat tsw = Tsw.rowRange(0,3).col(3);
        cv::Mat Rws = Rsw.t();
        cv::Mat Osw = -Rws * tsw;
        return Osw.clone();
    }
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPointPtr pMP, float viewingCosLimit, bool bForAllCam)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();
    for(int ic = 0; ic < mnCams; ic++) {
        if (ic != 0 && ! bForAllCam) continue;
        const cv::Mat Tsc = mvExtrinsics[ic];
        const cv::Mat Tcw = mTcw;
        const cv::Mat Tsw = Tsc * Tcw;

        const cv::Mat Rsw = Tsw.rowRange(0,3).colRange(0,3);
        const cv::Mat tsw = Tsw.rowRange(0,3).col(3);

        const cv::Mat Pic = Rsw*P+tsw;

        const float &PicX = Pic.at<float>(0);
        const float &PicY= Pic.at<float>(1);
        const float &PicZ = Pic.at<float>(2);

        // Check positive depth
        if(PicZ < 0.0f)
            continue;


        // Project in image and check it is not outside
        const float invz = 1.0f/PicZ;
        const float u=mvfx[ic]*PicX*invz+mvcx[ic];
        const float v=mvfy[ic]*PicY*invz+mvcy[ic];

        if(u<mvMinX[ic] || u>mvMaxX[ic])
            continue;


        if(v<mvMinY[ic] || v>mvMaxY[ic])
            continue;


        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P - GetCameraCenter(ic);
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();
        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<viewingCosLimit)
            continue;

        const int nPredictedLevel = pMP->PredictScale(dist,shared_from_this());

        pMP->mbTrackInView = true;
        pMP->mTrackProjCamera = ic;
        pMP->mTrackProjX = u;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    return false;
}



vector<size_t> Frame::GetFeaturesInArea(const int& c, const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(totalN);

    const int nMinCellX = max(0,(int)floor((x-mvMinX[c]-r)*mvfGridElementWidthInv[c]));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mvMinX[c]+r)*mvfGridElementWidthInv[c]));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mvMinY[c]-r)*mvfGridElementHeightInv[c]));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mvMinY[c]+r)*mvfGridElementHeightInv[c]));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mvGrids[c][ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                size_t localKpId = vCell[j];
                size_t realKpId = 0;
                for(int ic = 0; ic < c; ic++) realKpId += mvN[ic];
                realKpId += localKpId;
                const cv::KeyPoint &kpUn = mvvkeysUnTemp[c][localKpId];
                
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0 && kpUn.octave>maxLevel)
                        continue;
                }


                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r) {
                    vIndices.push_back(localKpId);
                }

            }
        }
    }

    return vIndices;   // 这个Indices是local 下的
}



bool Frame::PosInGrid(const int& c, const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = cvRound((kp.pt.x - mvMinX[c])*mvfGridElementWidthInv[c]);
    posY = cvRound((kp.pt.y - mvMinY[c])*mvfGridElementHeightInv[c]);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    // cout << S::yellow << __FUNCTION__ << S::endc << endl;
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

void Frame::UndistortKeyPoints(const int& c)
{
    auto coeffi = mpCameras->getDistCoeffsi(c);
    auto k = mpCameras->getKi(c);
    if(coeffi.at<float>(0) == 0.0)
    {
        mvvkeysUnTemp[c] = mvvkeysTemp[c];
        return;
    }

    // Fill matrix with points
    cv::Mat mat(mvN[c],2,CV_32F);
    for(int i=0; i<mvN[c]; i++)
    {
        mat.at<float>(i,0)=mvvkeysTemp[c][i].pt.x;
        mat.at<float>(i,1)=mvvkeysTemp[c][i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,k,coeffi,cv::Mat(),k);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvvkeysUnTemp[c].resize(mvN[c]);
    for(int i=0; i<mvN[c]; i++)
    {
        cv::KeyPoint kp = mvvkeysTemp[c][i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvvkeysUnTemp[c][i]=kp;
    }
}

size_t Frame::GetGlobalIdxByLocal(const size_t& localIdx, const int& cam)
{
    size_t globalKpId = 0;
    for(int ic = 0; ic < cam; ic++) globalKpId += mvN[ic];
    globalKpId += localIdx;
    return globalKpId;
}



void Frame::ComputeImageBounds(const int& c)
{
    auto coeffi = mpCameras->getDistCoeffsi(c);
    auto k = mpCameras->getKi(c);
    if(coeffi.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=mvImages[c].cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=mvImages[c].rows;
        mat.at<float>(3,0)=mvImages[c].cols; mat.at<float>(3,1)=mvImages[c].rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,k,coeffi,cv::Mat(),k);
        mat=mat.reshape(1);

        mvMinX[c] = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mvMaxX[c] = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mvMinY[c] = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mvMaxY[c] = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mvMinX[c] = 0.0f;
        mvMaxX[c]= mvImages[c].cols;
        mvMinY[c] = 0.0f;
        mvMaxY[c] = mvImages[c].rows;
    }
}


void Frame::SaveImageWithMatches(string preName)
{

    cv::RNG rng(time(0));
    cv::Mat showimg = Converter::jointImage(mvImages);
    if(showimg.channels()<3) //this should be always true
        cvtColor(showimg,showimg,CV_GRAY2BGR);
    auto vpMP = mvpMapPoints;
    unordered_map<int,int> camKps;
    for(int i = 0; i < totalN; i++ ) {
        auto pMP = vpMP[i];
        if( !pMP || pMP->isBad())continue;
        int camid = keypointToCam[i];
        camKps[camid]++;
        cv::Point2f pt1,pt2;

        auto color = cv::Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
        auto newx = mvTotalKeysUn[i].pt.x + camid  * 640;
        auto newy = mvTotalKeysUn[i].pt.y;
        cv::circle(showimg,cv::Point(newx, newy),5,color,-1);
    }

    string name = preName;
    for(int i = 0; i < mnCams; i++) {
        name += "cam" + to_string(i) + "_" + to_string(camKps[i]);
    }
    cv::imwrite("F" + to_string(mnId) + name + ".jpg", showimg);
    return;
}




} //namespace ORB_SLAM
