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
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(MapPtr  pMap, const string &strSettingPath) :
    mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::SetCameras(CamerasPtr pCams) {
    mpCameras = pCams;
    mnCams = pCams->mnCameras;
    mvCamerasPoses.resize(mnCams);
}

void MapDrawer::SetScale(const double& scale) {
    mKeyFrameSize *= 0.7 * scale;
    // mKeyFrameLineWidth *= scale;
    // mGraphLineWidth *= scale;
    // mPointSize *= scale;
    mCameraSize *= 0.7 * scale;
    // mCameraLineWidth *= scale;
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPointPtr > &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPointPtr > &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPointPtr > spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);


    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if( ! vpMPs[i] || vpMPs[i]->isBad())
            continue;
        if(vpMPs[i]->mbViewdByDifCams) continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);


    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        //|| spRefMPs.count(vpMPs[i])
        if(! vpMPs[i] || vpMPs[i]->isBad())
            continue;
        if( ! vpMPs[i]->mbViewdByDifCams) continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();

        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();

    //    glPointSize(0.5 * mPointSize);
    //    glBegin(GL_POINTS);
    //    glColor3f(1.0,0.0,0.0);

    //    for(set<MapPointPtr >::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    //    {
    //        if((*sit)->isBad())
    //            continue;
    //        cv::Mat pos = (*sit)->GetWorldPos();
    //        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    //    }

    //    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFramePtr > vpKFs = mpMap->GetAllKeyFrames();
    const vector<KeyFramePtr> vpRefKFs = mpMap->GetReferenceKeyFrames();

    set<KeyFramePtr > spRefKFs(vpRefKFs.begin(), vpRefKFs.end());

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFramePtr  pKF = vpKFs[i];

            if(pKF->isBad())
                continue;

            for(int j = 0; j < mnCams; j++) {
                if(j != 0 && ! pKF->mbMapScaled) continue;
                cv::Mat Tcw = pKF->GetPose();
                cv::Mat Tsc = pKF->mpCameras->getExtrinsici(j);
                cv::Mat Tsw = Tsc * Tcw;

                cv::Mat Rsw = Tsw.rowRange(0,3).colRange(0,3);
                cv::Mat tsw = Tsw.rowRange(0,3).col(3);
                cv::Mat Rws = Rsw.t();
                cv::Mat Ow = -Rws*tsw;
                cv::Mat Tws = cv::Mat::eye(4,4,Tsw.type());
                Rws.copyTo(Tws.rowRange(0,3).colRange(0,3));
                Ow.copyTo(Tws.rowRange(0,3).col(3));

                Tws = Tws.t();

                glPushMatrix();

                glMultMatrixf(Tws.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                if(spRefKFs.count(vpKFs[i]))
                    glColor3f(0.0f, 0.0f, 1.0f); // 紫
                else
                    glColor3f(0.227f,0.56f,0.718f); // 浅蓝
                glBegin(GL_LINES);
                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }


        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.28f,0.0f,0.2f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFramePtr > vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFramePtr >::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFramePtr  pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFramePtr > sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFramePtr >::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}


void MapDrawer::DrawCurrentCamera()
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    for( int i = 0; i< mnCams; i++) {
        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(mvCamerasPoses[i].m);
#else
        glMultMatrixd(mvCamerasPoses[i].m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,0.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }


}

void MapDrawer::DrawRelocCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(2 * mCameraLineWidth);
    glColor3f(0.917f,0.0f,0.196f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        for(int i = 0; i < mnCams; i++) {

            pangolin::OpenGlMatrix Ms;

            cv::Mat Tcw = mCameraPose;
            cv::Mat Tsc = mpCameras->getExtrinsici(i);
            cv::Mat Tsw = Tsc * Tcw;

            cv::Mat Rws(3,3,CV_32F);
            cv::Mat tws(3,1,CV_32F);

            {
                unique_lock<mutex> lock(mMutexCamera);
                Rws = Tsw.rowRange(0,3).colRange(0,3).t();
                tws = -Rws*Tsw.rowRange(0,3).col(3);
            }

            Ms.m[0] = Rws.at<float>(0,0);
            Ms.m[1] = Rws.at<float>(1,0);
            Ms.m[2] = Rws.at<float>(2,0);
            Ms.m[3]  = 0.0;

            Ms.m[4] = Rws.at<float>(0,1);
            Ms.m[5] = Rws.at<float>(1,1);
            Ms.m[6] = Rws.at<float>(2,1);
            Ms.m[7]  = 0.0;

            Ms.m[8] = Rws.at<float>(0,2);
            Ms.m[9] = Rws.at<float>(1,2);
            Ms.m[10] = Rws.at<float>(2,2);
            Ms.m[11]  = 0.0;

            Ms.m[12] = tws.at<float>(0);
            Ms.m[13] = tws.at<float>(1);
            Ms.m[14] = tws.at<float>(2);
            Ms.m[15]  = 1.0;

            if(i == 0) M = Ms;
            mvCamerasPoses[i] = Ms;
        }

    }
    else
        M.SetIdentity();
}


void MapDrawer::SetRelocCameraPose(const cv::Mat &Tcw)
{
    mRelocCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentRelocOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mRelocCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mRelocCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mRelocCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
