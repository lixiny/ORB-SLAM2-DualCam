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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<set>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <memory>

using namespace std;
namespace ORB_SLAM2
{


class MapPoint;
class KeyFrame;
class Frame;

typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<Frame> FramePtr;
typedef shared_ptr<MapPoint> MapPointPtr;


class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a,
                                  const cv::Mat &b);


    /** Search matches between Frame keypoints and projected MapPoints. Returns number of matches
     * Used to track the local map (Tracking)
     * @brief SearchByProjection
     * @param pF
     * @param vpMapPoints
     * @param th
     * @return
     */
    int SearchByProjection(FramePtr pF,
                           const std::vector<MapPointPtr> &vpMapPoints,
                           const float th=3);


    /** Project MapPoints tracked in last frame into the current frame and search matches.
     * Used to track from previous frame (Tracking)
     * @brief SearchByProjection
     * @param pCurrentFrame
     * @param pLastFrame
     * @param th
     * @param bMapScaled
     * @return
     */
    int SearchByProjection(FramePtr pCurrentFrame,
                           const FramePtr pLastFrame,
                           const float th,
                           bool bMapScaled);

    //
    /** Project all the mapPoints tracked in the pKF into the
     *  current Frame 's query camera. and search matches.
     * @brief SearchByProjectionOnCam
     * @param pF
     * @param query
     * @param pKF
     * @param sAlreadyFound
     * @param th
     * @param ORBdist
     * @return
     */
    int SearchByProjectionOnCam(FramePtr pF,
                                const int& query,
                                KeyFramePtr pKF,
                                const set<MapPointPtr > &sAlreadyFound,
                                const float th,
                                const int ORBdist);

    /**
     * @brief SearchByProjection
     * @param pKF
     * @param vpMapPoints
     * @param sAlreadyFound
     * @param th
     * @param ORBdist
     * @return
     */
    int SearchByProjection(KeyFramePtr pKF,
                           const std::vector<MapPointPtr> &vpMapPoints,
                           const set<MapPointPtr > &sAlreadyFound,
                           const float th ,
                           const int ORBdist);


    /**
     * @brief SearchByProjectionOnCam
     * @param pFcur
     * @param query
     * @param pFlast
     * @param th
     * @return
     */
    int SearchByProjectionOnCam(FramePtr pFcur,
                                const int& query,
                                FramePtr pFlast,
                                const float th);


    /** Project MapPoints using a Similarity Transformation and search matches.
     * Used in loop detection (Loop Closing)
     * @brief SearchByProjection
     * @param pKF
     * @param query
     * @param Scq_w
     * @param vpPoints
     * @param vpMatched
     * @param th
     * @return
     */
    int SearchByProjection(KeyFramePtr  pKF,
                           const int & query,
                           cv::Mat Scq_w,
                           const vector<MapPointPtr > &vpPoints,
                           vector<MapPointPtr > &vpMatched,
                           int th);



    /** Search matches between MapPoints in a KeyFrame and ORB in a Frame.
     * Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
     * Used in Relocalisation and Loop Detection
     *
     * @brief SearchByBoW
     * @param F
     * @param pKF
     * @param vpMapPointMatches
     * @param bMapScaled
     * @return
     */
    int SearchByBoW(FramePtr F,
                    KeyFramePtr pKF,
                    std::vector<MapPointPtr> &vpMapPointMatches,
                    bool bMapScaled);

    /**
     * @brief SearchByBoWCrossCam
     * @param pKF1
     * @param c1
     * @param pKF2
     * @param c2
     * @param vpMatches12
     * @return
     */
    int SearchByBoWCrossCam(KeyFramePtr pKF1,
                            const int& c1,
                            KeyFramePtr pKF2,
                            const int& c2,
                            std::vector<MapPointPtr> &vpMatches12);


    /**
     * @brief SearchByBoWCrossCam
     * @param F
     * @param cF
     * @param pKF
     * @param cKF
     * @param vpMapPointMatches
     * @return
     */
    int SearchByBoWCrossCam(FramePtr F,
                            const int& cF,
                            KeyFramePtr pKF,
                            const int& cKF,
                            std::vector<MapPointPtr> &vpMapPointMatches);


    /** Matching for the Map Initialization (only used in the monocular case)
     * @brief SearchForInitialization
     * @param pF1
     * @param pF2
     * @param vbPrevMatched
     * @param vnMatches12
     * @param windowSize
     * @return
     */
    int SearchForInitialization(FramePtr pF1,
                                FramePtr pF2,
                                std::vector<cv::Point2f> &vbPrevMatched,
                                std::vector<int> &vnMatches12,
                                int windowSize=10);



    /**  Matching to triangulate new MapPoints. Check Epipolar Constraint.
     * @brief SearchForTriangulation
     * @param pKF1
     * @param pKF2
     * @param F12
     * @param vMatchedPairs
     * @param cam
     * @return
     */
    int SearchForTriangulation(KeyFramePtr pKF1,
                               KeyFramePtr pKF2,
                               cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs,
                               int cam = 0);


    /** Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
     * @brief SearchBySim3CrossCam
     * @param pKF1
     * @param c1
     * @param pKF2
     * @param c2
     * @param vpMatches12
     * @param s12
     * @param R12
     * @param t12
     * @param th
     * @return
     */
    int SearchBySim3CrossCam(KeyFramePtr pKF1,
                             const int& c1,
                             KeyFramePtr pKF2,
                             const int& c2,
                             std::vector<MapPointPtr> &vpMatches12,
                             const float &s12,
                             const cv::Mat &R12,
                             const cv::Mat &t12,
                             const float th);

    /** Project MapPoints into KeyFrame and search for duplicated MapPoints.
     * @brief Fuse
     * @param pKF
     * @param vpMapPoints
     * @param th
     * @return
     */
    int Fuse(KeyFramePtr pKF,
             const vector<MapPointPtr> &vpMapPoints,
             const float th=3.0);


    /** Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
     * @brief Fuse
     * @param pKF
     * @param Scw
     * @param vpPoints
     * @param th
     * @param vpReplacePoint
     * @return
     */
    int Fuse(KeyFramePtr pKF,
             cv::Mat Scw,
             const std::vector<MapPointPtr> &vpPoints,
             float th,
             vector<MapPointPtr> &vpReplacePoint);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

    const int CAP = 0;


protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1,
                               const cv::KeyPoint &kp2,
                               const cv::Mat &F12,
                               const KeyFramePtr pKF);

    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int>* histo,
                            const int L,
                            int &ind1,
                            int &ind2,
                            int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
