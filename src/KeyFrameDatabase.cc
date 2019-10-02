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
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (const ORBVocabularyPtr voc):
    mpVoc(voc)
{

}

void KeyFrameDatabase::setNCams(int n)
{
    mnCams = n;
    mvvInvertedFiles.resize(mnCams);
    for(int i= 0; i < mnCams; i++) {
        mvvInvertedFiles[i].resize(mpVoc->size());
    }
}

void KeyFrameDatabase::add(KeyFramePtr pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(int c = 0; c < pKF->mnCams; c++) {
        for(DBoW2::BowVector::const_iterator vit= pKF->mvBowVec[c].begin(), vend=pKF->mvBowVec[c].end(); vit!=vend; vit++)
            mvvInvertedFiles[c][vit->first].push_back(pKF);
    }
}

void KeyFrameDatabase::erase(KeyFramePtr  pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry

    for(int c = 0 ; c < pKF->mnCams; c++) {
        for(DBoW2::BowVector::const_iterator vit=pKF->mvBowVec[c].begin(), vend=pKF->mvBowVec[c].end(); vit!=vend; vit++)
        {
            // List of keyframes that share the word
            list<KeyFramePtr > &lKFs =  mvvInvertedFiles[c][vit->first];

            for(list<KeyFramePtr >::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                if(pKF==*lit)
                {
                    lKFs.erase(lit);
                    break;
                }
            }
        }
    }
}

void KeyFrameDatabase::clear()
{

    mvvInvertedFiles.clear();
    mvvInvertedFiles.resize(mnCams);
    for(int i= 0; i < mnCams; i++) {
        mvvInvertedFiles[i].resize(mpVoc->size());
    }

}


vector<KeyFramePtr > KeyFrameDatabase::DetectLoopCandidatesForCam(KeyFramePtr pKF,
                                                                  const int& queryC,
                                                                  const int& respC,
                                                                  float minScore)
{
    // 提出所有与该pKF相连的KeyFrame，这些相连Keyframe都是局部相连，在闭环检测的时候将被剔除
    set<KeyFramePtr > spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFramePtr > lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    // 步骤1：找出和当前帧具有公共单词的所有关键帧（不包括与当前帧链接的关键帧）
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历该pKF的每一个word
        for(DBoW2::BowVector::const_iterator vit=pKF->mvBowVec[queryC].begin(), vend=pKF->mvBowVec[queryC].end(); vit != vend; vit++)
        {
            // 提取所有包含该word的KeyFrame
            // 所有包含 vit->first 这个wordID 的关键帧.
            list<KeyFramePtr > &lKFs = mvvInvertedFiles[respC][vit->first];

            for(list<KeyFramePtr >::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFramePtr  pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)  // pKFi还没有标记为pKF的候选帧
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))  // 与pKF局部链接的关键帧不进入闭环候选帧
                    {
                        pKFi->mnLoopQuery=pKF->mnId;  // pKFi标记为pKF的候选帧，之后直接跳过判断
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;  // 记录pKFi与pKF具有相同word的个数
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFramePtr>();

    list<pair<float,KeyFramePtr > > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
     // 步骤2：统计所有闭环候选帧中与pKF具有共同单词最多的单词数
    int maxCommonWords=0;
    for(list<KeyFramePtr >::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    // 步骤3：遍历所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    for(list<KeyFramePtr >::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFramePtr  pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mvBowVec[queryC],pKFi->mvBowVec[respC]);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFramePtr >();

    list<pair<float,KeyFramePtr > > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 步骤4：计算候选帧组得分，得到最高组得分bestAccScore，并以此决定阈值minScoreToRetain
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
    // 具体而言：lScoreAndMatch中每一个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,KeyFramePtr > >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFramePtr  pKFi = it->second;
        vector<KeyFramePtr > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first; // 该组最高分数
        float accScore = it->first; // 该组累计得分
        KeyFramePtr  pBestKF = pKFi;  // 该组最高分数对应的关键帧
        for(vector<KeyFramePtr >::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFramePtr  pKF2 = *vit;
            // 因为pKF2->mnLoopQuery==pKF->mnId，所以只有pKF2也在闭环候选帧中，才能贡献分数
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)// 统计得到组里分数最高的KeyFrame
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)  // 记录所有组中组得分最高的组
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFramePtr > spAlreadyAddedKF;
    vector<KeyFramePtr > vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    // 步骤5：得到组得分大于minScoreToRetain的组，得到组中分数最高的关键帧 0.75*bestScore
    for(auto it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFramePtr  pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

// Relocalization target camera tgtC at source camera srcC
std::vector<KeyFramePtr> KeyFrameDatabase::DetectRelocalizationCandidatesForCam(FramePtr F, const int& queryC, const int& respC )
{
    list<KeyFramePtr > lKFsSharingWords;
    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);
        // cout << "  F->mBowVec.size() after ComputeBoWOnCam" << F->mBowVec.size() << endl;
        for(DBoW2::BowVector::const_iterator vit = F->mvBowVec[queryC].begin(), vend = F->mvBowVec[queryC].end(); vit != vend; vit++)
        {
            list<KeyFramePtr > &lKFs = mvvInvertedFiles[respC][vit->first];

            for(list<KeyFramePtr >::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFramePtr  pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }

//        cout << "lKFsSharingWords size" << lKFsSharingWords.size() << endl;
//        for(auto eachKF:lKFsSharingWords) {
//            cout << "KFi mnRelocWords: " << eachKF->mnRelocWords  << endl;
//        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFramePtr >();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFramePtr >::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFramePtr > > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(list<KeyFramePtr >::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFramePtr  pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mvBowVec[queryC],pKFi->mvBowVec[respC]);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFramePtr >();

    list<pair<float,KeyFramePtr > > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFramePtr > >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFramePtr  pKFi = it->second;
        vector<KeyFramePtr > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFramePtr  pBestKF = pKFi;
        for(vector<KeyFramePtr >::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFramePtr  pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // cout << "lAccScoreAndMatch size" << lAccScoreAndMatch.size() << endl;
    // cout << "bestAccuScore: " << bestAccScore << endl;

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    // cout << "minScoreToRetain: " << minScoreToRetain << endl;
    set<KeyFramePtr > spAlreadyAddedKF;
    vector<KeyFramePtr > vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFramePtr > >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            KeyFramePtr  pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    // cout << "vpRelocCandidates.size(): " << vpRelocCandidates.size() << endl;
    return vpRelocCandidates;
}

} //namespace ORB_SLAM
