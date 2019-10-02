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

#ifndef MAP_H
#define MAP_H

#include <set>
#include <vector>
#include <mutex>
#include <memory>

using namespace std;
namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

typedef shared_ptr<KeyFrame> KeyFramePtr;
typedef shared_ptr<MapPoint> MapPointPtr;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFramePtr pKF);
    void AddMapPoint(MapPointPtr pMP);
    bool IsPointInMap(MapPointPtr pMP);
    bool IsKeyFrameInMap(KeyFramePtr pKF);
    void EraseMapPoint(MapPointPtr pMP);
    void EraseKeyFrame(KeyFramePtr pKF);
    void SetReferenceMapPoints(const std::vector<MapPointPtr> &vpMPs);
    void SetReferenceKeyFrames(const std::vector<KeyFramePtr> &vpKFs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFramePtr> GetAllKeyFrames();
    std::vector<MapPointPtr> GetAllMapPoints();
    std::vector<MapPointPtr> GetReferenceMapPoints();
    std::vector<KeyFramePtr> GetReferenceKeyFrames();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFramePtr> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    size_t mnStartSaveKFid = 0;

protected:
    std::set<MapPointPtr> mspMapPoints;
    std::set<KeyFramePtr> mspKeyFrames;

    std::vector<MapPointPtr> mvpReferenceMapPoints;
    std::vector<KeyFramePtr> mvpReferenceKeyFrames;

    long unsigned int mnMaxKFid;


    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;
    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
