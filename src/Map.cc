/*
 *--------------------------------------------------------------------------------------------------
 * DXSLAM: A Robust and Efficient Visual SLAM System with Deep Features
 *　Author(s):
 * Dongjiang Li, Xuesong Shi, Qiwei Long, Shenghui Liu, Wei Yang, Fangshi Wang, Qi Wei, Fei Qiao qiaofei@mail.tsinghua.edu.cn
 * --------------------------------------------------------------------------------------------------
 * DXSLAM shows that deep CNN-based features can be well incorporated into modern SLAM systems, 
 * and significantly improve the system’s performance. DXSLAM is based on the famous ORB-SLAM2. 
 * If you haven't learn ORB_SLAM2 code, you'd better to be familiar with ORB_SLAM2 project first.
 *　@article{murORB2,
 *　title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
 *　author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
 * journal={IEEE Transactions on Robotics},
 *　volume={33},
 * number={5},
 * pages={1255--1262},
 * doi = {10.1109/TRO.2017.2705103},
 * year={2017}
 *　}
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2020, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/). All rights reserved.
 * Licensed under the GPLv3 License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://github.com/ivipsourcecode/dxslam/blob/master/License-gpl.txt
 *--------------------------------------------------------------------------------------------------
 */
#include "Map.h"

#include<mutex>

namespace DXSLAM
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

std::vector<MapPoint*> Map::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(std::set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(std::set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
