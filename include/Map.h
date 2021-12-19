/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>

namespace ORB_SLAM3
{

    class MapPoint;
    class KeyFrame;
    class Atlas;
    class KeyFrameDatabase;

    class Map
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &mnId;
            ar &mnInitKFid;
            ar &mnMaxKFid;
            ar &mnBigChangeIdx;
            // Set of KeyFrames and MapPoints, in this version the set serializator is not working
            // ar & mspKeyFrames;
            // ar & mspMapPoints;

            ar &mvpBackupKeyFrames;
            ar &mvpBackupMapPoints;

            ar &mvBackupKeyFrameOriginsId;

            ar &mnBackupKFinitialID;
            ar &mnBackupKFlowerID;

            ar &mbImuInitialized;
            ar &mbIsInertial;
            ar &mbIMU_BA1;
            ar &mbIMU_BA2;

            ar &mnInitKFid;
            ar &mnMaxKFid;
            ar &mnLastLoopKFid;
        }

    public:
        Map();
        Map(int initKFid);
        ~Map();

        /**
         * @brief 用来在地图中插入关键帧,同时更新关键帧的最大id
         * 
         * @param[in] pKF 
         */
        void AddKeyFrame(KeyFrame *pKF);
        void AddMapPoint(MapPoint *pMP);
        void EraseMapPoint(MapPoint *pMP);
        void EraseKeyFrame(KeyFrame *pKF);

        /**
         * @brief 设置参考地图点用于绘图显示局部地图点（红色）
         * 
         * @param[in] vpMPs 
         */
        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);
        void InformNewBigChange();
        int GetLastBigChangeIdx();

        std::vector<KeyFrame *> GetAllKeyFrames();
        std::vector<MapPoint *> GetAllMapPoints();

        /**
         * @brief 将会获取获取参考地图点
         * 
         * @return std::vector<MapPoint *> 
         */
        std::vector<MapPoint *> GetReferenceMapPoints();

        /**
         * @brief 获取地图点数目
         * 
         * @return long unsigned int 
         */
        long unsigned int MapPointsInMap();

        /**
         * @brief 获取地图中的关键帧数目
         * 
         * @return long unsigned 
         */
        long unsigned KeyFramesInMap();

        long unsigned int GetId();

        long unsigned int GetInitKFid();
        void SetInitKFid(long unsigned int initKFif);
        long unsigned int GetMaxKFid();

        KeyFrame *GetOriginKF();

        void SetCurrentMap();
        void SetStoredMap();

        bool HasThumbnail();
        bool IsInUse();

        void SetBad();
        bool IsBad();

        void clear();

        int GetMapChangeIndex();
        void IncreaseChangeIndex();
        int GetLastMapChange();
        void SetLastMapChange(int currentChangeId);

        void SetImuInitialized();
        bool isImuInitialized();

        /**
         * @brief 是对关键帧、MapPoint施加旋转
         * 
         * @param[in] R 
         */
        void RotateMap(const cv::Mat &R);

        /**
         * @brief 对施加的旋转进行应用
         * 
         * @param[in] R 
         * @param[in] s 
         * @param[in] bScaledVel 
         * @param[in] t 
         */
        void ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel = false, const cv::Mat t = cv::Mat::zeros(cv::Size(1, 3), CV_32F));

        void SetInertialSensor();
        bool IsInertial();
        void SetIniertialBA1();
        void SetIniertialBA2();
        bool GetIniertialBA1();
        bool GetIniertialBA2();

        void PrintEssentialGraph();

        /**
         * @brief 通过两个for循环来检测第一个关键帧是否具有父代和子代
         * 
         * @return true 
         * @return false 
         */
        bool CheckEssentialGraph();
        void ChangeId(long unsigned int nId);

        unsigned int GetLowerKFID();

        /**
         * @brief 首先通过一个for循环来保存原始关键帧的id，接下来是将集合的容器备份到向量，来达到一个预存的目的。
         * 
         * @param[in] spCams 
         */
        void PreSave(std::set<GeometricCamera *> &spCams);
        void PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc, map<long unsigned int, KeyFrame *> &mpKeyFrameId, map<unsigned int, GeometricCamera *> &mpCams);

        void printReprojectionError(list<KeyFrame *> &lpLocalWindowKFs, KeyFrame *mpCurrentKF, string &name, string &name_folder);

        vector<KeyFrame *> mvpKeyFrameOrigins;
        vector<unsigned long int> mvBackupKeyFrameOriginsId;
        KeyFrame *mpFirstRegionKF;
        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

        bool mbFail;

        // Size of the thumbnail (always in power of 2)
        static const int THUMB_WIDTH = 512;
        static const int THUMB_HEIGHT = 512;

        static long unsigned int nNextId;

    protected:
        long unsigned int mnId;
        // 记录地图中的地图点
        std::set<MapPoint *> mspMapPoints;
        // 记录地图中的关键帧指针
        std::set<KeyFrame *> mspKeyFrames;

        std::vector<MapPoint *> mvpBackupMapPoints;
        std::vector<KeyFrame *> mvpBackupKeyFrames;

        KeyFrame *mpKFinitial;
        KeyFrame *mpKFlowerID;

        unsigned long int mnBackupKFinitialID;
        unsigned long int mnBackupKFlowerID;

        std::vector<MapPoint *> mvpReferenceMapPoints;

        bool mbImuInitialized;
        // ?
        int mnMapChange;
        // ？
        int mnMapChangeNotified;

        long unsigned int mnInitKFid;
        // 记录最大关键帧id
        long unsigned int mnMaxKFid;
        long unsigned int mnLastLoopKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;

        // View of the map in aerial sight (for the AtlasViewer)
        GLubyte *mThumbnail;

        bool mIsInUse;
        bool mHasTumbnail;
        bool mbBad = false;

        bool mbIsInertial; //标记是否有imu
        bool mbIMU_BA1;
        bool mbIMU_BA2;

        std::mutex mMutexMap;
    };

} // namespace ORB_SLAM3

#endif // MAP_H
