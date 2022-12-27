#pragma once

#include "Utils.hpp"
#include "Feature.hpp"
#include <Eigen/Core>


unsigned long MapPoint::ptID=0;
class MapPoint {
    public:
        static unsigned long ptID;
        int obsCount;
        std::shared_ptr<MapPoint> ptr;
        std::mutex mapPointMutex;
        Eigen::Vec3 position;
        std::vector<std::shared_ptr<Feature>> observations;
        MapPoint(){}
        MapPoint(Eigen::Vec3 pos);
        void setPos(const Eigen::Vec3 &pos);
        Eigen::Vec3 getPos();
        
}