#ifndef MAPPOINT_H
#define MAPPOINT_H


#include "Utils.hpp"
#include <Eigen/Core>


class Feature;
class MapPoint {
    public:
        typedef std::shared_ptr<MapPoint> ptr;
        unsigned long ptID;
        int obsCount;
        std::mutex mapPointMutex;
        Vec3 position;
        std::vector<std::shared_ptr<Feature>> observations;
        MapPoint(){}
        MapPoint(unsigned long _ptID, Vec3 _pos) : ptID(_ptID), position(_pos) {

        }
        void addObservation(std::shared_ptr<Feature> feature);
        static MapPoint::ptr createMapPoint();
        void setPos(const Vec3 pos);
        void setPos(const cv::Point3d pt);
        Vec3 getPos();

};

#endif // TODOITEM_H
