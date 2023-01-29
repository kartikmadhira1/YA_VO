#include "../include/MapPoint.hpp"




MapPoint::ptr MapPoint::createMapPoint() {
    static unsigned long _ptID = 0;
    MapPoint::ptr mp = std::make_shared<MapPoint>();
    mp->ptID = ++_ptID; 
    return mp;
}


void MapPoint::setPos(const cv::Point3d pt) {
    //lock the mutex first 
    std::unique_lock<std::mutex> lock(mapPointMutex);
    this->position << pt.x , pt.y, pt.z;
}

void MapPoint::setPos(const Vec3 pos) {
    //lock the mutex first 
    std::unique_lock<std::mutex> lock(mapPointMutex);
    this->position << pos.coeff(0) ,pos.coeff(1), pos.coeff(2);

}

Vec3 MapPoint::getPos() {
    //lock the mutex first 
    std::unique_lock<std::mutex> lock(mapPointMutex);
    return this->position;
}




void MapPoint::addObservation(std::shared_ptr<Feature> feat) {
    std::unique_lock<std::mutex> lock(mapPointMutex);
    this->observations.push_back(feat);
    obsCount++;
}