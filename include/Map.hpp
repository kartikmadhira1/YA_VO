#ifndef MAP_H
#define MAP_H

#include <unordered_map>
#include "Frame.hpp"
#include "MapPoint.hpp"
#include "Feature.hpp"





class Map {
    public:
        // 3d points stored as a map
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandMarksType;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> FramesType;
        typedef std::shared_ptr<Map> ptr;

        Map() {}
        // initialize map with initial pose 
        void initMap();
        // Insert frames and mappoint to the whole map
        void insertKeyFrame(Frame::ptr fr);
        void insertMapPoint(MapPoint::ptr mp)

    private:
        std::mutex mapLock;
        LandMarksType landmarks;
        FramesType frames;

        Frame::Ptr currentFrame = nullptr;

};

#endif // TODOITEM_H
