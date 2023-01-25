#include "../include/Map.hpp"






// Add frame to the frame_id -> frame mapping with FramesType variable
void Map::insertKeyFrame(Frame::ptr currFrame) {
    std::unique_lock<std::mutex> lck(mapLock);

    this->currentFrame = currFrame;
    if (frames.find(currFrame->frameID) == frames.end()) {
        frames.insert(std::make_pair(currentFrame->frameID, currentFrame));
    } else {
        frames.insert(std::make_pair(currentFrame->frameID, currentFrame));
    }
    if (activeF.find(currFrame->frameID) == activeF.end()) {
        activeF.insert(std::make_pair(currentFrame->frameID, currentFrame));
    } else {
        activeF.insert(std::make_pair(currentFrame->frameID, currentFrame));
    }
}

void Map::insertMapPoint(MapPoint::ptr mp) {
    std::unique_lock<std::mutex> lck(mapLock);

    if (landmarks.find(mp->ptID) == landmarks.end()) {
        landmarks.insert(std::make_pair(mp->ptID, mp));
    } else {
        landmarks.insert(std::make_pair(mp->ptID, mp));
    }
    if (activeL.find(mp->ptID) == activeL.end()) {
        activeL.insert(std::make_pair(mp->ptID, mp));
    } else {
        activeL.insert(std::make_pair(mp->ptID, mp));
    }
}


bool Map::resetActive() {
    // empty both active frames and landmarks
    std::unique_lock<std::mutex> lck(mapLock);

    activeL.clear();
    activeF.clear();
}


Map::ptr Map::createMap() {
    Map::ptr _map = std::make_shared<Map>();
    return _map;
}

Map::FramesType Map::getActiveFrames() {
    std::unique_lock<std::mutex> lck(mapLock);
    return activeF;
}

Map::FramesType Map::getFrames() {
    std::unique_lock<std::mutex> lck(mapLock);
    return frames;
}

Map::LandMarksType Map::getActiveMPs() {
    std::unique_lock<std::mutex> lck(mapLock);
    return activeL;

}

Map::LandMarksType Map::getMPs() {
    std::unique_lock<std::mutex> lck(mapLock);
    return landmarks;
}