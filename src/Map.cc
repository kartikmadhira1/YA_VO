#include "../include/Map.hpp"






// Add frame to the frame_id -> frame mapping with FramesType variable
void Map::insertKeyFrame(Frame::ptr currFrame) {
    this->currentFrame = currFrame;
    if (frames.find(currFrame->frameID) == frames.end()) {
        frames.insert(std::make_pair(currentFrame->frameID, currentFrame));
    } else {
        frames.insert(std::make_pair(currentFrame->frameID, currentFrame));
    }
}

void Map::insertMapPoint(MapPoint::ptr mp) {
    if (landmarks.find(mp->ptID) == landmarks.end()) {
        landmarks.insert(std::make_pair(mp->ptID, mp));
    } else {
        landmarks.insert(std::make_pair(mp->ptID, mp));
    }
}


