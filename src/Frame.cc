#include "../include/Frame.hpp"
#include "../include/Utils.hpp"


void Frame::setPose(Sophus::SE3d &pose_) {
    std::unique_lock<std::mutex> lock(poseMutex);
    this->pose = pose_;
}

Sophus::SE3d Frame::getPose() {
    std::unique_lock<std::mutex> lock(poseMutex);
    return this->pose;
}