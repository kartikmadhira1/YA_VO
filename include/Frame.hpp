#pragma once

#include "Feature.hpp"
#include <sophus/se3.hpp>
#include <memory>
#include "Image.hpp"



unsigned long Frame::frameID = 0;
class Frame : public Image {
    public:
        typedef std::shared_ptr<Frame> ptr;
        static unsigned long frameID;
        Sophus::SE3d pose;
        std::vector<std::shared_ptr<Feature>> features;
        Frame(){}
        Frame(const cv::Mat &img) : Image(const cv::Mat &img) {
            frameID++;
            pose = Sophus::SE3d();
        }
        void setPose(Sophus::SE3d &pose_);
        Sophus::SE3d getPose();
    private:
        std::mutex poseMutex; //lock whenever accesing/writing to the object.

}