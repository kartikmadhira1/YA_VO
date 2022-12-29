#ifndef FRAME_H
#define FRAME_H
#include "Feature.hpp"
#include "Image.hpp"
#include <sophus/se3.hpp>
#include <memory>



class Frame : public Image {
    public:
        typedef std::shared_ptr<Frame> ptr;
        unsigned long frameID;
        Sophus::SE3d pose;
        std::vector<std::shared_ptr<Feature>> features;
        Frame() {}
        Frame(const cv::Mat &img, unsigned long _frameID) : Image(img), frameID(_frameID) {}
        void setPose(Sophus::SE3d &pose_);
        Sophus::SE3d getPose();
        static std::shared_ptr<Frame> CreateFrame();
    private:
        std::mutex poseMutex; //lock whenever accesing/writing to the object.

};


#endif // TODOITEM_H
