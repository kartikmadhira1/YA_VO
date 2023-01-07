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
        std::vector<Sophus::SE3d> sisPoses;
        std::vector<std::shared_ptr<Feature>> features;
        Frame() {}
        Frame(const cv::Mat &img, unsigned long _frameID) : Image(img), frameID(_frameID) {
            Eigen::Matrix<double, 3, 3> R;
            R << 1,0,0,0,1,0,0,0,1;
            Eigen::Matrix<double, 3, 1> t;
            t  << 0, 0, 0;
            Sophus::SE3d identPose(R, t);
            this->pose = identPose;
        }
        void setPose(Sophus::SE3d &pose_);
        Sophus::SE3d getPose();
        static std::shared_ptr<Frame> CreateFrame();
    private:
        std::mutex poseMutex; //lock whenever accesing/writing to the object.

};


#endif // TODOITEM_H
