#include "Image.hpp"
#include "Feature.hpp"
#include <sophus/se3.hpp>





unsigned long Frame::frameID = 0;
class Frame : public Image {
    public:
        static unsigned long frameID;
        Sophus::SE3d pose;
        Frame(const cv::Mat &img) : Image(const cv::Mat &img) {
            frameID++;
        }

}