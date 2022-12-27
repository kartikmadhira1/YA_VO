#include <memory>
#include "Utils.hpp"


unsigned long Feature::featureID = 0;
class Feature {
    public:
        static unsigned long featureID;
        cv::KeyPoint kp;
        std::weak_ptr<Frame> frame;
        std::weak_ptr<MapPoint> mapPoint;
        Feature(){}
        Feature(std::shared_ptr<Frame> _frame, const cv::KeyPoint &_kp) : frame(_frame), kp(_kp) {
            featureID++;
        }
};



