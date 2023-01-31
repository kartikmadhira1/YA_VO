#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include "Utils.hpp"
#include "MapPoint.hpp"

class Frame;
class Feature {
    public:
        typedef std::shared_ptr<Feature> ptr;
        unsigned long featureID;
        cv::Point2d kp;
        std::weak_ptr<Frame> frame;
        std::weak_ptr<MapPoint> mapPoint;
        Feature(){}
        Feature(std::shared_ptr<Frame> _frame, const cv::Point2d &_kp) : frame(_frame) {
            this->kp.x = _kp.x;
            this->kp.y = _kp.y;
        }
        bool isOutlier = false;
};



#endif // TODOITEM_H
