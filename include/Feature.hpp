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
        cv::Point2i kp;
        std::weak_ptr<Frame> frame;
        std::weak_ptr<MapPoint> mapPoint;
        Feature(){}
        Feature(std::shared_ptr<Frame> _frame, const cv::Point2i &_kp) : frame(_frame), kp(_kp) {}
};



#endif // TODOITEM_H
