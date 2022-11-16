#pragma once
#include "BriefDescriptor.hpp"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>



class KeyPoint;

// 1. See to it if Kitti dataset has any of the distortion coeff available.
// 2. Are the images raw or they are rectified?
// 3. Write rectifier if necessary
class Image {

    public :
        Image(const cv::Mat &img);
        cv::Mat rawImage;
        std::vector<KeyPoint> keypoints;
        int getW() const;
        int getH() const;
        void unDistort();
        uint8_t getPixelVal(int i, int j) const ;
        
};