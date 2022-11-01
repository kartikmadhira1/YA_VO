#pragma once
#include "Utils.hpp"
#include "FeatureExtractor.hpp"






// 1. See to it if Kitti dataset has any of the distortion coeff available.
// 2. Are the images raw or they are rectified?
// 3. Write rectifier if necessary

class Image {

    public :
        Image();
        cv::Mat img;
        cv::Mat rawImage;
        FeatExt _features;
        Image(const cv::Mat &img);
        void unDistort();
        ~Image();
        
};