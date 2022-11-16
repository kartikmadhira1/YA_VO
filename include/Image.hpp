#pragma once
#include "Utils.hpp"
#include "BriefDescriptor.hpp"






// 1. See to it if Kitti dataset has any of the distortion coeff available.
// 2. Are the images raw or they are rectified?
// 3. Write rectifier if necessary

class Image {

    public :
        Image() {

        }
        cv::Mat img;
        cv::Mat rawImage;
        std::vector<KeyPoint> keypoints;
        Image(const cv::Mat &img);
        int getW();
        int getH();
        void unDistort();
        uint8_t getPixelVal(int i, int j) const ;
        ~Image() {
            
        }
        
};