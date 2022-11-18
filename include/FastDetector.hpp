#pragma once
#include <iostream>
#include <set>
#include "Utils.hpp"
#include "Image.hpp"

/*
1. for pixel Pn in the image with intensity Ip
2. Set threshold of intensity value T, 
3. Consider 16 pixels (Bresenham circle points) surrounding the pixel Pn
4. ..

*/





class FastDetector {

    private:
        // "N" number used to detect if pixels are above or below threshold.
        int minDetectionThreshold;
        // Bresenham radius
        int bresRadius;
        uint8_t intensityThreshold;
        int fastCornerNumThreshold;
        int harrisThreshold;

    public :
        FastDetector(int minDetectionThresold=12, uint8_t intensityThreshold=50) {
            this->minDetectionThreshold=12;
            this->bresRadius=3;
            this->intensityThreshold=120;
            this->fastCornerNumThreshold=100;
            this->harrisThreshold=2;
        }
        void putPixelColor(Image &img, cv::Point pt);
        void putPixel(Image &img, cv::Point pt);
        void putPixel(Image &img, cv::Point pt, uint8_t pixVal);
        void convolve2d(const Image &img, cv::Mat &kernel, cv::Mat &output);
        void gaussianBlur(const Image &img, int sigma, cv::Mat &outImage);
        void preComputeHarris(const Image &img, cv::Mat &Ix, cv::Mat &Iy);
        float getHarrisCornerResponse(const Image &img, int x, int y);
        std::vector<cv::Point> getAllSymPoints(int x, int y);
        std::vector<cv::Point> getFastFeatures(const Image &img);
        std::vector<cv::Point> getBresenhamCirclePoints(const Image &img, int x, int y);
        bool checkContiguousPixels(uint8_t centPixel, const std::vector<cv::Point> &circlePoints, const Image &img);
        inline bool checkInBetween( uint8_t centPixel, uint8_t condPixel);

        ~FastDetector() {
        }
};