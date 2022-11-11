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
        int intensityThreshold;

    public :
        FastDetector(int minDetectionThresold=12, int intensityThreshold=50) {
            minDetectionThreshold=12;
            bresRadius=3;
            intensityThreshold=50;
        }
        void putPixel(Image &img, cv::Point pt);
        std::vector<cv::Point2i> getAllSymPoints(int x, int y);
        std::vector<cv::Point2i> getFastFeatures(const Image &img);
        std::vector<cv::Point2i> getBresenhamCirclePoints(const Image &img, int x, int y);
        bool checkContiguousPixels(uint8_t centPixel, const std::vector<cv::Point2i> &circlePoints, const Image &img);
        inline bool checkInBetween( uint8_t centPixel, uint8_t condPixel);
        ~FastDetector() {
        }
};