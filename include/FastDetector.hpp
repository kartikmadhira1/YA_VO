#include <iostream>
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

    public :
        FastDetector() {
            minDetectionThreshold=12;
            bresRadius=3;
        }
        void putPixel(Image &img, int x, int y);
        std::vector<cv::Point2i> getAllSymPoints(int x, int y);
        std::vector<cv::Point2i> getFastFeatures(Image &img);
        std::vector<cv::Point2i> getBresenhamCirclePoints(Image &img, int x, int y);
        ~FastDetector() {
        }
};