#include "../include/FastDetector.hpp"



// Get all circle points with pixel centered at x, y
std::vector<cv::Point2i> FastDetector::getBresenhamCirclePoints(Image &img, int x, int y) {
    
    int xLoop = x;
    int yLoop = y + bresRadius;
    int d = 3-2*bresRadius;
    std::vector<cv::Point2i> circlePoints;
    while(yLoop<=xLoop) {
        xLoop++;

        if (d<0) {
            d = d + 4*xLoop + 6;
        } else {
            yLoop--;
            d = d + 4*(xLoop-yLoop) + 10;
        }
        // append points to vector;
        circlePoints.emplace_back(cv::Point2i(xLoop, yLoop));
    }
}