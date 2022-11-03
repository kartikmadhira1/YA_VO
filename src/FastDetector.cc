#include "../include/FastDetector.hpp"



// Get all circle points with pixel centered at x, y
std::vector<cv::Point2i> FastDetector::getBresenhamCirclePoints(Image &img, int xc, int yc) {
    
    int xLoop = 0;
    int yLoop = bresRadius;
    int d = 3-2*bresRadius;
    std::vector<cv::Point2i> circlePoints;
    while(yLoop>=xLoop) {
        xLoop++;

        if (d<=0) {
            d = d + 4*xLoop + 6;
        } else {
            yLoop--;
            d = d + 4*(xLoop-yLoop) + 10;
        }
        // append points to vector;
        // These points are w.r.t to x and y, but actual opencv image coordinates will have to be shifted
        int  xAct, yAct;
        std::vector<cv::Point2i> symPoints = getAllSymPoints(xLoop, yLoop);
        for (auto& eachVal : symPoints) {
            if (xLoop > 0) {
                xAct = std::abs(eachVal.x-xc);
            } else {
                xAct = eachVal.x+xc;
            }

            if (yLoop > 0) {
                yAct = std::abs(eachVal.y-yc);
            } else {
                yAct = eachVal.y+yc;
            }
            circlePoints.emplace_back(cv::Point2i(xAct, yAct));
        }
    }
    
    circlePoints.emplace_back(cv::Point2i(xc-bresRadius, yc));
    circlePoints.emplace_back(cv::Point2i(xc+bresRadius, yc));
    circlePoints.emplace_back(cv::Point2i(xc, yc+bresRadius));
    circlePoints.emplace_back(cv::Point2i(xc, yc-bresRadius));
    return circlePoints;
}

// 

std::vector<cv::Point2i> FastDetector::getAllSymPoints(int x, int y) {
    return {cv::Point(-x, -y), cv::Point(-x, y), cv::Point(x, -y), cv::Point(x, y), cv::Point(-y, -x), cv::Point(-y, x), cv::Point(y,-x)};
}

void FastDetector::putPixel(Image &img, int x, int y) {

    cv::Vec3b color(255, 255, 255);
    img.rawImage.at<cv::Vec3b>(cv::Point(x, y)) = color;
}

