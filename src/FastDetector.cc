#include "../include/FastDetector.hpp"

static bool compFirst(const std::pair<int, int> &pair1, const std::pair<int, int> &pair2) {
    if (pair1.second < pair2.second) {
        return true;
    } 
    if (pair1.second == pair2.second) {
        if (pair1.first >= pair2.first) {
            true;
        } else {
            false;
        }
    }
    return false;
}

static bool compSec(const std::pair<int, int> &pair1, const std::pair<int, int> &pair2) {
    if (pair1.second > pair2.second) {
        return true;
    } 
    if (pair1.second == pair2.second) {
        if (pair1.first < pair2.first) {
            true;
        } else {
            false;
        }
    }
    return false;
}

// Get all circle points with pixel centered at x, y
std::vector<cv::Point2i> FastDetector::getBresenhamCirclePoints(Image &img, int xc, int yc) {
    
    int xLoop = 0;
    int yLoop = bresRadius;
    int d = 3-2*bresRadius;
    std::vector<cv::Point2i> circlePoints;
    // circlePoints.reserve(16);
    std::vector<cv::Point2i> tempCirclePoints;
    std::set<std::pair<int, int>, decltype(compFirst)*> ordFirstHalf(compFirst);
    std::set<std::pair<int, int>, decltype(compSec)*> ordSecHalf(compSec);

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
        std::vector<cv::Point2i> symPoints = getAllSymPoints(xLoop, yLoop);
        for (int i=0;i<symPoints.size();i++) {
            int  xAct, yAct;

            if (symPoints[i].x >= 0) {
                xAct = xc + std::abs(symPoints[i].x);
            } else {
                xAct = xc - std::abs(symPoints[i].x);
            }

            if (symPoints[i].y >= 0) {
                yAct = yc - std::abs(symPoints[i].y);
            } else {
                yAct = yc + std::abs(symPoints[i].y);
            }
            if (symPoints[i].x >=0) {
  
                ordFirstHalf.insert(std::make_pair(xAct, yAct));
            } else {


                ordSecHalf.insert(std::make_pair(xAct, yAct));
            }
        }
    }

    ordFirstHalf.insert(std::make_pair(xc+bresRadius, yc));
    ordSecHalf.insert(std::make_pair(xc-bresRadius, yc));
    
    circlePoints.push_back(cv::Point(xc, yc-bresRadius));

    for (auto &eachVal : ordFirstHalf) {
        circlePoints.push_back(cv::Point(eachVal.first, eachVal.second));

    }
    circlePoints.push_back(cv::Point(xc, yc+bresRadius));
    for (auto &eachVal : ordSecHalf) {
        circlePoints.push_back(cv::Point(eachVal.first, eachVal.second));
    }
    return circlePoints;
}







std::vector<cv::Point2i> FastDetector::getAllSymPoints(int x, int y) {
    return {cv::Point(x, y) , cv::Point(y, x), cv::Point(y, -x), cv::Point(x, -y), cv::Point(-x, -y), cv::Point(-y, -x), cv::Point(-y, x), cv::Point(-x, y)};
}

void FastDetector::putPixel(Image &img, cv::Point pt) {

    cv::Vec3b *color = new cv::Vec3b(255, 255, 255);
    img.rawImage.at<cv::Vec3b>(pt) = *color;
}

