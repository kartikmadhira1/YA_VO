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



std::vector<cv::Point2i> FastDetector::getFastFeatures( Image &img) {
    // core implementation of the fast features
    // 1. For each pixel in the image
    // 2. Get the bresenham continuos points for the pixel
    // 3. First check the indices 0, 8, if BOTH the pixel falls in the range [Ip-thres, Ip+thres], reject point
    // 4. If 3 is false then check indices 5, 13 in the range [Ip-thres, Ip+thres], if ONE of them is true, then move to 5.
    // 5. If 3 and 4 false, then check if the 12 continous pixels DONT fall in the zone.

    std::vector<cv::Point2i> retCorners;
    // uint8_t pixelVal = (uint8_t)img.rawImage.data;
    for (int i=4;i<img.rawImage.rows-4;i++) {
        for(int j=4;j<img.rawImage.cols-4;j++) {
            uint8_t centPixel = img.getPixelVal(i, j);
            std::vector<cv::Point2i> circlePoints = getBresenhamCirclePoints(img.rawImage, i, j);

            // Check conditions
            cv::Point2i P1 = circlePoints[0];
            cv::Point2i P8 = circlePoints[7];
            cv::Point2i P5 = circlePoints[4];
            cv::Point2i P13 = circlePoints[12];

            uint8_t p1Val = img.getPixelVal(P1.x, P1.y);
            uint8_t p8Val = img.getPixelVal(P8.x, P8.y);
            uint8_t p5Val = img.getPixelVal(P5.x, P5.y);
            uint8_t p13Val = img.getPixelVal(P13.x, P13.y);
            
            // TEST PERFORMANCE USING THIS V/S USING AN INLINE FUNCTION TO CHECK THIS
            if (checkIntBetween(centPixel, p1Val) && checkIntBetween(centPixel, p8Val)) {
                continue;
            }
            if (checkIntBetween(centPixel, p5Val) || checkIntBetween(centPixel, p13Val)) {
                // check for contiguous 12 pixels 

                if (checkContiguousPixels(centPixel, circlePoints)) {
                    // This is a valid corner
                    retCorners.push_back(cv::Point2i(i, j));
                }

            } else {
                continue;
            }

        }
    }
}




bool FastDetector::checkContiguousPixels(uint8_t centPixel, const std::vector<cv::Point2i> &circlePoints) {
    int currInd = 0;
    int pixCount = 0;
    while (currInd < 16) {
        if (checkIntBetween(centPixel, circlePoints[currInd])) {
            pixCount=0;
            currInd++;
        } else {
            pixCount++;
            currInd++;
        }
        if (pixCount>=12) {
            return true;
        }
    }
}

inline bool FastDetector::checkIntBetween(uint8_t centPixel, uint8_t condPixel) {
    if ((centPixel > condPixel - intensityThreshold) && (centPixel < condPixel + intensityThreshold)) {return true;}
    return false;
}
