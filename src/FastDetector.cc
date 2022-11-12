#include "../include/FastDetector.hpp"

struct FastFeature {
    public:
        FastFeature(const cv::Point2i &pt, float cornerResponse) {
            this->x = pt.x;
            this->y = pt.y;
        }
        int x;
        int y;
        float cornerResponse;
}




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
std::vector<cv::Point2i> FastDetector::getBresenhamCirclePoints(const Image &img, int xc, int yc) {
    
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

    uint8_t *color = new uint8_t(255);
    img.rawImage.at<uint8_t>(pt) = *color;
}

void FastDetector::putPixel(Image &img, cv::Point pt, uint8_t pixVal) {

    uint8_t *color = new uint8_t(pixVal);
    img.rawImage.at<uint8_t>(pt) = *color;
}



std::vector<cv::Point2i> FastDetector::getFastFeatures(const Image &img) {
    // core implementation of the fast features
    // 1. For each pixel in the image
    // 2. Get the bresenham continuos points for the pixel
    // 3. First check the indices 0, 8, if BOTH the pixel falls in the range [Ip-thres, Ip+thres], reject point
    // 4. If 3 is false then check indices 5, 13 in the range [Ip-thres, Ip+thres], if ONE of them is true, then move to 5.
    // 5. If 3 and 4 false, then check if the 12 continous pixels DONT fall in the zone.

    std::vector<FastFeature> retCorners;
    // uint8_t pixelVal = (uint8_t)img.rawImage.data;
    for (int i=4;i<img.rawImage.rows-4;i++) {
        for(int j=4;j<img.rawImage.cols-4;j++) {
            uint8_t centPixel = img.getPixelVal(i, j);
            std::vector<cv::Point2i> circlePoints = getBresenhamCirclePoints(img, i, j);

            // Check conditions
            cv::Point2i P1 = circlePoints[0];
            cv::Point2i P8 = circlePoints[7];
            cv::Point2i P5 = circlePoints[4];
            cv::Point2i P13 = circlePoints[12];

            uint8_t p1Val = img.getPixelVal(P1.x, P1.y);
            uint8_t p8Val = img.getPixelVal(P8.x, P8.y);
            uint8_t p5Val = img.getPixelVal(P5.x, P5.y);
            uint8_t p13Val = img.getPixelVal(P13.x, P13.y);
            
            // TEST PERFORMANCE USING THIS V/S USING AN INLINE FUNCTION TO CHECK THIS CONDITION
            if ((!checkInBetween(centPixel, p1Val)) && (!checkInBetween(centPixel, p8Val))) {

                if (!checkInBetween(centPixel, p5Val) || !checkInBetween(centPixel, p13Val)) {
                    // check for contiguous 12 pixels 

                    if (checkContiguousPixels(centPixel, circlePoints, img)) {
                        // This is a valid corner
                        float corScore = getHarrisCornerResponse(img, i, j)
                        FastFeature newFeature(cv::Point2i(i, j), corScore);
                        retCorners.push_back(newFeature);
                    }

                } else {
                    continue;
                }
            }

        }
    }

    // sort the corners based on the score

    std::sort(retCorners.begin(), retCorners.end(), [](const FastFeature &a, const FastFeature &b) {
        return a.cornerScore > b.cornerScore;
    });
    return retCorners;
}




bool FastDetector::checkContiguousPixels(uint8_t centPixel, const std::vector<cv::Point2i> &circlePoints, const Image &img) {
    int currInd = 0;
    int pixCount = 0;
    while (currInd < 16) {
        // std::cout << pixCount << std::endl;
        if (checkInBetween(centPixel, img.getPixelVal(circlePoints[currInd].x, circlePoints[currInd].y))) {
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

    return false;
}

inline bool FastDetector::checkInBetween( uint8_t centPixel,  uint8_t condPixel) {
    if ((centPixel > condPixel - intensityThreshold) && (centPixel < condPixel + intensityThreshold)) {
        return true;
    }
    return false;
}

// Get harris corner response for a single FAST detected corner point
void FastDetector::convolve2d(const Image &img, cv::Mat kernel, cv::Mat &output) {
    // 1. Get the kernel size
    // 2. Get the image size
    // 3. For each pixel in the image
    // 4. Get the kernel size around the pixel
    // 5. Multiply the kernel with the image patch
    // 6. Sum the values and store in the output image
    // 7. Return the output image

    // Add border to the original image

    cv::Mat imgWithBorder;
    cv::copyMakeBorder(img.rawImage, imgWithBorder, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);

    int kernelSize = kernel.rows;
    int imgSize = img.rawImage.rows;
    cv::Mat outputImg = cv::Mat::zeros(imgSize, imgSize, CV_32FC1);
    for (int i=0;i<imgSize;i++) {
        for (int j=0;j<imgSize;j++) {
            float sum = 0;
            for (int k=0;k<kernelSize;k++) {
                for (int l=0;l<kernelSize;l++) {
                    sum += kernel.at<float>(k, l) * imgWithBorder.at<uint8_t>(i, j);
                }
            }
            outputImg.at<float>(i, j) = sum;
        }
    }
}



void FastDetector::preComputeHarris(const Image &img, cv::Mat &Ix, cv::Mat &Iy) { 
    // precompute the harris moments for the image

    // Get x derivate first with sobel operator
    cv::Mat sobelx = (cv::Mat_<int>(3,3) << -1, 0, 1, -2, 0, 2, -1, 0, 1);
    cv::Mat sobely = (cv::Mat_<int>(3,3) << -1, -2, -1, 0, 0, 0, 1, 2, 1);

    convolve2d(img, sobelx, Ix);
    convolve2d(img, sobely, Iy);

}
float FastDetector::getHarrisCornerResponse(const Image &img, int x, int y) {
    // 1. Convolve image with x and y derivative kernels Ix, Iy
    // 2. Compute Ix^2, Iy^2, Ix*Iy
    // 3. 
    cv::Mat Ix = cv::Mat::zeros(img.rawImage.rows, img.rawImage.cols, CV_32FC1);
    cv::Mat Iy = cv::Mat::zeros(img.rawImage.rows, img.rawImage.cols, CV_32FC1);
    preComputeHarris(img, Ix, Iy);

    cv::Mat Ix2 = Ix.mul(Ix);
    cv::Mat Iy2 = Iy.mul(Iy);
    cv::Mat Ixy = Ix.mul(Iy);

    cv::Mat M = cv::Mat::zeros(2, 2, CV_32FC1);

    for (int i=x-1;i<=x+1;i++) {
        for (int j=y-1;j<=y+1;j++) {
            M.at<float>(0, 0) += Ix2.at<float>(i, j);
            M.at<float>(0, 1) += Ixy.at<float>(i, j);
            M.at<float>(1, 0) += Ixy.at<float>(i, j);
            M.at<float>(1, 1) += Iy2.at<float>(i, j);
        }
    }

    cv::Mat eigenValues;
    cv::eigen(M, eigenValues);
    
    float harrisResponse = eigenValues.at<float>(0, 0) * eigenValues.at<float>(1, 1) - eigenValues.at<float>(0, 1) * eigenValues.at<float>(1, 0) - 0.04 * (eigenValues.at<float>(0, 0) + eigenValues.at<float>(1, 1)) * (eigenValues.at<float>(0, 0) + eigenValues.at<float>(1, 1));

    return harrisResponse;
}
    


