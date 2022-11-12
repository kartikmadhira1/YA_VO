#include <gtest/gtest.h>
#include "../include/FastDetector.hpp"



TEST(FastDetector, BresenhamCircleCheck) {

    cv::Mat testcv(cv::Size(50,50), CV_8UC1, cv::Scalar(0));
    cv::Mat testEQ = cv::imread("../tests/testBresenham.png", 0);
    Image testImage(testcv);
    FastDetector fd(12, 50);
    std::vector<cv::Point2i> bresPoints = fd.getBresenhamCirclePoints(testImage, 25, 25);
    // fd.putPixel(testImage, 25, 25);
    EXPECT_EQ(bresPoints.size(), 16);
    for (auto &eachVal : bresPoints) {
        fd.putPixel(testImage, eachVal);
    }
   

    EXPECT_TRUE(testImage.rawImage.rows == testEQ.rows);
    EXPECT_TRUE(testImage.rawImage.cols== testEQ.cols);
    // EXPECT_TRUE(testcv.rows == testEQ.rows);
    cv::Mat retMat;
    cv::Point *pos;

    cv::subtract(testImage.rawImage, testEQ, retMat);
    // cv::imwrite("test.png", retMat);

    bool eqCheck = cv::checkRange(retMat, true, pos, -1, 1);
    EXPECT_TRUE(eqCheck);
}


// Check if the getFastFeatures works on one specific example for a bresenham circle.


// First check checkContiguousPixels for a specific test case
TEST(FastDetector, CheckContiguosPixels) {

    cv::Mat testcv(cv::Size(50,50), CV_8UC1, cv::Scalar(0));
    // cv::Mat testEQ = cv::imread("../tests/testBresenham.png");
    // std::string s = type2str(testcv.type());
    Image testImage(testcv);
    FastDetector fd(12, 50);
    std::vector<cv::Point2i> bresPoints = fd.getBresenhamCirclePoints(testImage, 25, 25);
    // fd.putPixel(testImage, 25, 25);
    for (auto &each : bresPoints) {
        fd.putPixel(testImage, each);
    }

    bool checkContinuity = fd.checkContiguousPixels(testImage.getPixelVal(25, 25), bresPoints, testImage);

    EXPECT_EQ(checkContinuity, true);

    fd.putPixel(testImage, cv::Point(25, 25));

    checkContinuity = fd.checkContiguousPixels(testImage.getPixelVal(25, 25), bresPoints, testImage);

    EXPECT_EQ(checkContinuity, false);

}


TEST(FastDetector, CheckDiscontinuous) {

    cv::Mat testcv(cv::Size(50,50), CV_8UC1, cv::Scalar(0));
    // cv::Mat testEQ = cv::imread("../tests/testBresenham.png");
    // std::string s = type2str(testcv.type());
    Image testImage(testcv);
    FastDetector fd(12, 50);
    std::vector<cv::Point2i> bresPoints = fd.getBresenhamCirclePoints(testImage, 25, 25);
    // fd.putPixel(testImage, 25, 25);
    for (int i=0;i<11;i++) {
        fd.putPixel(testImage, cv::Point(bresPoints[i].x, bresPoints[i].y));
    }

    bool checkContinuity = fd.checkContiguousPixels(testImage.getPixelVal(25, 25), bresPoints, testImage);

    EXPECT_EQ(checkContinuity, false);
}



TEST(FastDetector, GetFeaturesTest) {

    // cv::Mat testcv(cv::Size(50,50), CV_8UC1, cv::Scalar(0));
    cv::Mat testcv = cv::imread("/home/kartik/da-sauce/hope/build/testFeat.png", 0);
    // std::string s = type2str(testcv.type());
    Image testImage(testcv);
    FastDetector fd(12, 50);
    auto features = fd.getFastFeatures(testImage);
    fd.putPixel(testImage, cv::Point(25, 25), 0);
    // cv::imwrite("testFeat.png", testImage.rawImage);

}
