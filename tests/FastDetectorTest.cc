#include <gtest/gtest.h>
#include "../include/FastDetector.hpp"
// #include "../include/LoopHandler.hpp"


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


// // Check if the getFastFeatures works on one specific example for a bresenham circle.


// // First check checkContiguousPixels for a specific test case
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
    for (auto &each : features) {
         std::cout << each << std::endl;
            // fd.putPixelColor(testRGBImage, each);
            // cv::circle(imgRGB, each, 5, cv::Scalar(255, 255, 255), 2);
        }
    // fd.putPixel(testImage, cv::Point(25, 25), 0);
    // cv::imwrite("testFeat.png", testImage.rawImage);

}




TEST(FastDetectors, CheckCorners) {
    std::string configPath = "../config/KITTI_mock_test.json";
    LoopHandler Lh(configPath);
    cv::Mat testImage = cv::imread( Lh.leftPathTrain[1],0);
    Image testImageObj(testImage);
    FastDetector fd(12, 50);
    auto features = fd.getFastFeatures(testImageObj);
    
    cv::Mat imgRGB;

    cv::cvtColor(testImage, imgRGB, cv::COLOR_GRAY2RGB);
    for (cv::Point &each : features) {
        cv::circle(imgRGB, cv::Point(each.y, each.x), 2, cv::Scalar(0, 255, 0), 2);
    }
    // std::cout << type2str(testImage.type()) << std::endl;
    // std::cout << type2str(testAgain.type()) << std::endl;

    cv::imwrite("testAllFeat.png", imgRGB);

}