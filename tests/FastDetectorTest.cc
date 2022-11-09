#include <gtest/gtest.h>
#include "../include/FastDetector.hpp"



TEST(FastDetector, bresenhamCircleCheck) {

    cv::Mat testcv(cv::Size(50,50), CV_8UC3, cv::Scalar(0));
    cv::Mat testEQ = cv::imread("../tests/testBresenham.png");
    Image testImage(testcv);
    FastDetector fd(12, 50);
    std::vector<cv::Point2i> bresPoints = fd.getBresenhamCirclePoints(testImage, 25, 25);
    // fd.putPixel(testImage, 25, 25);
    // // std::cout << bresPoints.size() << std::endl;
    int cnt = 0;
    for (auto &eachVal : bresPoints) {
        // std::cout << eachVal.x << " " << eachVal.y << std::endl;
        fd.putPixel(testImage, eachVal);
        // cv::imwrite("test" + std::to_string(cnt) + ".png", testImage.rawImage);

        cnt++;
    }
    // cv::imwrite("test1.png", testImage.rawImage);

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

