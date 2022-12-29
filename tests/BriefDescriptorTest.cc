#include <gtest/gtest.h>
#include "../include/BriefDescriptor.hpp"
#include "../include/LoopHandler.hpp"
#include "../include/FastDetector.hpp"
// Test getPixel method 


// Hard rule on allowing only grayscale images 
TEST(BriefDescritor, GetSideBySideImageMatches) {
    Brief brief(256);

    std::string configPath = "../config/KITTI_mock_test.json";
    LoopHandler Lh(configPath);
    cv::Mat testImage1 = cv::imread( Lh.leftPathTrain[0],0);
    cv::Mat testImage2 = cv::imread( Lh.leftPathTrain[1],0);

    Image testObj1(testImage1);
    Image testObj2(testImage2);
    FastDetector fd(12, 50);
    auto features1 = fd.getFastFeatures(testObj1);
    auto features2 = fd.getFastFeatures(testObj2);
    
    brief.computeBrief(features1, testObj1);
    brief.computeBrief(features2, testObj2);

    std::cout << "Features1 size: " << features1.size() << std::endl;
    std::cout << "Features2 size: " << features2.size() << std::endl;

    

    std::vector<Matches> matches = brief.matchFeatures(testObj1, testObj2);

    std::vector<Matches> filterMatches;

    brief.removeOutliers(matches, filterMatches, 20.0);

    std::cout << filterMatches.size() << std::endl;
    cv::Mat sideBySide = brief.drawMatches(testObj1, testObj2, filterMatches);
    cv::cvtColor(testImage1, testImage1, cv::COLOR_GRAY2RGB);
    cv::cvtColor(testImage2, testImage2, cv::COLOR_GRAY2RGB);

    for (cv::Point &each : features1) {
        cv::circle(testImage1, cv::Point(each.y, each.x), 2, cv::Scalar(0, 255, 0), 2);
    }
       for (cv::Point &each : features2) {
        cv::circle(testImage2, cv::Point(each.y, each.x), 2, cv::Scalar(0, 255, 0), 2);
    }

    cv::imwrite("image1.png", testImage1);
    cv::imwrite("image2.png", testImage2);


    cv::imwrite("testSideBySide.png", sideBySide);

}



