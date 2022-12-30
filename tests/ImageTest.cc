#include <gtest/gtest.h>
#include "../include/Image.hpp"
#include "../include/LoopHandler.hpp"
#include "../include/FastDetector.hpp"
#include "../include/Frame.hpp"
// #include "../include/LoopHandler.hpp"
// Test getPixel method 


// Hard rule on allowing only grayscale images 
TEST(LoopHandler, ImageDataType) {
    std::string configPath = "../config/KITTI_mock_test.json";
    LoopHandler Lh(configPath);
    // auto train = Lh.leftPathTrain();
    cv::Mat testImage = cv::imread( Lh.leftPathTrain[0], -1);

    std::string s = type2str(testImage.type());

    EXPECT_EQ(s, "8UC1");
}


TEST(ImageTest, GetPixelMethod) {
   
    cv::Mat testIm= cv::imread("../tests/testBresenham.png", 0);
    std::string s = type2str(testIm.type());
    EXPECT_EQ(s, "8UC1");
    Image testImage(testIm);
    FastDetector fd(12, 50);

    auto circlePoints = fd.getBresenhamCirclePoints(testImage, 25, 25);

    for (auto &eachPoint : circlePoints) {
        EXPECT_EQ(testImage.getPixelVal(eachPoint.x, eachPoint.y), 255);        
    }

}

