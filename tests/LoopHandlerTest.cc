#include <gtest/gtest.h>
#include "../include/LoopHandler.hpp"
// #include "../include/Utils.cc"


TEST(LoopHandler, CopyConstructor) {
    std::string configPath = "../config/KITTI_mock_test.json";
    LoopHandler Lh(configPath);
    EXPECT_EQ(Lh.stereoStatus(), false);
    EXPECT_EQ(Lh.getSeqNo(), "00");
    EXPECT_EQ(Lh.getLeftImagesPath(), "../../dataset/sequences/00/image_0/");

}


TEST(LoopHandler, PathTrainTest) {
    std::string configPath = "../config/KITTI_mock_test.json";
    LoopHandler Lh(configPath);
    EXPECT_EQ(Lh.getLeftTrainLength(), 4541);

}



TEST(LoopHandler, ImageDataType) {
    std::string configPath = "../config/KITTI_mock_test.json";
    LoopHandler Lh(configPath);
    // auto train = Lh.leftPathTrain();
    cv::Mat testImage = cv::imread( Lh.leftPathTrain[0], -1);

    std::string s = type2str(testImage.type());

    EXPECT_EQ(s, "8UC1");
}