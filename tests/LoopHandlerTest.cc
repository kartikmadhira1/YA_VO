#include <gtest/gtest.h>
#include "../include/LoopHandler.hpp"
#include "../include/Utils.hpp"


TEST(LoopHandler, CopyConstructor) {
    std::string configPath = "../config/KITTI_mock_test.json";
    LoopHandler Lh(configPath);
    EXPECT_EQ(Lh.stereoStatus(), true);
    EXPECT_EQ(Lh.getSeqNo(), "00");
    EXPECT_EQ(Lh.getLeftImagesPath(), "../dataset/sequences/00/images_0/");

}