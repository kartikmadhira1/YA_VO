#include <gtest/gtest.h>
#include "../include/LoopHandler.hpp"
// #include "../include/Utils.cc"
// #include "../include/FastDetector.hpp"  
// #include "../include/BriefDescriptor.hpp"

// TEST(LoopHandler, CopyConstructor) {
//     std::string configPath = "../config/KITTI_mock_test.json";
//     LoopHandler Lh(configPath);
//     EXPECT_EQ(Lh.stereoStatus(), false);
//     EXPECT_EQ(Lh.getSeqNo(), "00");
//     EXPECT_EQ(Lh.getLeftImagesPath(), "../../dataset/sequences/00/image_0/");

// }


// TEST(LoopHandler, PathTrainTest) {
//     std::string configPath = "../config/KITTI_mock_test.json";
//     LoopHandler Lh(configPath);
//     EXPECT_EQ(Lh.getLeftTrainLength(), 4541);

// }


// TEST(LoopHandler, checkNextFrameDims) {
//     std::string configPath = "../config/KITTI_mock_test.json";
//     LoopHandler Lh(configPath);
//     auto nextFrame = Lh.getNextFrame();
//     EXPECT_EQ(nextFrame->getW(), 1241);
//     EXPECT_EQ(nextFrame->getH(), 376);
// }


// TEST(LoopHandler, checkFrameIDs) {
//     std::string configPath = "../config/KITTI_mock_test.json";
//     LoopHandler Lh(configPath);
//     auto nextFrame = Lh.getNextFrame();
//     EXPECT_EQ(nextFrame->frameID, 0);
//     nextFrame = Lh.getNextFrame();
//     EXPECT_EQ(nextFrame->frameID, 1);
//     nextFrame = Lh.getNextFrame();
//     EXPECT_EQ(nextFrame->frameID, 2);
// }


// // TEST(LoopHandler, checkFeatureExtraction) {
// //     std::string configPath = "../config/KITTI_mock_test.json";
// //     LoopHandler Lh(configPath);
// //     auto nextFrame = Lh.getNextFrame();
// //     FastDetector fd(12, 50);
// //     Brief brief(256);
// //     auto features = fd.getFastFeatures(*nextFrame);
// //     brief.computeBrief(features, *nextFrame);
// //     EXPECT_EQ(features.size(), 70);
// // }


// // TEST(LoopHandler, checkFeatureInsertion) {
// //     std::string configPath = "../config/KITTI_mock_test.json";
// //     LoopHandler Lh(configPath);
// //     Frame::ptr nextFrame = Lh.getNextFrame();
// //     Lh.insertFrameFeatures(nextFrame);
// //     EXPECT_EQ(nextFrame->features.size(), 70);
// // }




TEST(LoopHandler, checkVOStep) {
    std::string configPath = "../config/KITTI.json";
    LoopHandler Lh(configPath);
    Lh.handler3D.intrinsics.Left.printK();
    Lh.viz->viewerRun();
    Lh.runVO();
}