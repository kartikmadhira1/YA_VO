#include <gtest/gtest.h>
#include "../include/Utils.hpp"

TEST(UtilsCheck, stringParseCheck) {
      cv::Mat _cv(4, 4, CV_64F);
      std::string stringCheck = "P0: 7.1 8.2 8.3 9.3 10.3 11 12 13 14 15 16 17 18 19 20 21";      parseCalibString(stringCheck, _cv);
      
      EXPECT_EQ(_cv.at<double>(0,0), (double)7.1);
      EXPECT_EQ(_cv.at<double>(0,2), (double)8.3);
      EXPECT_EQ(_cv.at<double>(1,2), (double)12);
      EXPECT_EQ(_cv.at<double>(3,3), (double)21);

}



TEST(UtilsCheck, checkInstrinsicIntegrity) {
      std::string path = "/home/kartik/da-sauce/dataset/sequences/00/calib.txt";
      Intrinsics intrinsics;
      getCalibParams(path, intrinsics);
      // intrinsics.left
}