#include "../include/Optimizer.hpp"
#include "../include/3DHandler.hpp"
#include "../include/LoopHandler.hpp"
#include "../include/Frame.hpp"
#include "../include/FastDetector.hpp"
#include <gtest/gtest.h>






TEST(Optimizer, checkPartialBA) {
      // R and t obtained from the decomposing F matrix can be checked against the pose obtained from the partial BA

      // Get 3d points triangulated from the first two frames
      // Get 2d points from the first two frames

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

    _3DHandler handler;
    std::string calibPath = "../tests/calib.txt";
    handler.setCalibParams(calibPath);

    handler.intrinsics.Left.printK();
    cv::Mat F = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat u = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat vt = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat w = cv::Mat::zeros(3, 3, CV_64F);

    handler.getFRANSAC(filterMatches, F, 200, 0.1);
    
    // Essential matrix from fundamental matrix
    cv::Mat E = handler.intrinsics.Left.getK().t() * F * handler.intrinsics.Left.getK();


    Pose p = handler.disambiguateRT(E, filterMatches);



    // Get the triangulated points from the Pose of the second frame

    //Prepate points to triangulate
    std::vector<cv::Point2d> cam0Pnts;
    std::vector<cv::Point2d> cam1Pnts;
    // Get all points from matches 
    for (int i = 0; i < filterMatches.size(); i++) {
        cam0Pnts.push_back(cv::Point(filterMatches[i].pt1.x, filterMatches[i].pt1.y));
        cam1Pnts.push_back(cv::Point(filterMatches[i].pt2.x, filterMatches[i].pt2.y));
    }
    // Placeholder for triangulated points
    cv::Mat pnts3D(4, cam0Pnts.size(), CV_64F);
    std::cout << p.P << std::endl;
    cv::Mat P0 = (cv::Mat_<double>(3, 4) << 1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0);
    P0 = handler.intrinsics.Left.getK() * P0;
    cv::triangulatePoints(P0, p.P, cam0Pnts, cam1Pnts, pnts3D);


    // 1. Get fundamental matrix from essential matrix F = K1.inv().t() * E * K1.inv()
    // 2. Get epiline for the first image from fundamental matrix x2.t() * F = 0, x2 here is image coordinates
    // 3. Let this line be l in the form ax + by + c = 0
    // 4. To get a two points on the line we use x = 0 and y = -c/b, x = -c/a and y = 0

    vector<cv::Point3d> pts3D;
    vector<cv::Point2d> pts2D;
    for (int i = 0; i < pnts3D.cols; i++) {
        cv::Point3d pt3d(pnts3D.at<double>(0, i), pnts3D.at<double>(1, i), pnts3D.at<double>(2, i));
        std::cout << pt3d << std::endl;
        pts3D.push_back(pt3d);
    }

    std::cout << "3D points size for optim: " << pts3D.size() << std::endl;
    std::cout << "2D points size for optimization" << cam1Pnts.size() << std::endl; 
    Sophus::SE3d pose;
    Optimizer optim;

    optim.partialBA(pts3D, cam1Pnts, handler.intrinsics.Left.getK(), pose);

}


