#include <gtest/gtest.h>
#include "../include/BriefDescriptor.hpp"
#include "../include/FastDetector.hpp"
#include "../include/3DHandler.hpp"
// Test getPixel method 


// // Hard rule on allowing only grayscale images 
// TEST(_3DHandler, CheckDisambiguateRT) {
//     Brief brief(256);

//     std::string configPath = "../config/KITTI_mock_test.json";
//     LoopHandler Lh(configPath);
//     cv::Mat testImage1 = cv::imread( Lh.leftPathTrain[0],0);
//     cv::Mat testImage2 = cv::imread( Lh.leftPathTrain[1],0);

//     Image testObj1(testImage1);
//     Image testObj2(testImage2);
//     FastDetector fd(12, 50);
//     auto features1 = fd.getFastFeatures(testObj1);
//     auto features2 = fd.getFastFeatures(testObj2);
    
//     brief.computeBrief(features1, testObj1);
//     brief.computeBrief(features2, testObj2);

//     std::cout << "Features1 size: " << features1.size() << std::endl;
//     std::cout << "Features2 size: " << features2.size() << std::endl;

    

//     std::vector<Matches> matches = brief.matchFeatures(testObj1, testObj2);

//     std::vector<Matches> filterMatches;

//     brief.removeOutliers(matches, filterMatches, 20.0);

//     std::cout << filterMatches.size() << std::endl;
//     cv::Mat sideBySide = brief.drawMatches(testObj1, testObj2, filterMatches);

//     _3DHandler handler("../tests/calib.txt");

//     handler.intrinsics.Left.printK();
//     cv::Mat E = cv::Mat::zeros(3, 3, CV_64F);
//     cv::Mat u = cv::Mat::zeros(3, 3, CV_64F);
//     cv::Mat vt = cv::Mat::zeros(3, 3, CV_64F);
//     cv::Mat w = cv::Mat::zeros(3, 3, CV_64F);
//     handler.getEssentialMatrix(filterMatches, E, u, w, vt);
//     Pose p = handler.disambiguateRT(E, u, w, vt, filterMatches);

//     // create skew symmetric matrix for t
//     cv::Mat t_hat = cv::Mat::zeros(3, 3, CV_64F);
//     t_hat.at<double>(0, 1) = -p.t.at<double>(2, 0);
//     t_hat.at<double>(0, 2) = p.t.at<double>(1, 0);
//     t_hat.at<double>(1, 0) = p.t.at<double>(2, 0);
//     t_hat.at<double>(1, 2) = -p.t.at<double>(0, 0);
//     t_hat.at<double>(2, 0) = -p.t.at<double>(1, 0);
//     t_hat.at<double>(2, 1) = p.t.at<double>(0, 0);
    
//     for (auto &eachMatch : filterMatches) {
//         double x1 = (eachMatch.pt1.y - handler.intrinsics.Left.getCx())/handler.intrinsics.Left.getFx();
//         double y1 = (eachMatch.pt1.x - handler.intrinsics.Left.getCy())/handler.intrinsics.Left.getFy();

//         double x2 = (eachMatch.pt2.y -handler.intrinsics.Left.getCx())/handler.intrinsics.Left.getFx();
//         double y2 =  (eachMatch.pt2.x -handler.intrinsics.Left.getCy())/handler.intrinsics.Left.getFy();

//         cv::Mat p1 = (cv::Mat_<double>(3, 1) << x1, y1, 1);
//         cv::Mat p2 = (cv::Mat_<double>(3, 1) << x2, y2, 1);
        
//         // std::cout << "p1 " << p1 << std::endl;
//         // std::cout << "p2 " << p2 << std::endl;
 
//         cv::Mat d = p2.t() * t_hat * p.R * p1;
        
        
//         std::cout << "d: " << d << std::endl;
//     }

// }



TEST(_3DHandler, DrawEpilines) {

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

    _3DHandler handler("../tests/calib.txt");

    handler.intrinsics.Left.printK();
    cv::Mat E = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat u = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat vt = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat w = cv::Mat::zeros(3, 3, CV_64F);
    handler.getEssentialMatrix(filterMatches, E, u, w, vt);
    

    // 1. Get fundamental matrix from essential matrix F = K1.inv().t() * E * K1.inv()
    // 2. Get epiline for the first image from fundamental matrix x2.t() * F = 0, x2 here is image coordinates
    // 3. Let this line be l in the form ax + by + c = 0
    // 4. To get a two points on the line we use x = 0 and y = -c/b, x = -c/a and y = 0

    cv::Mat F = handler.intrinsics.Left.getK().inv().t() * E * handler.intrinsics.Left.getK().inv();


    for (auto &eachMatch : filterMatches) {
        double x1 = eachMatch.pt1.x;
        double y1 = eachMatch.pt1.y;

        double x2 = eachMatch.pt2.x;
        double y2 = eachMatch.pt2.y;

        cv::Mat p1 = (cv::Mat_<double>(3, 1) << x1, y1, 1);
        cv::Mat p2 = (cv::Mat_<double>(3, 1) << x2, y2, 1);

        cv::Mat l = F.t() * p1;

        double a = l.at<double>(0, 0);
        double b = l.at<double>(0, 1);
        double c = l.at<double>(0, 2);
        cv::Point2i p1Line(-c/a, 0);
        cv::Point2i p2Line(0, -c/b);

        cv::line(testImage2, p1Line, p2Line, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    }

    cv::imwrite("../tests/epilines.png", testImage2);


}