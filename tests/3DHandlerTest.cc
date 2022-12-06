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
    cv::Mat F = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat u = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat vt = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat w = cv::Mat::zeros(3, 3, CV_64F);
    std::pair<float, float> meanVec1, meanVec2, meanVec3, meanVec4;
    std::pair<float, float> m1, m2, m3, m4;
    handler.getFRANSAC(filterMatches, F, m1, m2, m3, m4, 200, 0.1);
    

    // 1. Get fundamental matrix from essential matrix F = K1.inv().t() * E * K1.inv()
    // 2. Get epiline for the first image from fundamental matrix x2.t() * F = 0, x2 here is image coordinates
    // 3. Let this line be l in the form ax + by + c = 0
    // 4. To get a two points on the line we use x = 0 and y = -c/b, x = -c/a and y = 0

    // get image height and width
    int height = testImage1.rows;
    int width = testImage1.cols;


    std::vector<double> x1Vec, y1Vec, x2Vec, y2Vec;


    for (int i = 0; i < matches.size(); i++) {

        double x1 = matches[i].pt1.x;
        double y1 = matches[i].pt1.y;

        double x2 = matches[i].pt2.x;
        double y2 = matches[i].pt2.y;

        // These points are now normalized image coordinates with the optical center as reference
        // x1 = (x1 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        // x2 = (x2 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        // y1 =  (y1 -intrinsics.Left.getCy())/intrinsics.Left.getFy();
        // y2 =  (y2 -intrinsics.Left.getCy())/intrinsics.Left.getFy();

        x1Vec.push_back(x1);
        y1Vec.push_back(y1);
        x2Vec.push_back(x2);
        y2Vec.push_back(y2);

    }

    for (auto &eachMatch : filterMatches) {
        float x1 = eachMatch.pt1.x;
        float y1 = eachMatch.pt1.y;

        float x2 = eachMatch.pt2.x;
        float y2 = eachMatch.pt2.y;
        // x1 = (x1 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        // x2 = (x2 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        // y1 =  (y1 -intrinsics.Left.getCy())/intrinsics.Left.getFy();
        // y2 =  (y2 -intrinsics.Left.getCy())/intrinsics.Left.getFy();


        auto meanVec1 = handler.getMeanVar(x1Vec);
        auto meanVec2 = handler.getMeanVar(y1Vec);
        auto meanVec3 = handler.getMeanVar(x2Vec);
        auto meanVec4 = handler.getMeanVar(y2Vec);

        float normX1 = (x1 - meanVec1.first)/(meanVec1.second + std::numeric_limits<float>::epsilon());
        float normY1 = (y1 - meanVec2.first)/(meanVec2.second + std::numeric_limits<float>::epsilon());
        float normX2 = (x2 - meanVec3.first)/(meanVec3.second + std::numeric_limits<float>::epsilon());
        float normY2 = (y2 - meanVec4.first)/(meanVec4.second + std::numeric_limits<float>::epsilon());


        cv::Mat p1 = (cv::Mat_<float>(3, 1) << normX1, normY1, 1);
        cv::Mat p2 = (cv::Mat_<float>(3, 1) << normX2, normY2, 1);

        std::cout<< "p1: " << p1 << std::endl;
        std::cout<< "p2: " << p2 << std::endl;
        cv::Mat l = F.t() * p1;

        double a = l.at<double>(0, 0);
        double b = l.at<double>(0, 1);
        double c = l.at<double>(0, 2);
        cv::Point2f p1Line(handler.unNormalizePoint(-c/b, meanVec3.first, meanVec3.second), 0);
        cv::Point2f p2Line(handler.unNormalizePoint(-(a*width + c)/b, meanVec3.first, meanVec3.second), handler.unNormalizePoint(width, meanVec4.first, meanVec4.second));

        cv::line(testImage2, p1Line, p2Line, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        cv::Mat d = p2.t() * F * p1;

        std::cout << "Distance: " << d << std::endl;
        std::cout << p1Line << " " << p2Line << std::endl;
        cv::line(testImage2, p1Line, p2Line, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    }

    cv::imwrite("../tests/epilines.png", testImage2);


}



TEST(_3DHandler, DrawEpilinesOpenCV) {

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

    // Get the fundamental matrix from opencv



    std::vector<cv::Point2i> points1;
    std::vector<cv::Point2i> points2;

    for (auto &eachPoint : filterMatches) {
        points1.push_back(cv::Point2i(eachPoint.pt1.x, eachPoint.pt1.y));
        points2.push_back(cv::Point2i(eachPoint.pt2.x, eachPoint.pt2.y));
    }


    cv::Mat F = cv::findFundamentalMat(points1, points2, cv::FM_8POINT, 3.0, 0.99);

    // get image height and width
    int height = testImage1.rows;
    int width = testImage1.cols;

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
        cv::Point2i p1Line(-c/b, 0);
        cv::Point2i p2Line(-(a*width + c)/b, width);

        cv::line(testImage2, p1Line, p2Line, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        cv::Mat d = p2.t() * F * p1;

        std::cout << p1Line << " " << p2Line << std::endl;

        std::cout << "Distance: " << d << std::endl;

    }

    cv::imwrite("../tests/epilinesOpencv.png", testImage2);


}






