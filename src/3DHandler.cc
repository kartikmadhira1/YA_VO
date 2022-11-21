#include "../include/3DHandler.hpp"




_3DHandler::_3DHandler(std::string calibPath) {
    getCalibParams(calibPath, intrinsics);
}



std::pair<double, double> _3DHandler::getMeanVar(std::vector<double> &vec) {
    double mean = 0.0;
    double var = 0.0;
    for (double &each : vec) {
        mean += each;
    }
    mean /= vec.size();
    for (double &each : vec) {
        var += (each - mean) * (each - mean);
    }
    var /= vec.size();
    return std::make_pair(mean, std::sqrt(var));
}

bool _3DHandler::getEssentialMatrix(const std::vector<Matches> &matches, cv::Mat &E, cv::Mat &u, cv::Mat &w, cv::Mat &vt) {
    // 1. Get all the points in w.r.t to the optical center of the image
    // 2. For each of the match keypoints, stack them in the Nx9 matrix
    // 3. Compute the essential matrix using the SVD

    // preload the camera matrix


    if (matches.size() < 8) {
        std::cout << "Not enough matches to compute the essential matrix" << std::endl;
        return false;
    }

    cv::Mat solveMat = cv::Mat::zeros(matches.size(), 9, CV_64F);

    std::vector<double> x1Vec, y1Vec, x2Vec, y2Vec;


    for (int i = 0; i < matches.size(); i++) {

        double x1 = matches[i].pt1.y;
        double y1 = matches[i].pt1.x;

        double x2 = matches[i].pt2.y;
        double y2 = matches[i].pt2.x;
        std::cout << "x1 " << x1 << " y1 " << y1 << std::endl;
        x1 = (x1 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        x2 = (x2 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        y1 =  (y1 -intrinsics.Left.getCy())/intrinsics.Left.getFy();
        y2 =  (y2 -intrinsics.Left.getCy())/intrinsics.Left.getFy();

        x1Vec.push_back(x1);
        y1Vec.push_back(y1);
        x2Vec.push_back(x2);
        y2Vec.push_back(y2);

    }



    for (int i = 0; i < matches.size(); i++) {

        float x1 = matches[i].pt1.y;
        float y1 = matches[i].pt1.x;

        float x2 = matches[i].pt2.y;
        float y2 = matches[i].pt2.x;
        x1 = (x1 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        x2 = (x2 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        y1 =  (y1 -intrinsics.Left.getCy())/intrinsics.Left.getFy();
        y2 =  (y2 -intrinsics.Left.getCy())/intrinsics.Left.getFy();

        std::cout << "x1 " << x1 << " y1 " << y1 << std::endl;

        auto meanVec1 = getMeanVar(x1Vec);
        auto meanVec2 = getMeanVar(y1Vec);
        auto meanVec3 = getMeanVar(x2Vec);
        auto meanVec4 = getMeanVar(y2Vec);

        double normX1 = (x1 - meanVec1.first)/meanVec1.second;
        double normY1 = (y1 - meanVec2.first)/meanVec2.second;
        double normX2 = (x2 - meanVec3.first)/meanVec3.second;
        double normY2 = (y2 - meanVec4.first)/meanVec4.second;
        //

        std::cout << "x1 norm " << normX1 << " y1 norm " << normY2 << std::endl;
 
        solveMat.at<double>(i, 0) = normX1 * normX2;
        solveMat.at<double>(i, 1) = normX1 * normY2;
        solveMat.at<double>(i, 2) = normX1;
        solveMat.at<double>(i, 3) = normY1 * normX2;
        solveMat.at<double>(i, 4) = normY1 * normY2;
        solveMat.at<double>(i, 5) = normY1;
        solveMat.at<double>(i, 6) = normX2;
        solveMat.at<double>(i, 7) = normY2;
        solveMat.at<double>(i, 8) = 1;
    }


    // Solve for given solveMat which is Mx9, with SVD for solveMat.t() * solveMat
    cv::SVD fullSolveSVD(solveMat, cv::SVD::FULL_UV); 
    
    // The last column of Vt is the null space
    // https://cmsc426.github.io/sfm/
 
    E = fullSolveSVD.vt.row(8).reshape(0, 3);
    std::cout << type2str(E.type()) << std::endl;
    cv::SVD ESolveSVD(E, cv::SVD::FULL_UV); 

    // Enforce the rank 2 constraint    
    w = cv::Mat::diag(ESolveSVD.w);
    w.at<double>(2, 2) = 0;
    vt = ESolveSVD.vt;
    u = ESolveSVD.u;
    E = ESolveSVD.u * w * ESolveSVD.vt;

    return true;

}



Pose _3DHandler::disambiguateRT(const cv::Mat &E, const cv::Mat &u, const cv::Mat &w, cv::Mat &vt, std::vector<Matches> &matches) {

    // Four possible solutions are possible for the essential matrix

    // 1. R = U * r(90).t()* Vt
    //    T = U * r(90)* w * Vt
    // 2. R = U * r(90).t()* Vt
    //    T = -U * r(90)* w * Vt
    // 3. R = U * r(-90).t()* Vt
    //    T = U * r(90)* w * Vt
    // 4. R = U * Wt * Vt
    //    T = -U * r(90)* w * Vt

    cv::Mat r90 = rotateMatrixZ(90);
    cv::Mat negr90 = rotateMatrixZ(-90);

    //Prepate points to triangulate
    std::vector<cv::Point2d> cam0Pnts;
    std::vector<cv::Point2d> cam1Pnts;
    // Get all points from matches 
    for (int i = 0; i < matches.size(); i++) {
        cam0Pnts.push_back(cv::Point(matches[i].pt1.y, matches[i].pt1.x));
        cam1Pnts.push_back(cv::Point(matches[i].pt2.y, matches[i].pt2.x));
    }
    // Placeholder for triangulated points
    cv::Mat pnts3D(4, cam0Pnts.size(), CV_64F);

    std::vector<Pose> poseTrain;

    // Condition 1
    cv::Mat R1 = u * r90.t() * vt;
    cv::Mat t = u.col(2);
    if (cv::determinant(R1) < 0) {
        R1 = -R1;
        t = -t;
    }
    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F);
    P1.at<double>(0, 3) = -t.at<double>(0);
    P1.at<double>(1, 3) = -t.at<double>(1);
    P1.at<double>(2, 3) = -t.at<double>(2);
    P1 = intrinsics.Left.K * R1 * P1;
    P1 = P1/P1.at<double>(2, 3);


    // Encapsulate in a Pose struct
    Pose pose1(R1, t, P1);

    cv::triangulatePoints(P1, P1, cam0Pnts, cam1Pnts, pnts3D);

    if (checkDepthPositive(pnts3D, pose1)) {
        poseTrain.push_back(pose1);
    }

    // Condition 2
    cv::Mat R2 = u * r90.t() * vt;
    cv::Mat t2 = -u.col(2);
    if (cv::determinant(R2) < 0) {
        R2 = -R2;
        t2 = -t2;
    }
    cv::Mat P2 = cv::Mat::eye(3, 4, CV_64F);
    P2.at<double>(0, 3) = -t2.at<double>(0);
    P2.at<double>(1, 3) = -t2.at<double>(1);
    P2.at<double>(2, 3) = -t2.at<double>(2);
    P2 = intrinsics.Left.K * R2 * P2;
    P2 = P2/P2.at<double>(2, 3);

    Pose pose2(R2, t2, P2);

    cv::triangulatePoints(P2, P2, cam0Pnts, cam1Pnts, pnts3D);
    if (checkDepthPositive(pnts3D, pose2)) {
        poseTrain.push_back(pose2);
    }


    // Condition 3
    cv::Mat R3 = u * negr90.t() * vt;
    cv::Mat t3 = u.col(2);
    if (cv::determinant(R3) < 0) {
        R3 = -R3;
        t3 = -t3;
    }
    cv::Mat P3 = cv::Mat::eye(3, 4, CV_64F);
    P3.at<double>(0, 3) = -t3.at<double>(0);
    P3.at<double>(1, 3) = -t3.at<double>(1);
    P3.at<double>(2, 3) = -t3.at<double>(2);
    P3 = intrinsics.Left.K * R3 * P3;
    P3 = P3/P3.at<double>(2, 3);

    Pose pose3(R3, t3, P3);

    cv::triangulatePoints(P3, P3, cam0Pnts, cam1Pnts, pnts3D);
    if (checkDepthPositive(pnts3D, pose3)) {
        poseTrain.push_back(pose3);
    }


    // Condition 4
    cv::Mat R4 = u * negr90.t() * vt;
    cv::Mat t4 = -u.col(2);
    if (cv::determinant(R4) < 0) {
        R4 = -R4;
        t4 = -t4;
    }
    cv::Mat P4 = cv::Mat::eye(3, 4, CV_64F);
    P4.at<double>(0, 3) = -t4.at<double>(0);
    P4.at<double>(1, 3) = -t4.at<double>(1);
    P4.at<double>(2, 3) = -t4.at<double>(2);
    P4 = intrinsics.Left.K * R4 * P4;

    P4 = P4/P4.at<double>(2, 3);
    Pose pose4(R4, t4, P4);

    cv::triangulatePoints(P4, P4, cam0Pnts, cam1Pnts, pnts3D);
    if (checkDepthPositive(pnts3D, pose4)) {
        poseTrain.push_back(pose4);
    }

    // Assert only two possible conditions where Z>0
    assert(poseTrain.size() == 2);


    // Choose the best pose
    if (poseTrain[0].numChierality > poseTrain[1].numChierality) {
        std::cout << poseTrain[0].numChierality << std::endl;
        std::cout << poseTrain[1].numChierality << std::endl;

        return poseTrain[0];
    }
    else {
        std::cout << poseTrain[0].numChierality << std::endl;

        std::cout << poseTrain[1].numChierality << std::endl;
        return poseTrain[1];
    }
}



bool _3DHandler::checkDepthPositive(cv::Mat &pnts3D, Pose &pose) {
    // Check if the depth of the points are positive
    for (int i = 0; i < pnts3D.cols; i++) {
        if ((pnts3D.at<double>(2, i)/pnts3D.at<double>(3, i)) < 0) {
            std::cout << "Depth is negative " << (pnts3D.at<double>(2, i)/pnts3D.at<double>(3, i)) << std::endl;
            return false;
        } else {
            pnts3D.at<double>(0, i) = (pnts3D.at<double>(0, i)/pnts3D.at<double>(3, i));
            pnts3D.at<double>(1, i) = (pnts3D.at<double>(1, i)/pnts3D.at<double>(3, i));
            pnts3D.at<double>(2, i) = (pnts3D.at<double>(2, i)/pnts3D.at<double>(3, i));
            pose.numChierality++;
        }
    }
    return true;
}

// Rotates input matrix by 90 degrees
cv::Mat _3DHandler::rotateMatrixZ(int rotateAngle) {
    cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64F);
    rotMat.at<double>(0, 0) = cos(rotateAngle);
    rotMat.at<double>(0, 1) = -sin(rotateAngle);
    rotMat.at<double>(1, 0) = sin(rotateAngle);
    rotMat.at<double>(1, 1) = cos(rotateAngle);
    rotMat.at<double>(2, 2) = 1;
    return rotMat;
}


_3DHandler::~_3DHandler() {
    // destructor
}