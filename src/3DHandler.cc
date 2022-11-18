#include "../include/3DHandler.hpp"



3DHandler::3DHandler(std::string calibPath) {
    getCalibParams(calibPath, intrinsics);
}

bool 3DHandler::getEssentialMatrix(const std::vector<Matches> &matches, cv::Mat &E, cv::Mat &u, cv::Mat &w, cv::Mat &vt) {
    // 1. Get all the points in w.r.t to the optical center of the image
    // 2. For each of the match keypoints, stack them in the Nx9 matrix
    // 3. Compute the essential matrix using the SVD

    // preload the camera matrix


    if (matches.size() < 8) {
        std::cout << "Not enough matches to compute the essential matrix" << std::endl;
        return false;
    }

    cv::Mat solveMat = cv::Mat::zeros(matches.size(), 9, CV_64F);

    for (int i = 0; i < matches.size(); i++) {

        int x1 = eachMatch.pt1.y;
        int y1 = eachMatch.pt1.x;

        int x2 = eachMatch.pt2.y;
        int y2 = eachMatch.pt2.x;

        x1 = -intrinsics.getCx() + x1;
        x2 = -intrinsics.getCx() + x2;
        y1 =  intrinsics.getCy() - y1;
        y2 =  intrinsics.getCy() - y2;

        //
        solveMat.at<double>(i, 0) = x2 * x1;
        solveMat.at<double>(i, 1) = y2 * x1;
        solveMat.at<double>(i, 2) = x2;
        solveMat.at<double>(i, 3) = y2 * x1;
        solveMat.at<double>(i, 4) = y2 * y1;
        solveMat.at<double>(i, 5) = y2;
        solveMat.at<double>(i, 6) = x1;
        solveMat.at<double>(i, 7) = y1;
        solveMat.at<double>(i, 8) = 1;
    }


    cv::Mat w, u, vt;  // w is the singular values
    cv::SVD::compute(solveMat, w, u, vt);

    // solveMat = u * w * vt
    // Put a hard constraint on the last singular value
    w.at<double>(8) = 0;
    w.at<double>(0) = (w.at<double>(0) + w.at<double>(4))/2;
    w.at<double>(4) = (w.at<double>(0) + w.at<double>(4))/2;


    E = u * cv::Mat::diag({w.at<double>(0),w.at<double>(4), w.at<double>(8)}) * vt;





    return true;


}



void 3DHandler::disambiguateRT(const cv::Mat &E, const cv::Mat &u, const cv::Mat &w, cv::Mat &vt, cv::Mat &finalR, cv::Mat &finalT, cv::vector<Matches> &matches) {

    // Four possible solutions are possible for the essential matrix

    // 1. R = U * r(90).t()* Vt
    //    T = U * r(90)* w * Vt
    // 2. R = U * r(90).t()* Vt
    //    T = -U * r(90)* w * Vt
    // 3. R = U * r(-90).t()* Vt
    //    T = U * r(90)* w * Vt
    // 4. R = U * Wt * Vt
    //    T = -U * r(90)* w * Vt

    cv::Mat w0 = cv::Mat::diag({w.at<double>(0),w.at<double>(4), w.at<double>(8)});
    cv::Mat r90 = rotateMatrixZ(90);
    cv::Mat negr90 = rotateMatrixZ(-90);

    //Prepate points to triangulate
    std::vector<cv::Point2d> cam0Pnts;
    std::vector<cv::Point2d> cam1Pnts;

    cam0Pnts.push_back(cv::Point2d(matches[0].pt1.y, matches[0].pt1.x));
    cam1Pnts.push_back(cv::Point2d(matches[0].pt2.y, matches[0].pt2.x));

    // Condition 1
    cv::Mat R1 = u * r90.t() * vt;
    cv::Mat t = u * r90 * w0 * vt;
    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64F);
    P1.at<double>(0, 3) = -t.at<double>(0);
    P1.at<double>(1, 3) = -t.at<double>(1);
    P1.at<double>(2, 3) = -t.at<double>(2);
    P1 = intrinsics.getK() * R1 * P1;
    cv::Mat pnts3D(4, cam0Pnts.size(),CV_64F);

    cv::triangulatePoints(P1, P1, cam0Pnts, cam1Pnts, matches, pnts3D);

}



// Rotates input matrix by 90 degrees
cv::Mat 3DHandler::rotateMatrixZ(int rotateAngle) {
    cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_64F);
    rotMat.at<double>(0, 0) = cos(rotateAngle);
    rotMat.at<double>(0, 1) = -sin(rotateAngle);
    rotMat.at<double>(1, 0) = sin(rotateAngle);
    rotMat.at<double>(1, 1) = cos(rotateAngle);
    rotMat.at<double>(2, 2) = 1;
    return rotMat;
}










3DHandler::~3DHandler() {
    // destructor
}