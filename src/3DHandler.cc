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

bool _3DHandler::getFundamentalMatrix(const std::vector<Matches> &matches, cv::Mat &F) {
    // 1. Get all the points in w.r.t to the optical center of the image
    // 2. For each of the match keypoints, stack them in the Nx9 matrix
    // 3. Compute the essential matrix using the SVD

    // preload the camera matrix


    if (matches.size() < 8) {
        std::cout << "Not enough matches to compute the essential matrix" << std::endl;
        return false;
    }

    cv::Mat solveMat = cv::Mat::zeros(matches.size(), 9, CV_32FC1);

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



    for (int i = 0; i < matches.size(); i++) {

        float x1 = matches[i].pt1.x;
        float y1 = matches[i].pt1.y;

        float x2 = matches[i].pt2.x;
        float y2 = matches[i].pt2.y;
        // x1 = (x1 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        // x2 = (x2 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        // y1 =  (y1 -intrinsics.Left.getCy())/intrinsics.Left.getFy();
        // y2 =  (y2 -intrinsics.Left.getCy())/intrinsics.Left.getFy();


        auto meanVec1 = getMeanVar(x1Vec);
        auto meanVec2 = getMeanVar(y1Vec);
        auto meanVec3 = getMeanVar(x2Vec);
        auto meanVec4 = getMeanVar(y2Vec);

        float normX1 = (x1 - meanVec1.first)/(meanVec1.second + std::numeric_limits<float>::epsilon());
        float normY1 = (y1 - meanVec2.first)/(meanVec2.second + std::numeric_limits<float>::epsilon());
        float normX2 = (x2 - meanVec3.first)/(meanVec3.second + std::numeric_limits<float>::epsilon());
        float normY2 = (y2 - meanVec4.first)/(meanVec4.second + std::numeric_limits<float>::epsilon());
        //
 
        solveMat.at<float>(i, 0) = normX1 * normX2;
        solveMat.at<float>(i, 1) = normX1 * normY2;
        solveMat.at<float>(i, 2) = normX1;
        solveMat.at<float>(i, 3) = normY1 * normX2;
        solveMat.at<float>(i, 4) = normY1 * normY2;
        solveMat.at<float>(i, 5) = normY1;
        solveMat.at<float>(i, 6) = normX2;
        solveMat.at<float>(i, 7) = normY2;
        solveMat.at<float>(i, 8) = 1;
    }


    // Solve for given solveMat which is Mx9, with SVD for solveMat.t() * solveMat
    cv::SVD fullSolveSVD(solveMat.t()*solveMat, cv::SVD::FULL_UV); 
    
    // The last column of Vt is the null space

    F = fullSolveSVD.vt.row(8).reshape(1, 3);

    // // ########WARNING -> E IS F MATRIX FOR NOW FOR TESTING PURPOSES

    // E = intrinsics.Left.getK().t() * E * intrinsics.Left.getK();

    // std::cout << "vt last row " << fullSolveSVD.vt.row(8)<< std::endl;
    // std::cout << "E :" << E << std::endl;

    // std::cout << E.rows << "<---row cols---->" << E.cols << std::endl;

    // std::cout << type2str(E.type()) << std::endl;
    cv::SVD FSolveSVD(F, cv::SVD::FULL_UV);

    // Enforce the rank 2 constraint
 
    FSolveSVD.w.at<float>(2) = 0;


    cv::Mat w = cv::Mat::diag(FSolveSVD.w);
    F = FSolveSVD.u * w * FSolveSVD.vt;
    F = F/F.at<float>(2,2);
    return true;

}

float _3DHandler::unNormalizePoint(float pt, float mean, float var) {
    return (pt * var) + mean;
}

bool _3DHandler::getFRANSAC(std::vector<Matches> matches, cv::Mat &F, std::pair<float, float> meanVar1, std::pair<float, float> meanVar2, 
                                std::pair<float, float> meanVar3, std::pair<float, float> meanVar4,  int iterations=300, double threshold=0.1) {   

    // 1. For N iterations, pick 8 random matches
    // 2. Compute the fundamental matrix using the 8 matches
    // 3. Compute the error for each match
    // 4. If the error is less than a threshold, add it to the inlier set
    // 5. If the inlier set is larger than the best inlier set, replace it
    // 6. Refit the fundamental matrix using the inlier set
    // 7. Return the fundamental matrix
    if (matches.size() < 8) {
        return false;
    }
    std::random_device seeder;
    std::mt19937 generator(seeder());
    std::uniform_int_distribution<int> dist(0, matches.size()-1);
    cv::Mat finalF;
    int maxInliers = INT_MIN;
    for (int i = 0; i < iterations; i++) {
        std::vector<Matches> newMatches;
        for (int j = 0; j < 8; j++) {
            newMatches.push_back(matches[dist(generator)]);
        }
        cv::Mat newF;
        getFundamentalMatrix(newMatches, newF);
        int inlierCount = 0;
        std::vector<double> x1Vec, y1Vec, x2Vec, y2Vec;

        // Get the mean and variance of the x and y coordinates
        for (int l = 0; l < matches.size(); l++) {

            double x1 = matches[l].pt1.x;
            double y1 = matches[l].pt1.y;

            double x2 = matches[l].pt2.x;
            double y2 = matches[l].pt2.y;

            x1Vec.push_back(x1);
            y1Vec.push_back(y1);
            x2Vec.push_back(x2);
            y2Vec.push_back(y2);

        }

        // These points are now normalized image coordinates with the optical center as reference
        // x1 = (x1 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        // x2 = (x2 -intrinsics.Left.getCx())/intrinsics.Left.getFx();
        // y1 =  (y1 -intrinsics.Left.getCy())/intrinsics.Left.getFy();
        // y2 =  (y2 -intrinsics.Left.getCy())/intrinsics.Left.getFy();


        auto meanVec1 = getMeanVar(x1Vec);
        auto meanVec2 = getMeanVar(y1Vec);
        auto meanVec3 = getMeanVar(x2Vec);
        auto meanVec4 = getMeanVar(y2Vec);
        for (int k = 0; k < matches.size(); k++) {

            float x1 = matches[k].pt1.x;
            float y1 = matches[k].pt1.y;

            float x2 = matches[k].pt2.x;
            float y2 = matches[k].pt2.y;


            float normX1 = (x1 - meanVec1.first)/(meanVec1.second + std::numeric_limits<float>::epsilon());
            float normY1 = (y1 - meanVec2.first)/(meanVec2.second + std::numeric_limits<float>::epsilon());
            float normX2 = (x2 - meanVec3.first)/(meanVec3.second + std::numeric_limits<float>::epsilon());
            float normY2 = (y2 - meanVec4.first)/(meanVec4.second + std::numeric_limits<float>::epsilon());

            cv::Mat p1 = (cv::Mat_<float>(3, 1) << normX1, normY1, 1);
            cv::Mat p2 = (cv::Mat_<float>(3, 1) << normX2, normY2, 1);


            cv::Mat error = (p2.t() * newF * p1);
            if (fabs(error.at<float>(0)) < threshold) {
                // std::cout << "error: " << fabs(error.at<float>(0))  << std::endl;
                inlierCount++;
            }
        }
        if (inlierCount > maxInliers) {
            maxInliers = inlierCount;
            std::cout << "Max inliers: " << maxInliers << std::endl;
            F = newF;
            meanVar1 = meanVec1;
            meanVar2 = meanVec2;
            meanVar3 = meanVec3;
            meanVar4 = meanVec4;

        }
    }

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