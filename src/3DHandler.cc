#include "../include/3DHandler.hpp"




_3DHandler::_3DHandler(std::string calibPath) {
    getCalibParams(calibPath, intrinsics);
}



double _3DHandler::getMeanVar(std::vector<double> &vec) {
    double mean = 0.0;
    double var = 0.0;
    for (double &each : vec) {
        mean += each;
    }
    mean /= vec.size();
    return mean;
}


cv::Mat _3DHandler::constructNormMatrix(std::vector<double> xVec, std::vector<double> yVec, 
                                                double xMean, double yMean) {

    cv::Mat normMat = cv::Mat::eye(3, 3, CV_64F);
    double scaleDenom = 0.0;
    for (int i = 0; i < xVec.size(); i++) {
        double xHat = xVec[i] - xMean;
        double yHat = yVec[i] - yMean;
        scaleDenom += sqrt(xHat*xHat + yHat*yHat);
    }

    double scale = sqrt(2.0) / (scaleDenom / xVec.size());

    normMat.at<double>(0, 0) = scale;
    normMat.at<double>(1, 1) = scale;
    normMat.at<double>(0, 2) = -scale * xMean;
    normMat.at<double>(1, 2) = -scale * yMean;

    return normMat;
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
    

    // Get the mean of all x and y points

    std::vector<double> x1Vec, y1Vec, x2Vec, y2Vec;

    for (auto &each : matches) {
        x1Vec.push_back(each.pt1.x);
        y1Vec.push_back(each.pt1.y);
        x2Vec.push_back(each.pt2.x);
        y2Vec.push_back(each.pt2.y);
    }

    double x1Mean = getMeanVar(x1Vec);
    double y1Mean = getMeanVar(y1Vec);
    double x2Mean = getMeanVar(x2Vec);
    double y2Mean = getMeanVar(y2Vec);

    // Construct normalization matrix for each ui, vi for both images

    cv::Mat normImage1 = constructNormMatrix(x1Vec, y1Vec, x1Mean, y1Mean);
    cv::Mat normImage2 = constructNormMatrix(x2Vec, y2Vec, x2Mean, y2Mean);

    cv::Mat solveMat = cv::Mat::zeros(matches.size(), 9, CV_64F);

    for (int i = 0; i < matches.size(); i++) {

        double x1 = matches[i].pt1.x;
        double y1 = matches[i].pt1.y;

        double x2 = matches[i].pt2.x;
        double y2 = matches[i].pt2.y;
      
        // Normalize the points
        cv::Mat p1 = (cv::Mat_<double>(3, 1) << x1, y1, 1);
        cv::Mat p2 = (cv::Mat_<double>(3, 1) << x2, y2, 1);

        cv::Mat normP1 = normImage1 * p1;
        cv::Mat normP2 = normImage2 * p2;

        // Need inhomogenous coordinates
        double normX1 = normP1.at<double>(0, 0);
        double normY1 = normP1.at<double>(1, 0);
        double normX2 = normP2.at<double>(0, 0);
        double normY2 = normP2.at<double>(1, 0);

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
    cv::SVD fullSolveSVD(solveMat.t()*solveMat, cv::SVD::FULL_UV); 

    // F solution is the right singular vector corresponding to the smallest singular value
    F = fullSolveSVD.vt.row(8).reshape(1, 3);

    // SVD again on F to get the closest rank 2 matrix
    cv::SVD FSolveSVD(F, cv::SVD::FULL_UV);

    // Enforce rank 2    
    FSolveSVD.w.at<double>(2) = 0;

    cv::Mat w = cv::Mat::diag(FSolveSVD.w);
    F = FSolveSVD.u * w * FSolveSVD.vt;

    // Denormalize F
    F = normImage2.t() * F * normImage1;

    F = F/F.at<double>(2,2);
    
    return true;

}


bool _3DHandler::getFRANSAC(std::vector<Matches> matches, cv::Mat &F, int iterations=300, double threshold=0.1) {   

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
        for (int k = 0; k < matches.size(); k++) {

            double x1 = matches[k].pt1.x;
            double y1 = matches[k].pt1.y;

            double x2 = matches[k].pt2.x;
            double y2 = matches[k].pt2.y;

            cv::Mat p1 = (cv::Mat_<double>(3, 1) << x1, y1, 1);
            cv::Mat p2 = (cv::Mat_<double>(3, 1) << x2, y2, 1);


            cv::Mat error = (p2.t() * newF * p1);
            if (fabs(error.at<double>(0)) < threshold) {
                // std::cout << "error: " << fabs(error.at<double>(0))  << std::endl;
                inlierCount++;
            }
        }
        if (inlierCount > maxInliers) {
            maxInliers = inlierCount;
            newF.copyTo(F);
        }
    }

    return true;
}



Pose _3DHandler::disambiguateRT(const cv::Mat &E, std::vector<Matches> &matches) {

    // Four possible solutions are possible for the essential matrix

    // 1. R = U * r(90).t()* Vt
    //    T = U * r(90)* w * Vt
    // 2. R = U * r(90).t()* Vt
    //    T = -U * r(90)* w * Vt
    // 3. R = U * r(-90).t()* Vt
    //    T = U * r(90)* w * Vt
    // 4. R = U * Wt * Vt
    //    T = -U * r(90)* w * Vt
    cv::SVD ESolveSVD(E, cv::SVD::FULL_UV);
    
    cv::Mat u = ESolveSVD.u;
    cv::Mat vt = ESolveSVD.vt;
    cv::Mat w = ESolveSVD.w;
    
    cv::Mat r90 = rotateMatrixZ(90);
    cv::Mat negr90 = rotateMatrixZ(-90);

    //Prepate points to triangulate
    std::vector<cv::Point2d> cam0Pnts;
    std::vector<cv::Point2d> cam1Pnts;
    // Get all points from matches 
    for (int i = 0; i < matches.size(); i++) {
        cam0Pnts.push_back(cv::Point(matches[i].pt1.x, matches[i].pt1.y));
        cam1Pnts.push_back(cv::Point(matches[i].pt2.x, matches[i].pt2.y));
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

    if (checkDepthPositive(pnts3D, R1, t, pose1)) {
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
    if (checkDepthPositive(pnts3D, R2, t2,  pose2)) {
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
    if (checkDepthPositive(pnts3D, R3, t3, pose3)) {
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
    if (checkDepthPositive(pnts3D, R4, t4, pose4)) {
        poseTrain.push_back(pose4);
    }
    

    int maxInliers = 0;
    Pose bestPose;
    for (Pose &eachPose : poseTrain) {
        if (eachPose.numChierality > maxInliers) {
            maxInliers = eachPose.numChierality;
            std::cout << "Max inliers: " << maxInliers << std::endl;
            bestPose = eachPose;
        }
    }
    return bestPose;
}



bool _3DHandler::checkDepthPositive(cv::Mat &pnts3D, cv::Mat R, cv::Mat t, Pose &pose) {
    // Check if the depth of the points are positive
    for (int i = 0; i < pnts3D.cols; i++) {

        cv::Mat r3 = R.row(2);

        pnts3D.at<double>(0, i) = pnts3D.at<double>(0, i)/pnts3D.at<double>(3, i);
        pnts3D.at<double>(1, i) = pnts3D.at<double>(1, i)/pnts3D.at<double>(3, i);
        pnts3D.at<double>(2, i) = pnts3D.at<double>(2, i)/pnts3D.at<double>(3, i);

        cv::Mat X = (cv::Mat_<double>(3, 1) << pnts3D.at<double>(0, i), pnts3D.at<double>(1, i), pnts3D.at<double>(2, i));


        cv::Mat p3 = r3*(X - t);
        if ( p3.at<double>(0) < 0) {
            continue;
        } else {
           
            pose.numChierality++;
        }
    }
    if (pose.numChierality == 0) {
        return false;
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