#include "../include/LoopHandler.hpp"

LoopHandler::LoopHandler() {

}

LoopHandler::LoopHandler(const std::string &config) : brief (256), fd(12, 50)  {
    std::ifstream ifs(config);
    Json::Reader reader;
    Json::Value value;
    reader.parse(ifs, value);
    std::string basePath = value["basePath"].asString();
    seqNo = value["sequence"].asString();
    std::string calibPath = basePath  + seqNo + "/calib.txt";
    handler3D.setCalibParams(calibPath);
    std::string camType = value["cameraType"].asString();
    if (camType == "mono") {
        isStereo = false;
        leftImagesPath = basePath + seqNo + "/image_0/";
        // rightImagesPath = basePath + seqNo + "/images_1/";
        rightImagesPath = "";
    } else {
        rightImagesPath = basePath + seqNo + "/image_1/";
    }
    generatePathTrain();
    _trainIterator = leftPathTrain.begin();


    map = Map::createMap();
    viz = std::make_shared<Viewer>();
    viz->setMap(map);
    
}



void LoopHandler::generatePathTrain() {
    std::vector<boost::filesystem::path> filesLeft;
    std::vector<boost::filesystem::path> filesRight;
    if (isStereo) {
        filesLeft = getFilesInFolder(leftImagesPath);
        for (auto &eachLeftPath : filesLeft) {
            leftPathTrain.push_back(eachLeftPath.string());
        }
        // Generate train for Right
        filesRight = getFilesInFolder(rightImagesPath);
        for (auto &eachRightPath : filesRight) {
            rightPathTrain.push_back(eachRightPath.string());
        }

    } else {
        filesLeft = getFilesInFolder(leftImagesPath);
        for (auto &eachLeftPath : filesLeft) {
            leftPathTrain.push_back(eachLeftPath.string());
        }
    }
}


std::string LoopHandler::getSeqNo() {
    return seqNo;
}

bool LoopHandler::stereoStatus() {
    if (this->isStereo) {
        return true;
    }
    return false;
}

std::string LoopHandler::getLeftImagesPath() {
    return leftImagesPath;
}

int LoopHandler::getLeftTrainLength() {
    return leftPathTrain.size();
}


void LoopHandler::addFrame(Frame::ptr _frame) {
    currentFrame = _frame;
    if (status == voStatus::INIT) {
        if (lastFrame) {
            bool success = buildInitMap();
            if (success) {
                status = voStatus::TRACKING;
                if(viz) {
                    viz->addCurrentFrame(currentFrame);
                    viz->updateMap();
                }
                std::cout << "Built initial map" << std::endl;
            }
          
        }
    }
    else if (status == voStatus::TRACKING) {
        // Map has already been built, now track the features
        trackLastFrame(); 
    }
    lastFrame = currentFrame;
}



/*
    LAST FRAME ->GET FEATURES -> GET MAP POINTS -> PROJECT TO CURRENT FRAME - 
    -> IF STATUS FOR FEATURE IS GOOD -> ADD AS A NEW FEATURE -> ADD CORRESPONDING PREV. MAP POINT TO THIS NEW FEATURE
    -> ADD THIS NEW FEATURE TO CURRENT FRAME -> KEEP COUNT OF NEW FEATURES ADDED
                    
*/


void LoopHandler::trackLastFrame() {

    // Initial guess for the new pose based on motion 
    if (lastFrame) {
        currentFrame->setPose(relativeMotion*lastFrame->pose);
    }

    for (auto &eachKp:lastFrame->features) {
        // convert the weak ptr to shared with lock 
        if (auto mapPtr = eachKp->mapPoint.lock() ) {
            auto mp = mapPtr->position;
            // project mp on the current pose
            auto currKp = currentFrame->world2Camera(mp, currentFrame->pose, handler3D.intrinsics.Left.getK());
            // check the points fall within the boundry of the image
            cv::Point2d newPts = cv::Point2d(currKp.coeff(0)/currKp.coeff(2), currKp.coeff(1)/currKp.coeff(2));
            // if (newPts.x > 0 && newPts.y > 0) {
            // }

        }
    }
    relativeMotion = currentFrame->pose * lastFrame->pose.inverse();
    map->insertKeyFrame(currentFrame);
    if(viz) {
        viz->addCurrentFrame(currentFrame);
        viz->updateMap();
    }
}



bool LoopHandler::takeVOStep() {
    // Get the next image 
    Frame::ptr frame = getNextFrame();
    if (frame != nullptr) {
        // Get features
        insertFrameFeatures(frame);
        // Add frame to pipeline
        addFrame(frame);
        return true;
    }

}



void LoopHandler::insertFrameFeatures(Frame::ptr _frame) {
    // Get FAST features and have keypoints acc. to harris responses
    auto features = fd.getFastFeatures(*_frame);
    // Compute BRIEF features
    brief.computeBrief(features, *_frame);
    // Add features to frame
    
}


void LoopHandler::runVO() {
    takeVOStep();
    takeVOStep();
    takeVOStep();
    takeVOStep();
    takeVOStep();
    takeVOStep();
    takeVOStep();
    takeVOStep();
    takeVOStep();


}

bool LoopHandler::buildInitMap() {

    std::vector<Matches> matches = brief.matchFeatures(*currentFrame, *lastFrame);
    std::vector<Matches> filterMatches;

    brief.removeOutliers(matches, filterMatches, 20.0);
    // adding as features only those points that have been matched and filtered out
    for (auto &eachMatch : filterMatches) {
        Feature::ptr feat1 = std::make_shared<Feature>(lastFrame, cv::Point2d(eachMatch.pt1.x, eachMatch.pt1.y));
        lastFrame->features.push_back(feat1);
        Feature::ptr feat2 = std::make_shared<Feature>(currentFrame, cv::Point2d(eachMatch.pt2.x, eachMatch.pt2.y));
        currentFrame->features.push_back(feat2);
    }
    cv::Mat F = cv::Mat::zeros(3, 3, CV_64F);
    handler3D.getFRANSAC(filterMatches, F, 200, 0.1);    
    // Essential matrix from fundamental matrix
    cv::Mat E = handler3D.intrinsics.Left.getK().t() * F * handler3D.intrinsics.Left.getK();
    // Get the pose of the second camera
    // Pose p = handler3D.disambiguateRT(E, filterMatches);

    cv::Mat R; cv::Mat t;
    CV2DPoints lastFramePts;
    CV2DPoints currFramePts;
    cv::Point2d principal_point(607.193, 185.216);  //相机光心, TUM dataset标定值
    cv::Point2d focal_length(718.856, 718.856);  //相机光心, TUM dataset标定值

    for (int i = 0; i < filterMatches.size(); i++) {
        lastFramePts.push_back(cv::Point(filterMatches[i].pt1.x, filterMatches[i].pt1.y));
        currFramePts.push_back(cv::Point(filterMatches[i].pt2.x, filterMatches[i].pt2.y));
    }

    cv::recoverPose(E, lastFramePts, currFramePts, handler3D.intrinsics.Left.getK(), R, t);


    Eigen::Matrix<double, 3, 3> REigen ;

    REigen <<   R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), 
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), 
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

    Eigen::Matrix<double, 3, 1>  tEigen ;

    tEigen << t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0);



    Sophus::SE3d currPose(REigen, tEigen);
    currentFrame->setPose(currPose);

    // insert current and last frame into the map
    map->insertKeyFrame(lastFrame);
    map->insertKeyFrame(currentFrame);

    //triangulate based on the last and currentFrame
    CV3DPoints new3dPoints = triangulate2View(lastFrame, currentFrame, filterMatches);

    // Points used in 3d triangulation = same as the ones matched after outlier removal

    for (int i=0;i<new3dPoints.size();i++) {
        MapPoint::ptr newPt = MapPoint::createMapPoint();
        // std::cout << new3dPoints[i].x << " " << new3dPoints[i].y << " " << new3dPoints[i].z << std::endl;
        newPt->setPos(new3dPoints[i]);
        // add features that map to the 3d point
        newPt->addObservation(lastFrame->features[i]);
        newPt->addObservation(currentFrame->features[i]);
        // insert 3d points that are part of the features in frame
        lastFrame->features[i]->mapPoint = newPt;
        currentFrame->features[i]->mapPoint = newPt;
        map->insertMapPoint(newPt);
    }

    relativeMotion = currentFrame->pose * lastFrame->pose.inverse();


    return true;
}


Sophus::SE3d LoopHandler::optimizePoseOnly(CV3DPoints pts3D, CV2DPoints camPts, cv::Mat K) {
    Sophus::SE3d _pose;

    optim.partialBA(pts3D, camPts, K, _pose);

    return _pose;
}






CV3DPoints LoopHandler::triangulate2View(Frame::ptr lastFrame, Frame::ptr currFrame, 
                                                    const std::vector<Matches> filtMatches) {
    
    CV2DPoints lastFramePts;
    CV2DPoints currFramePts;
   
    for (int i = 0; i < filtMatches.size(); i++) {
        lastFramePts.push_back(cv::Point(filtMatches[i].pt1.x, filtMatches[i].pt1.y));
        currFramePts.push_back(cv::Point(filtMatches[i].pt2.x, filtMatches[i].pt2.y));
    }

    // Placeholder for triangulated points
    cv::Mat pnts3D(4, lastFramePts.size(), CV_64F);
   
    cv::Mat P0 = sophus2ProjMat(lastFrame);
    cv::Mat P1 = sophus2ProjMat(currFrame);

    cv::triangulatePoints(P0, P1, lastFramePts, currFramePts, pnts3D);

    CV3DPoints pts3D;
    for (int i = 0; i < pnts3D.cols; i++) {
        cv::Point3d pt3d(pnts3D.at<double>(0, i)/pnts3D.at<double>(3, i), pnts3D.at<double>(1, i)/pnts3D.at<double>(3, i), pnts3D.at<double>(2, i)/pnts3D.at<double>(3, i));
        pts3D.push_back(pt3d);
    }

    return pts3D;


}


cv::Mat LoopHandler::sophus2ProjMat( Frame::ptr _frame) {

    Sophus::SE3d pose = _frame->getPose();

    Eigen::Matrix3d rot = pose.rotationMatrix();
    Vec3 trans = pose.translation();
    cv::Mat P0 = (cv::Mat_<double>(3, 4) << 1, 0, 0, trans.coeff(0),
                                            0, 1, 0, trans.coeff(1),
                                            0, 0, 1, trans.coeff(2));
    P0.at<double>(0,0) = rot.coeff(0,0); P0.at<double>(0,1) = rot.coeff(0,1); P0.at<double>(0,2) = rot.coeff(0,2);
    P0.at<double>(1,0) = rot.coeff(1,0); P0.at<double>(1,1) = rot.coeff(1,1); P0.at<double>(1,2) = rot.coeff(1,2);
    P0.at<double>(2,0) = rot.coeff(2,0); P0.at<double>(2,1) = rot.coeff(2,1); P0.at<double>(2,2) = rot.coeff(2,2);
    
    P0 = handler3D.intrinsics.Left.getK()*P0;
    
    // std::cout << "Proj matrix" << P0 << std::endl;

    return P0;

}

Frame::ptr LoopHandler::getNextFrame() {
    if (_trainIterator != leftPathTrain.end()) {
        cv::Mat img = cv::imread(*_trainIterator, 0);
        currentFrameId = _trainIterator - leftPathTrain.begin();
        _trainIterator++;
        // ADD FRAME ID TO FRAME!!!!!!!!!!!!!-----------------
        Frame::ptr frame = std::make_shared<Frame>(img, Frame::createFrameID());    
        return frame;
    }
    return nullptr;
}





LoopHandler::~LoopHandler() {}