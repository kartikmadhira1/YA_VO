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
        bool trackSuccess = track();
        // bool reinitSuccess = reinitialize();
        if (!trackSuccess) {
            bool reinitSuccess = reinitialize();
            if(viz) {
                viz->addCurrentFrame(currentFrame);
                viz->updateMap();
            }
            std::cout << "RESET successful" << std::endl;
        }
    }
    else if (status == voStatus::RESET) {
        bool reinitSuccess = reinitialize();
        if (reinitSuccess) {
            status = voStatus::TRACKING;
            if(viz) {
                viz->addCurrentFrame(currentFrame);
                viz->updateMap();
            }
            std::cout << "RESET successful" << std::endl;
        }
    }
    lastFrame = currentFrame;
}



bool LoopHandler::reinitialize() {

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
    currPose = currPose * lastFrame->pose;
    currentFrame->setPose(currPose);

    // insert current and last frame into the map
    // map->insertKeyFrame(lastFrame);
    map->insertKeyFrame(currentFrame);

    //triangulate based on the last and currentFrame
    CV3DPoints new3dPoints = triangulate2View(lastFrame, currentFrame, filterMatches);

    // Points used in 3d triangulation = same as the ones matched after outlier removal
    std::cout << new3dPoints.size() << "<---- number of points triangulated!" << std::endl;
    if (new3dPoints.size() < 5) {
        return false;
    }
    // now empty the active L and F frames
    map->resetActive();


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





/*
    LAST FRAME ->GET FEATURES -> GET MAP POINTS -> PROJECT TO CURRENT FRAME -> GET OPTICAL FLOW BETWEEN PREVIOUS AND CURRENT FRAME
    -> IF STATUS FOR FEATURE IS GOOD -> ADD AS A NEW FEATURE -> ADD CORRESPONDING PREV. MAP POINT TO THIS NEW FEATURE
    -> ADD THIS NEW FEATURE TO CURRENT FRAME -> KEEP COUNT OF NEW FEATURES ADDED
                    
*/

bool LoopHandler::track() {

    // Initial guess for the new pose based on motion 
    if (lastFrame) {
        currentFrame->setPose(relativeMotion*lastFrame->pose);
    }

    int goodInliers = trackLastFrame();

    int optimizedInliers = optimizePoseOnly();

    if (optimizedInliers < 9) {
        status = voStatus::RESET;
        std::cout << "tracking failed" << std::endl;
        return false;
    }

    relativeMotion = currentFrame->pose * lastFrame->pose.inverse();
    map->insertKeyFrame(currentFrame);

    if(viz) {
        viz->addCurrentFrame(currentFrame);
        viz->updateMap();
    }

    return true;
}



int LoopHandler::trackLastFrame() {

  
    // prepare points for optical flow check
    std::vector<cv::Point2f> lastFrameKpt;
    std::vector<cv::Point2f> currFrameKpt;
    for (auto &eachKp:lastFrame->features) {
        // convert the weak ptr to shared with lock 
        if (auto mapPtr = eachKp->mapPoint.lock() ) {
            auto mp = mapPtr->position;
            // project mp on the current pose
            auto currKp = currentFrame->world2Camera(mp, currentFrame->pose, handler3D.intrinsics.Left.getK());
            // check the points fall within the boundry of the image
            cv::Point2d newPts = cv::Point2d(currKp.coeff(0)/currKp.coeff(2), currKp.coeff(1)/currKp.coeff(2));
            lastFrameKpt.push_back(newPts);
            currFrameKpt.push_back(eachKp->kp);

        }
    }
    // calculate the optical flow between last and current frame 
    std::vector<uchar> status;
    cv::Mat error;
    std::cout << type2str(lastFrame->rawImage.type()) << std::endl;
    std::cout << type2str(currentFrame->rawImage.type()) << std::endl;

    cv::calcOpticalFlowPyrLK(lastFrame->rawImage, currentFrame->rawImage, lastFrameKpt, currFrameKpt,
                                status, error, cv::Size(50, 50), 3,
                                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01),
                                cv::OPTFLOW_USE_INITIAL_FLOW);
    

    int goodFeatures = 0;
    // add features that follow the optical flow

    for (int i=0; i<status.size();i++) {
        if(status[i]) {
            std::cout << "status good ! " << i << std::endl;
            Feature::ptr feat = std::make_shared<Feature>(currentFrame, cv::Point2d(currFrameKpt[i].x, currFrameKpt[i].y));
            // add feat to corresponding mappoint
            feat->mapPoint = lastFrame->features[i]->mapPoint;
            // add feature to the mapPoint
            currentFrame->features.push_back(feat);
            goodFeatures++;            
        }
    }


   
    return goodFeatures;

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
    // time these twi things
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    auto features = fd.getFastFeatures(*_frame);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> timeUsed1 = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Feat extraction cost time: " << timeUsed1.count() << " seconds." << endl;

    // Compute BRIEF features
    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    brief.computeBrief(features, *_frame);
    chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    chrono::duration<double> timeUsed2 = chrono::duration_cast<chrono::duration<double>>(t4 - t3);
    cout << "Descriptor generation cost time: " << timeUsed2.count() << " seconds." << endl;
    // Add features to frame
    
}


void LoopHandler::runVO() {
    

    while(1) {
        if (takeVOStep() == false) {
            break;
        }        
    }


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


int LoopHandler::optimizePoseOnly() {
    
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // Construct optimzation algorithm
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // Add camera vertex
    VertexPose *vertexPose = new VertexPose();
    vertexPose->setId(0);
    vertexPose->setEstimate(currentFrame->pose);
    optimizer.addVertex(vertexPose);

    // Camera parameters Eigen
    Eigen::Matrix3d KEigen;
    cv::Mat _K = handler3D.intrinsics.Left.getK();

    KEigen << _K.at<double>(0, 0), _K.at<double>(0, 1), _K.at<double>(0, 2),
              _K.at<double>(1, 0), _K.at<double>(1, 1), _K.at<double>(1, 2),
              _K.at<double>(2, 0), _K.at<double>(2, 1), _K.at<double>(2, 2);

    // // Convert 3d and 2d points in the form of Eigen
    // VecVector2d points2Deigen;
    // VecVector3d points3Deigen;
    // convertToEigen(points3D, points2D, points2Deigen, points3Deigen);
    std::vector<EdgeProjection*> edges;
    std::vector<Feature::ptr> features;
    int index=1;
    for (auto &eachKp:currentFrame->features) {
        if (auto mp = eachKp->mapPoint.lock()) {

            features.push_back(eachKp);
            Eigen::Vector3d point3d;
            Eigen::Vector2d point2d;
            Vec3 pos = mp->getPos();
            point3d << pos.coeff(0), pos.coeff(1), pos.coeff(2);
            point2d << eachKp->kp.y, eachKp->kp.x;
            std::cout << "-----3D point" << point3d << std::endl;
            std::cout << "-----2D point" << point2d << std::endl;
            EdgeProjection *edge = new EdgeProjection(point3d, KEigen);
            edge->setId(index);
            edge->setVertex(0, vertexPose);
            edge->setMeasurement(point2d);
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }


    std::cout << "initial optizimation done!" << std::endl;
    // estimate the Pose the determine the outliers
    const double chi2th = 5.991;
    int outlierCount = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertexPose->setEstimate(currentFrame->pose);
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        outlierCount = 0;

        // // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->isOutlier) {
                e->computeError();
            }
            if (e->chi2() > chi2th) {
                features[i]->isOutlier = true;
                e->setLevel(1);
                outlierCount++;
            } else {
                features[i]->isOutlier = false;
                e->setLevel(0);
            }

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }
    std::cout << "outlier removal done!" << std::endl;

    currentFrame->setPose(vertexPose->estimate());

    std::cout << "Current pose " << currentFrame->pose.matrix();
    std::cout << "Outlier/Inlier count" << outlierCount << "/" << features.size()-outlierCount << std::endl;

    // remove all map points that are outliers

    for (auto &eachFeat : features) {
        if (eachFeat->isOutlier) {
            eachFeat->mapPoint.reset();
            eachFeat->isOutlier = false;
        }
    }

    int inliers = features.size() - outlierCount;

    return inliers;

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