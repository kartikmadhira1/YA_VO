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
    gftt_ = cv::ORB::create(600);

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
            map->insertKeyFrame(currentFrame);
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


/*
    LAST FRAME ->GET FEATURES -> GET MAP POINTS -> PROJECT TO CURRENT FRAME -> GET OPTICAL FLOW BETWEEN PREVIOUS AND CURRENT FRAME
    -> IF STATUS FOR FEATURE IS GOOD -> ADD AS A NEW FEATURE -> ADD CORRESPONDING PREV. MAP POINT TO THIS NEW FEATURE
    -> ADD THIS NEW FEATURE TO CURRENT FRAME -> KEEP COUNT OF NEW FEATURES ADDED
                    
*/

bool LoopHandler::track() {

    // Initial guess for the new pose based on motion 
    // Is this the right way? check if relative motion has already been set? 
    // yes, checked relative motion from previous initial build
    std::cout << "Tracking started" << std::endl;
    if (lastFrame) {
        currentFrame->setPose(relativeMotion*lastFrame->pose);
    }

    int goodInliers = trackLastFrame();

    if (goodInliers < 2) {
        return false;
    }
    int optimizedInliers = optimizePoseOnly();

    // map->insertKeyFrame(currentFrame);
    
    if (optimizedInliers < 100) {
        std::cout << "tracking failed" << std::endl;
        return false;
    }

    relativeMotion = currentFrame->pose * lastFrame->pose.inverse();
    // map->insertKeyFrame(currentFrame);

    if(viz) {
        viz->addCurrentFrame(currentFrame);
        viz->updateMap();
    }

    return true;
}


bool LoopHandler::reinitialize() {

    
   
    // //Filter out the 3d points that were reset in the last frame
    // int resetPoints = 0;
    // std::vector<cv::Point> resetKeypoints;
    // for (int i=0;i<lastFrame->features.size();i++) {
    //     // if (lastFrame->features[i]->mapPoint.expired()) {
    //         // add these features to reset features
    //          resetKeypoints.push_back(cv::Point(lastFrame->features[i]->kp.x, lastFrame->features[i]->kp.y));
    //     // }
    // }
    // // put a mock image 
    // Image mockImage(lastFrame->rawImage);

    // brief.computeBrief(resetKeypoints, mockImage);

    // reset current frame features
    currentFrame->features.clear();

    std::vector<Matches> matches = brief.matchFeatures(*lastFrame, *currentFrame);
    std::vector<Matches> filterMatches;

    brief.removeOutliers(matches, filterMatches, 20.0);



    Image testObj1(lastFrame->rawImage);
    Image testObj2(currentFrame->rawImage);

 // std::cout << filterMatches.size() << std::endl;
    cv::Mat testImage1, testImage2;

    lastFrame->rawImage.copyTo(testImage1);
    currentFrame->rawImage.copyTo(testImage2);

    cv::Mat sideBySide = brief.drawMatches(testObj1, testObj2, filterMatches);

    // cv::imwrite("image1.png", testImage1);
    // cv::imwrite("image2.png", testImage2);


    // cv::imwrite("testSideBySideRESET.png", sideBySide);



    // // // adding as features only those points that have been matched and filtered out
    // for (auto &eachMatch : filterMatches) {
    //     // Feature::ptr feat1 = std::make_shared<Feature>(lastFrame, cv::Point2d(eachMatch.pt1.x, eachMatch.pt1.y));
    //     // lastFrame->features.push_back(feat1);
    //     Feature::ptr feat2 = std::make_shared<Feature>(currentFrame, cv::Point2d(eachMatch.pt2.x, eachMatch.pt2.y));
    //     currentFrame->features.push_back(feat2);
    // }

    std::cout << "Matched points: " << filterMatches.size() << std::endl;
    cv::Mat F = cv::Mat::zeros(3, 3, CV_64F);
    handler3D.getFRANSAC(filterMatches, F, 400, 0.1);    
    // Essential matrix from fundamental matrix

    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);

    cv::Mat E, mask;
    std::vector<cv::Point2f> prevFeatures, currFeatures;

    for (auto& eachPt : filterMatches) {
        prevFeatures.push_back(cv::Point2f(eachPt.pt1.x,eachPt.pt1.y ));
        currFeatures.push_back(cv::Point2f(eachPt.pt2.x,eachPt.pt2.y ));
    }

  	E = cv::findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

// 
    // cv::Mat E = handler3D.intrinsics.Left.getK().t() * F * handler3D.intrinsics.Left.getK();
    // Get the pose of the second camera
    // Pose p = handler3D.disambiguateRT(E, filterMatches);
    // E = E.inv();
    cv::Mat R; cv::Mat t;
    CV2DPoints lastFramePts;
    CV2DPoints currFramePts;
   

    for (int i = 0; i < filterMatches.size(); i++) {
        lastFramePts.push_back(cv::Point(filterMatches[i].pt1.x, filterMatches[i].pt1.y));
        currFramePts.push_back(cv::Point(filterMatches[i].pt2.x, filterMatches[i].pt2.y));
    }

    cv::recoverPose(E, currFramePts, lastFramePts, handler3D.intrinsics.Left.getK(), R, t);


    Eigen::Matrix<double, 3, 3> REigen ;

    REigen <<   R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), 
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), 
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

    Eigen::Matrix<double, 3, 1>  tEigen ;

    tEigen << t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0);

    // if (t.at<double>(2,0) < 0) {
    //     std::cout << "Failed to initialize" << std::endl;
    //     std::cout << "INITIAL T: " <<  tEigen << std::endl;
    //     return false;
        
    // }
    std::cout << "NEW T: " <<  tEigen << std::endl;

    Sophus::SE3d currPose(REigen,tEigen);
    currPose = currPose.inverse();
    currentFrame->setPose(currPose*lastFrame->pose);


    std::cout << "Number of matches after resetting: " << filterMatches.size() << std::endl;
    // print current frame ID
    std::cout << "Current frame ID: RESETTT " << currentFrame->frameID << std::endl;
    std::cout << "Current frame features size " << currentFrame->features.size() << std::endl;


    auto ret = triangulate2View(lastFrame, currentFrame, filterMatches, false);
    std::cout << "Current frame features size after triangulation " << currentFrame->features.size() << std::endl;

   
    relativeMotion = currentFrame->pose * lastFrame->pose.inverse();

    return true;

}









int LoopHandler::trackLastFrame() {

    // cv::Mat currImgcopy1, currColr1;
    // cv::Mat currImgcopy2, currColr2;

    // lastFrame->rawImage.copyTo(currImgcopy1);
    // currentFrame->rawImage.copyTo(currImgcopy2);

    // cv::cvtColor(currImgcopy1, currColr1, cv::COLOR_GRAY2RGB);
    // cv::cvtColor(currImgcopy2, currColr2, cv::COLOR_GRAY2RGB);

    // prepare points for optical flow check
    std::vector<cv::Point2f> lastFrameKpt;
    std::vector<cv::Point2f> currFrameKpt;
    int _i = 0;
    std::vector<int> lastFramePointsIndex;
    std::vector<cv::Point3d> pointsInFirst;
    std::vector<cv::Point3d> pointsInSecond;
    std::cout << "Last frame id: " << lastFrame->frameID << std::endl;
    std::cout << "Last features size " << lastFrame->features.size() << std::endl;

    for (int i = 0;i< lastFrame->features.size();i++) {
        // convert the weak ptr to shared with lock 
        if (auto mapPtr = lastFrame->features[i]->mapPoint.lock() ) {
            auto mp = mapPtr->position;
            // project mp on the current pose
            // take a step back and check if the current world points actually map to same image coordinates
            // step 1 - write all points to image 1, 
            // step 2 - triangulate points with image 2
            // step 3 - world2camera method to project all triangulated points to image 1.
            // step 4 - check diff between both the image.
            auto currKp = currentFrame->world2Camera(mp, currentFrame->pose, handler3D.intrinsics.Left.getK());
            // check the points fall within the boundry of the image
            // std::cout << currKp.coeff(0)/currKp.coeff(2) << " " << currKp.coeff(1)/currKp.coeff(2) << std::endl;



                // Checking in here with the x and y swap because it's getting inputted into opencv function
            cv::Point2i newPts = cv::Point2i(currKp.coeff(1)/currKp.coeff(2), currKp.coeff(0)/currKp.coeff(2));
            
            if (!(newPts.x < 0 || newPts.y < 0)) {
                currFrameKpt.push_back(newPts);
                lastFrameKpt.push_back(cv::Point2i(lastFrame->features[i]->kp.y, lastFrame->features[i]->kp.x));
                lastFramePointsIndex.push_back(i);
                // std::cout << "points in first*****" << newPts << std::endl;
                pointsInFirst.push_back(cv::Point3i(mp.coeff(0), mp.coeff(1), mp.coeff(2)));
            }

            // cv::circle(currColr2, cv::Point2i(newPts.y, newPts.x), 5, cv::Scalar(0,255,0), -1);
            // cv::circle(currColr1, cv::Point2i(eachKp->kp.y, eachKp->kp.x), 5, cv::Scalar(255,0,0), -1);
            // cv::imwrite("last" + std::to_string(currentFrame->frameID) + std::to_string(_i) + ".png", currColr1);
            // cv::imwrite("curr" + std::to_string(currentFrame->frameID) + std::to_string(_i) + ".png", currColr2);

            _i++;
        }
    }
    // print currframe pose

    // std::cout << "current pose in first**********"  <<currentFrame->pose.matrix() << std::endl;

    // calculate the optical flow between last and current frame 
    std::vector<uchar> flowStatus;
    cv::Mat error;
    cv::Mat mask = cv::Mat::zeros(lastFrame->rawImage.size(), lastFrame->rawImage.type());

    try {
        cv::calcOpticalFlowPyrLK(lastFrame->rawImage, currentFrame->rawImage, lastFrameKpt, currFrameKpt,
                                    flowStatus, error, cv::Size(11, 11), 3 ,
                                    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01),
                                    0,0.001);
    } catch (cv::Exception &e) {
        std::cout << "Caught tracking error, restarting";
        return false;
    }

    int goodFeatures = 0;
    // add features that follow the optical flow
    cv::Mat currImgcopy1, currColr1;
    cv::Mat currImgcopy2, currColr2;

    currentFrame->rawImage.copyTo(currImgcopy1);
    currentFrame->rawImage.copyTo(currImgcopy2);

    cv::cvtColor(currImgcopy1, currColr1, cv::COLOR_GRAY2RGB);
    cv::cvtColor(currImgcopy2, currColr2, cv::COLOR_GRAY2RGB);



    std::cout << "Size of flow status " << flowStatus.size() << std::endl;
    std::cout << "Size of last frame points " << lastFramePointsIndex.size() << std::endl;
    int _j=0;
    for (int i=0; i<flowStatus.size();i++) {
        if(flowStatus.at(i)==1) {
            // this is wrong//
              if (auto mapPtr = lastFrame->features[lastFramePointsIndex[i]]->mapPoint.lock()) {


                    Feature::ptr feat = std::make_shared<Feature>(currentFrame, cv::Point2i(currFrameKpt[i].y, currFrameKpt[i].x));
                
                    // feat->mapPoint = mapPtr;
                    // add feature to the mapPoint
                    currentFrame->features.push_back(feat);
                    
                    currentFrame->features[_j]->mapPoint = lastFrame->features[lastFramePointsIndex[i]]->mapPoint.lock();
                
                    // std::cout << mapPtr->getPos() << std::endl;
                    // cv::line(mask,cv::Point2i(currFrameKpt[i].y,currFrameKpt[i].x) , cv::Point2i(lastFrameKpt[i].y,lastFrameKpt[i].x) , cv::Scalar(0,255,0), 2);
                    auto currKp = currentFrame->world2Camera(mapPtr->getPos(), currentFrame->pose, handler3D.intrinsics.Left.getK());
                    
                    // print points in 
                    // std::cout << "POints in first *****" <<mapPtr->getPos() << std::endl;                    
                    // std::cout << "POints in second *****" << pointsInFirst[i] << std::endl;                    

                    // check the points fall within the boundry of the image
                    // std::cout << currKp.coeff(0)/currKp.coeff(2) << " " << currKp.coeff(1)/currKp.coeff(2) << std::endl;
                        // Checking in here with the x and y swap because it's getting inputted into opencv function
                    cv::Point2i newPts = cv::Point2d(currKp.coeff(0)/currKp.coeff(2), currKp.coeff(1)/currKp.coeff(2));
                    // std::cout << "points in second***** "  <<newPts.x << " " << newPts.y << std::endl;
                    cv::circle(currColr1, cv::Point2i(currFrameKpt[i].x,currFrameKpt[i].y), 5, cv::Scalar(0,255,0), -1);
                    cv::circle(currColr1, cv::Point2i(lastFrameKpt[i].x,lastFrameKpt[i].y), 5, cv::Scalar(255,0,0), -1);
                    cv::circle(currColr1, cv::Point2i(newPts.y, newPts.x), 5, cv::Scalar(0,0,255), -1);

                    // cv::imwrite("curr_frame" + std::to_string(currentFrame->frameID) + std::to_string(i) + ".png", currColr1);
                    // cv::imwrite("prevFrame" + std::to_string(currentFrame->frameID) + std::to_string(i) + ".png", currColr2);
                    goodFeatures++;
                    _j++;
            }      
        }
    }
    std::cout << "current frame size in TRACK^^^^^^^" << currentFrame->features.size() << std::endl;
    std::cout << "TRACK 2NS s^^^^^^^" << currentFrame->features[2]->mapPoint.lock()->getPos() << std::endl;


    cv::Mat copyImg;
    // cv::add(currColr, mask, copyImg);
    // cv::imwrite("curr_frame" + std::to_string(currentFrame->frameID) + ".png", currColr1);
    // cv::imwrite("prevFrame" + std::to_string(currentFrame->frameID) + ".png", currColr2);

    std::cout << "Amount of good features is: " << goodFeatures << std::endl;
    std::cout << "Amount of total features is: " << flowStatus.size() << std::endl;

    return goodFeatures;

}



bool LoopHandler::takeVOStep() {
    // Get the next image 
    Frame::ptr frame = getNextFrame();
    if (frame != nullptr) {
        // Get features
        insertFrameFeaturesOPENCV(frame);
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


void LoopHandler::insertFrameFeaturesOPENCV(Frame::ptr _frame) {
    cv::Mat mask(_frame->rawImage.size(), CV_8UC1, 255);
   
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(_frame->rawImage, keypoints);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> timeUsed1 = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Feat extraction cost time: " << timeUsed1.count() << " seconds." << endl;
    int cnt_detected = 0;
    std::vector<cv::Point> vec;
    for (auto &kp : keypoints) {
        
        vec.push_back(cv::Point(kp.pt.y, kp.pt.x));
    }
    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    brief.computeBrief(vec, *_frame);
    chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    chrono::duration<double> timeUsed2 = chrono::duration_cast<chrono::duration<double>>(t4 - t3);
    cout << "Descriptor generation cost time: " << timeUsed2.count() << " seconds." << endl;

}






void LoopHandler::runVO() {
    

    while(1) {
        if (takeVOStep() == false) {
            break;
        }
        // std::this_thread::sleep_for(std::chrono::nanoseconds(10));
        // std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::seconds(2));

    }


}

bool LoopHandler::buildInitMap() {

    std::vector<Matches> matches = brief.matchFeatures(*lastFrame, *currentFrame);
    std::vector<Matches> filterMatches;

    brief.removeOutliers(matches, filterMatches, 20.0);


    Image testObj1(lastFrame->rawImage);
    Image testObj2(currentFrame->rawImage);

 // std::cout << filterMatches.size() << std::endl;
    cv::Mat testImage1, testImage2;

    lastFrame->rawImage.copyTo(testImage1);
    currentFrame->rawImage.copyTo(testImage2);

    cv::Mat sideBySide = brief.drawMatches(testObj1, testObj2, filterMatches);

    // cv::imwrite("image1.png", testImage1);
    // cv::imwrite("image2.png", testImage2);


    // cv::imwrite("testSideBySide.png", sideBySide);


    // adding as features only those points that have been matched and filtered out
    for (auto eachMatch : filterMatches) {
        Feature::ptr feat1 = std::make_shared<Feature>(lastFrame, cv::Point2d(eachMatch.pt1.x, eachMatch.pt1.y));
        lastFrame->features.push_back(feat1);
        Feature::ptr feat2 = std::make_shared<Feature>(currentFrame, cv::Point2d(eachMatch.pt2.x, eachMatch.pt2.y));
        currentFrame->features.push_back(feat2);
    }
    std::cout << "Matched points: " << filterMatches.size() << std::endl;
    cv::Mat F = cv::Mat::zeros(3, 3, CV_64F);
    handler3D.getFRANSAC(filterMatches, F, 400, 0.1);    
    // Essential matrix from fundamental matrix

    double focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);

    cv::Mat E, mask;
    std::vector<cv::Point2f> prevFeatures, currFeatures;

    for (auto& eachPt : filterMatches) {
        prevFeatures.push_back(cv::Point2f(eachPt.pt1.x,eachPt.pt1.y ));
        currFeatures.push_back(cv::Point2f(eachPt.pt2.x,eachPt.pt2.y ));
    }

  	E = cv::findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

// 
    // cv::Mat E = handler3D.intrinsics.Left.getK().t() * F * handler3D.intrinsics.Left.getK();
    // Get the pose of the second camera
    // Pose p = handler3D.disambiguateRT(E, filterMatches);
    // E = E.inv();
    cv::Mat R; cv::Mat t;
    CV2DPoints lastFramePts;
    CV2DPoints currFramePts;
   

    for (int i = 0; i < filterMatches.size(); i++) {
        lastFramePts.push_back(cv::Point(filterMatches[i].pt1.x, filterMatches[i].pt1.y));
        currFramePts.push_back(cv::Point(filterMatches[i].pt2.x, filterMatches[i].pt2.y));
    }

    cv::recoverPose(E, currFramePts,lastFramePts,  handler3D.intrinsics.Left.getK(), R, t);


    Eigen::Matrix<double, 3, 3> REigen ;

    REigen <<   R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), 
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), 
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

    Eigen::Matrix<double, 3, 1>  tEigen ;

    tEigen << t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0);

    // if (t.at<double>(2,0) < 0) {
    //     std::cout << "Failed to initialize" << std::endl;
    //     std::cout << "INITIAL T: " <<  tEigen << std::endl;
    //     return false;
        
    // }
    std::cout << "INITIAL T: " <<  tEigen << std::endl;

    Sophus::SE3d currPose(REigen,tEigen);
    currentFrame->setPose(currPose.inverse());

    // insert current and last frame into the map
    map->insertKeyFrame(lastFrame);
    map->insertKeyFrame(currentFrame);

    //triangulate based on the last and currentFrame
    CV3DPoints new3dPoints = triangulate2View(lastFrame, currentFrame, filterMatches, true);
    // std::cout <<  "Triangulated ponts: " <<  new3dPoints.size() << std::endl;
    // // Points used in 3d triangulation = same as the ones matched after outlier removal



    // for (int i=0;i<new3dPoints.size();i++) {
    //     MapPoint::ptr newPt = MapPoint::createMapPoint();
    //     // std::cout << new3dPoints[i].x << " " << new3dPoints[i].y << " " << new3dPoints[i].z << std::endl;
       
    //     newPt->setPos(new3dPoints[i]);
    //     // add features that map to the 3d point
    //     newPt->addObservation(lastFrame->features[i]);
    //     newPt->addObservation(currentFrame->features[i]);
    //     // insert 3d points that are part of the features in frame
    //     lastFrame->features[i]->mapPoint = newPt;
    //     currentFrame->features[i]->mapPoint = newPt;
    //     map->insertMapPoint(newPt);
    // }

    // // REMEMBER RELATIVE IN ORIGINAL IS SET TO INVERSE!*************************
    relativeMotion = currentFrame->pose * lastFrame->pose.inverse();


    return true;
}





CV3DPoints LoopHandler::triangulate2View(Frame::ptr lastFrame, Frame::ptr currFrame, 
                                                    const std::vector<Matches> filtMatches, bool firstView) {
    
    CV2DPoints lastFramePts;
    CV2DPoints currFramePts;
    int landMarks = 0;

    for (int i = 0; i < filtMatches.size(); i++) {
        // lastFramePts.push_back(cv::Point(filtMatches[i].pt1.x, filtMatches[i].pt1.y));
        // currFramePts.push_back(cv::Point(filtMatches[i].pt2.x, filtMatches[i].pt2.y));
        // Vec2 pt1 = Vec2(filtMatches[i].pt1.x, filtMatches[i].pt1.y);
        // Vec2 pt2 = Vec2(filtMatches[i].pt2.x, filtMatches[i].pt2.y);
        std::vector<Sophus::SE3d> poses{lastFrame->pose, currFrame->pose};

        std::vector<Vec3> points {
            pixel2camera(cv::Point(filtMatches[i].pt1.x, filtMatches[i].pt1.y), handler3D.intrinsics.Left.getK()),
            pixel2camera(cv::Point(filtMatches[i].pt2.x, filtMatches[i].pt2.y), handler3D.intrinsics.Left.getK())};
        Vec3 pworld = Vec3::Zero();
        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
            auto newMapPoint = MapPoint::createMapPoint();
            if (firstView) {
                // pworld = currFrame->pose.inverse() * pworld;
                newMapPoint->setPos(pworld);
                newMapPoint->addObservation(currFrame->features[i]);
                newMapPoint->addObservation(lastFrame->features[i]);
                currentFrame->features[i]->mapPoint = newMapPoint;
                lastFrame->features[i]->mapPoint = newMapPoint;
                map->insertMapPoint(newMapPoint);
             // insert 3d points that are part of the features in frame
                landMarks++;
            } else {
                // std::cout << landMarks << " " << pworld << std::endl;
                Feature::ptr feat2 = std::make_shared<Feature>(currentFrame, cv::Point2d(filtMatches[i].pt2.x, filtMatches[i].pt2.y));
                feat2->mapPoint = newMapPoint;

                currentFrame->features.push_back(feat2);
                // pworld = lastFrame->pose* pworld;
                newMapPoint->setPos(pworld);
                newMapPoint->addObservation(feat2);
                map->insertMapPoint(newMapPoint);
                landMarks++;
            }
            
            
        }

    }
    std::cout << "New Landmarks: " << landMarks << std::endl;

    // // Placeholder for triangulated points
    // cv::Mat pnts3D(4, lastFramePts.size(), CV_64F);
   
    // cv::Mat P0 = sophus2ProjMat(lastFrame);
    // cv::Mat P1 = sophus2ProjMat(currFrame);

    // cv::triangulatePoints(P0, P1, lastFramePts, currFramePts, pnts3D);

    CV3DPoints pts3D;
    // for (int i = 0; i < pnts3D.cols; i++) {
    //     cv::Point3d pt3d(pnts3D.at<double>(0, i)/pnts3D.at<double>(3, i), pnts3D.at<double>(1, i)/pnts3D.at<double>(3, i), pnts3D.at<double>(2, i)/pnts3D.at<double>(3, i));
    //     if (pt3d.z > 0) {
    //         pts3D.push_back(pt3d);
    //     }
    // }

    return pts3D;


}



int LoopHandler::optimizePoseOnly() {
    
    typedef g2o::BlockSolver_6_3 BlockSolverType;  // pose is 6, landmark is 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // Construct optimzation algorithm
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);

    // Add camera vertex
    VertexPose *vertexPose = new VertexPose();
    vertexPose->setId(0);
    vertexPose->setEstimate(currentFrame->pose);
    optimizer.addVertex(vertexPose);

    // Camera parameters Eigen
    Eigen::Matrix3d KEigen;
    cv::Mat _K = handler3D.intrinsics.Left.getK();
  cv::Mat currImgcopy1, currColr1;
    cv::Mat currImgcopy2, currColr2;

    currentFrame->rawImage.copyTo(currImgcopy1);
    currentFrame->rawImage.copyTo(currImgcopy2);

    cv::cvtColor(currImgcopy1, currColr1, cv::COLOR_GRAY2RGB);
    cv::cvtColor(currImgcopy2, currColr2, cv::COLOR_GRAY2RGB);

    KEigen << _K.at<double>(0, 0), _K.at<double>(0, 1), _K.at<double>(0, 2),
              _K.at<double>(1, 0), _K.at<double>(1, 1), _K.at<double>(1, 2),
              _K.at<double>(2, 0), _K.at<double>(2, 1), _K.at<double>(2, 2);

    // // Convert 3d and 2d points in the form of Eigen
    // VecVector2d points2Deigen;
    // VecVector3d points3Deigen;
    // convertToEigen(points3D, points2D, points2Deigen, points3Deigen);
    std::vector<EdgeProjectionPoseOnly*> edges;
    std::vector<Feature::ptr> features;
    int index=1;
    // size of current frame features;
    std::cout << "current frame size in OPTI^^^^^^^" << currentFrame->features.size() << std::endl;
    std::cout << "OPTI 2NS s^^^^^^^" << currentFrame->features[2]->mapPoint.lock()->getPos() << std::endl;

    for (auto &eachKp:currentFrame->features) {
        if (auto mp = eachKp->mapPoint.lock()) {

            features.push_back(eachKp);
            Eigen::Vector3d point3d;
            Eigen::Vector2d point2d;
            // are the map points w.r.t to second or first frame
            Vec3 pos = mp->getPos();
            
            point3d << pos[0] , pos[1], pos[2];
            // std::cout << "3d points " << pos.coeff(0) << " " << pos.coeff(1)  << " " << pos.coeff(2)  << std::endl;
            point2d << eachKp->kp.x, eachKp->kp.y;
            // std::cout << "2d points " << eachKp->kp.x  << " " << eachKp->kp.y << " " << std::endl;
            auto currKp = currentFrame->world2Camera(pos, currentFrame->pose, handler3D.intrinsics.Left.getK());
            cv::Point2i newPts = cv::Point2d(currKp.coeff(0)/currKp.coeff(2), currKp.coeff(1)/currKp.coeff(2));
            cv::circle(currColr1, cv::Point2i(eachKp->kp.y, eachKp->kp.x), 5, cv::Scalar(0,0,255), -1);
            cv::circle(currColr1, cv::Point2i(newPts.y, newPts.x), 5, cv::Scalar(0,255,0), -1);

            // std::cout << "2d points reinstated" << newPts.x  << " " << newPts.y << " " << std::endl;

            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(point3d, KEigen);
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

    // cv::imwrite("opti_frame" + std::to_string(currentFrame->frameID) + ".png", currColr1);

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
                // std::cout << e->computeError() << std::endl;
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

    // Sophus::SE3d correctedPose(vertexPose->estimate().rotationMatrix(), vertexPose->estimate().translation());
    // currentFrame->setPose(correctedPose);

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





inline bool LoopHandler::triangulation(const std::vector<Sophus::SE3d> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
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

Vec3 LoopHandler::pixel2camera(const cv::Point p, const cv::Mat K) {
    double depth = 1;
    return Vec3(
            (p.x - K.at<double>(0,2)) * depth / K.at<double>(0,0),
            (p.y - K.at<double>(1,2)) * depth / K.at<double>(1,1),
            1
    );
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