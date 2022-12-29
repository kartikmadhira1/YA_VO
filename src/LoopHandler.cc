#include "../include/LoopHandler.hpp"

LoopHandler::LoopHandler() {

}

LoopHandler::LoopHandler(const std::string &config) : brief (256), fd(12, 50)  {
    std::ifstream ifs(config);
    Json::Reader reader;
    Json::Value value;
    reader.parse(ifs, value);
    std::string basePath = value["basePath"].asString();
    std::string calibPath = basePath + "calib.txt";
    handler3D.setCalibParams(calibPath);
    seqNo = value["sequence"].asString();
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
    if (status == voStatus::INIT) {
        currentFrame = _frame;
        if (lastFrame != nullptr) {
            buildInitMap();
        }
    } 
    lastFrame = currentFrame;
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
    for (auto &eachFeature : features) {
        Feature::ptr feat = std::make_shared<Feature>(_frame, cv::Point2i(eachFeature.x, eachFeature.y));
        _frame->features.push_back(feat);
    }
}


void LoopHandler::buildInitMap() {

    std::vector<Matches> matches = brief.matchFeatures(*currentFrame, *lastFrame);
    std::vector<Matches> filterMatches;

    brief.removeOutliers(matches, filterMatches, 20.0);


    handler3D.intrinsics.Left.printK();
    cv::Mat F = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat u = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat vt = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat w = cv::Mat::zeros(3, 3, CV_64F);

    handler3D.getFRANSAC(filterMatches, F, 200, 0.1);
    
    // Essential matrix from fundamental matrix
    cv::Mat E = handler3D.intrinsics.Left.getK().t() * F * handler3D.intrinsics.Left.getK();


    Pose p = handler3D.disambiguateRT(E, filterMatches);



}


Frame::ptr LoopHandler::getNextFrame() {
    if (_trainIterator != leftPathTrain.end()) {
        cv::Mat img = cv::imread(*_trainIterator, 0);
        currentFrameId = _trainIterator - leftPathTrain.begin();
        _trainIterator++;
        // ADD FRAME ID TO FRAME!!!!!!!!!!!!!-----------------
        Frame::ptr frame = std::make_shared<Frame>(img, currentFrameId);    
        return frame;
    }
    return nullptr;
}





LoopHandler::~LoopHandler() {}