#include "../include/LoopHandler.hpp"

LoopHandler::LoopHandler() {

}

LoopHandler::LoopHandler(const std::string &config) {
    std::ifstream ifs(config);
    Json::Reader reader;
    Json::Value value;
    reader.parse(ifs, value);
    std::string basePath = value["basePath"].asString();
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


void LoopHandler::takeVOStep() {
    // Get the next image 
    Frame::ptr frame = getNextFrame();
    if (frame != nullptr) {
        // Get features
        getFeatures(frame);
        // Add frame to pipeline
        addFrame(frame);
    }



}


Frame::ptr LoopHandler::getNextFrame() {
    if (_trainIterator != leftPathTrain.end()) {
        cv::Mat img = cv::imread(*_trainIterator);
        _trainIterator++;
        Frame::ptr frame = std::make_shared<Frame>(img);    
        return frame;
    }
    return nullptr;
}





LoopHandler::~LoopHandler() {}