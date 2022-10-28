#include "../include/LoopHandler.hpp"




LoopHandler::LoopHandler(const std::string &config) {
    std::ifstream(config)
    Json::Reader reader;
    Json::Value value;
    reader.parse(ifs, obj);
    std::string basePath = obj["basePath"].asString();
    seqNo = obj["sequence"].asString();
    std::string camType = obj["cameraType"].asString();
    if (camType == "mono") {
        isStereo = false;
        leftImagesPath = basePath + seqNo + "/images_0/";
        // rightImagesPath = basePath + seqNo + "/images_1/";
        rightImagesPath = "";
    } else {
        rightImagesPath = basePath + seqNo + "/images_1/";
    }
    generatePathTrain();
}



void LoopHandler::generatePathTrain() {
    std::vector<std::filesystem::path> filesLeft;
    std::vector<std::filesystem::path> filesRight;
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
