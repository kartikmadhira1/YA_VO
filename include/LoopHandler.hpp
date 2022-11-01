#pragma once
#include <fstream>
#include <jsoncpp/json/json.h>
#include "Image.hpp"




/*
1. Load config file - what sequence, base path, etc.
2. iterate through images
3.  Need methods like next() and iter() for image iteration -> this should be different class or should be in LoopHandler?
*/


// enum sensorType {stereo, mono}
class LoopHandler {
    private:
        std::string seqNo;
        std::string leftImagesPath;
        std::string rightImagesPath;
        bool isStereo;
        std::vector<std::string> leftPathTrain;
        std::vector<std::string> rightPathTrain;

        void generatePathTrain();
    public:
        LoopHandler();
        LoopHandler(const std::string &config);
        std::string getSeqNo();
        bool stereoStatus();
        std::string getLeftImagesPath();
        Image getNextImage();
        int getLeftTrainLength();
        ~LoopHandler();
};

