#include <fstream>
#include <jsoncpp/json/json.h>
#include "Utils.hpp"

/*
1. Load config file - what sequence, base path, etc.
2. iterate through images
3.  Need methods like next() and iter() for image iteration -> this should be different class or should be in LoopHandler?
*/
class Image;
// enum sensorType {stereo, mono}
class LoopHandler {
    private:
        std::string seqNo;
        std::string leftImagesPath;
        std::string rightImagesPath;
        bool isStereo;

        void generatePathTrain();
    public:
        std::vector<std::string> leftPathTrain;
        std::vector<std::string> rightPathTrain;
        LoopHandler();
        LoopHandler(const std::string &config);
        std::string getSeqNo();
        bool stereoStatus();
        std::string getLeftImagesPath();
        int getLeftTrainLength();
        ~LoopHandler();
};

