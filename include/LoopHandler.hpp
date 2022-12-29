#ifndef LOOPHANDLER_H
#define LOOPHANDLER_H
#include <fstream>
#include <json/json.h>
#include "Utils.hpp"
#include "Frame.hpp"
#include "BriefDescriptor.hpp"
#include "FastDetector.hpp"
#include "3DHandler.hpp"
/*
1. Load config file - what sequence, base path, etc.
2. iterate through images
3.  Need methods like next() and iter() for image iteration -> this should be different class or should be in LoopHandler?
*/

// enum sensorType {stereo, mono}

enum voStatus {INIT, TRACKING, ERROR};



class LoopHandler {
    private:
        std::string seqNo;
        std::string leftImagesPath;
        std::string rightImagesPath;
        bool isStereo;
        void generatePathTrain();
        Frame::ptr currentFrame = nullptr;
        Frame::ptr lastFrame = nullptr;
        voStatus status = voStatus::INIT;
        std::vector<std::string>::iterator _trainIterator;
        int currentFrameId;
    public:
        LoopHandler();
        LoopHandler(const std::string &config);
        Brief brief;
        FastDetector fd;
        std::vector<std::string> leftPathTrain;
        std::vector<std::string> rightPathTrain;


        _3DHandler handler3D;
        // Gather an image from the path train
        Frame::ptr getNextFrame();
        // takes a single step of adding a frame to the pipeline
        bool takeVOStep();
        //prepare a frame with it's keypoints
        void insertFrameFeatures( Frame::ptr _frame);
        // adds frame and passes onto other modules based on the status
        void addFrame(Frame::ptr _frame);
        // build initial map with current frame and last frame;
        void buildInitMap();

        
        
       
        std::string getSeqNo();
        bool stereoStatus();
        std::string getLeftImagesPath();
        int getLeftTrainLength();
        ~LoopHandler();
};






/*

Lets make the easiest case for a feasable output

1. Add 1st frame -> set pose to identity -> update viewer 
2. Add 2nd frame -> do triangulation and pose update -> update viewer


*/
#endif // TODOITEM_H
