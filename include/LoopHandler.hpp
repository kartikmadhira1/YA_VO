#ifndef LOOPHANDLER_H
#define LOOPHANDLER_H
#include <fstream>
#include <json/json.h>
#include "Utils.hpp"
#include "Frame.hpp"
#include "MapPoint.hpp"
#include "Map.hpp"
#include "viz.hpp"
#include "Optimizer.hpp"
#include "BriefDescriptor.hpp"
#include "FastDetector.hpp"
#include "3DHandler.hpp"
/*
1. Load config file - what sequence, base path, etc.
2. iterate through images
3.  Need methods like next() and iter() for image iteration -> this should be different class or should be in LoopHandler?
*/

// enum sensorType {stereo, mono}

typedef std::vector<cv::Point2d> CV2DPoints;
typedef std::vector<cv::Point3d> CV3DPoints;

enum voStatus {INIT, TRACKING, ERROR, RESET};

class LoopHandler {
    private:
        std::string seqNo;
        std::string leftImagesPath;
        std::string rightImagesPath;
        bool isStereo;
        void generatePathTrain();
        Frame::ptr currentFrame = nullptr;
        Frame::ptr lastFrame = nullptr;
        std::vector<std::string>::iterator _trainIterator;
        voStatus status = voStatus::INIT;
        int currentFrameId;

    public:
        LoopHandler();
        void setViewer();
        void insertFrameFeaturesOPENCV(Frame::ptr _frame);
        LoopHandler(const std::string &config);
        Map::ptr map = nullptr;
        Viewer::ptr viz = nullptr;
        Brief brief;
        FastDetector fd;
        std::vector<std::string> leftPathTrain;
        std::vector<std::string> rightPathTrain;

        Optimizer optim;
        _3DHandler handler3D;
        // Relative motion between last and current frame
        Sophus::SE3d relativeMotion;

        bool track();
        // Gather an image from the path train
        Frame::ptr getNextFrame();
        // takes a single step of adding a frame to the pipeline
        bool takeVOStep();
        //prepare a frame with it's keypoints
        void insertFrameFeatures( Frame::ptr _frame);
        // adds frame and passes onto other modules based on the status
        void addFrame(Frame::ptr _frame);
        // build initial map with current frame and last frame;
        bool buildInitMap();
        // run VO
        void runVO();
        // Track previous frame and coint inliers
        int trackLastFrame();
        int optimizePoseOnly();
        // triangulate points
        CV3DPoints triangulate2View(Frame::ptr lastFrame, Frame::ptr currFrame, 
                            const std::vector<Matches> filtMatches);

        // reinit
        bool reinitialize();

        cv::Mat sophus2ProjMat( Frame::ptr _frame);
        std::string getSeqNo();
        bool stereoStatus();
        std::string getLeftImagesPath();
        int getLeftTrainLength();
        cv::Ptr<cv::ORB> gftt_;  // feature detector in opencv

        ~LoopHandler();
};






/*

Lets make the easiest case for a feasable output

1. Add 1st frame -> set pose to identity -> update viewer 
2. Add 2nd frame -> do triangulation and pose update -> update viewer


*/
#endif // TODOITEM_H
