#include "Utils.hpp"
#include "Map.hpp"
#include "MapPoint.hpp"
#include <pangolin/display/display.h>
#include <pangolin/plot/plotter.h>

class Viewer {
    public:
        typedef std::shared_ptr<Viewer> ptr; 
        Viewer();
        Map::ptr viewerMap;
        void addCurrentFrame(Frame::ptr frame);
        // Pin the map used for visualization 
        void setMap(Map::ptr _map);
        // Every now and then lock the viewer thread and update KFs and MPs 
        void updateMap();
        // main thread loop that plots all KFs and MPs
        void plotterLoop();
        // draw single frame
        void drawFrame(Frame::ptr frame, const float *color);
        // draw single mappoint
        void drawMPs();
        // follow camera when drawing
        void followCurrentFrame(pangolin::OpenGlRenderState& visCamera);
        // Close everything 
        void close();
        void viewerRun();
        cv::Mat plotFromImage();
        // static Viewer::ptr createViewer();
        std::thread viewerThread;

    private:
        std::mutex viewerMutex;
        Map::ptr map = nullptr;
        Frame::ptr currentFrame = nullptr;
        bool viewerRunning = true;
        std::unordered_map<unsigned long, MapPoint::ptr> mps;
        std::unordered_map<unsigned long, Frame::ptr> frames;
};