#include "../include/LoopHandler.hpp"



int main() {
    std::string configPath = "../config/KITTI.json";
    LoopHandler Lh(configPath);
    Lh.handler3D.intrinsics.Left.printK();
    // Lh.viz->viewerRun();
    // Lh.viz->viewerThread = std::thread(std::bind(&Viewer::plotterLoop, Lh.viz));
    std::thread lhThread = std::thread(std::bind(&LoopHandler::runVO, Lh));
    Lh.viz->viewerRun();
}