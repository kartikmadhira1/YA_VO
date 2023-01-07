#include "../include/viz.hpp"

Viewer::Viewer() {

}

void Viewer::viewerRun() {
    // viewerThread = std::thread(std::bind(&Viewer::plotterLoop, this));
    plotterLoop();
}

void Viewer::close() {
    viewerRunning = false;
}

void Viewer::addCurrentFrame(Frame::ptr frame) {
    std::unique_lock<std::mutex> viewLock(viewerMutex);
    currentFrame = frame;
}

void Viewer::setMap(Map::ptr _map) {
    std::unique_lock<std::mutex> viewLock(viewerMutex);
    this->map = _map;
}

void Viewer::updateMap() {
    std::unique_lock<std::mutex> viewLock(viewerMutex);
    frames = map->getActiveFrames();
    mps = map->getActiveMPs();
}


void Viewer::plotterLoop() {
    try {
        pangolin::CreateWindowAndBind("YA_VIO", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
    
    pangolin::OpenGlRenderState visCamera(
        pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& visDisplay =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(visCamera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() && viewerRunning) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        visDisplay.Activate(visCamera);

        std::unique_lock<std::mutex> lock(viewerMutex);
        if (currentFrame) {
            drawFrame(currentFrame, green);
            followCurrentFrame(visCamera);

            cv::Mat img = plotFromImage();
            cv::imshow("image", img);
            cv::waitKey(1);
        }

        if (map) {
            drawMPs();
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
}


cv::Mat Viewer::plotFromImage() {
    cv::Mat img;
    // cv::cvtColor(currentFrame->rawImage, img, CV_GRAY2BGR);
    currentFrame->rawImage.copyTo(img);
    for (size_t i = 0; i < currentFrame->features.size(); ++i) {
        if (currentFrame->features[i]->mapPoint.lock()) {
            auto feat = currentFrame->features[i];
            // std::cout << feat->kp << std::endl;
            cv::circle(img, cv::Point2i(feat->kp.y, feat->kp.x), 2, cv::Scalar(0, 250, 0),
                       2);
        }
    }
    return img;
}


void Viewer::followCurrentFrame(pangolin::OpenGlRenderState& visCamera) {
    Sophus::SE3d Twc = this->currentFrame->pose.inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    visCamera.Follow(m, true);
}



void Viewer::drawFrame(Frame::ptr frame, const float *color) {
    Sophus::SE3d Twc = frame->pose.inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void Viewer::drawMPs() {
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : frames) {
        drawFrame(kf.second, red);
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto& ld: mps) {
        auto pos = ld.second->getPos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}


// Viewer::ptr Viewer::createViewer() {

//     Viewer::ptr viz = std::make_shared<Viewer>();
//     return viz;
// }