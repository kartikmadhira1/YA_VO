#include "../include/Frame.hpp"
#include "../include/Utils.hpp"


void Frame::setPose(Sophus::SE3d pose_) {
    std::unique_lock<std::mutex> lock(poseMutex);
    this->pose = pose_;
}

Sophus::SE3d Frame::getPose() {
    std::unique_lock<std::mutex> lock(poseMutex);
    return this->pose;
}


Vec3 Frame::world2Camera(const Vec3 &mp3D, const Sophus::SE3d &pose_, const cv::Mat &K) {

    Eigen::Matrix<double, 4, 1> homo3DPt;
    homo3DPt << mp3D.coeff(0), mp3D.coeff(1), mp3D.coeff(2), 1;
    // std::cout << "Vec3d point" <<  mp3D << std::endl;
    // std::cout << "Vec3d point" <<  homo3DPt << std::endl;
    Eigen::Matrix3d KEigen;
    KEigen <<K.at<double>(0, 0), K.at<double>(0, 1),  K.at<double>(0, 2),
                                K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
                                K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);
    Vec3 camPoints = KEigen*pose_.matrix3x4()*homo3DPt;
    return camPoints;
}


unsigned long Frame::createFrameID() {
    static int frameID_ = 0;
    frameID_++;
    return frameID_;
}