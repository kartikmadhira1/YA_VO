#ifndef _3DHANDLER_H
#define _3DHANDLER_H

#include "Utils.hpp"
#include "Image.hpp"
#include "opencv2/calib3d.hpp"


struct Pose {
    public:
        cv::Mat R;
        cv::Mat t;
        cv::Mat P;
        cv::Mat _3DPts;
        cv::Mat _2DPts;
        Sophus::SE3d sophusPose;
        int numChierality = 0;
        Pose(cv::Mat _R, cv::Mat _t, cv::Mat P) {
            this->R = _R;
            this->t = _R;
            this->P = P;
            Eigen::Matrix3d eigenR;
            Vec3 eigenT;
            cv::cv2eigen(_R, eigenR);
            cv::cv2eigen(_R, eigenT);

            this->sophusPose = Sophus::SE3d(eigenR, eigenT);
        }
        Pose() {}
        Pose operator=(const Pose &pose) {
            this->R = pose.R;
            this->t = pose.t;
            this->P = pose.P;
            this->numChierality = pose.numChierality;
            Eigen::Matrix3d eigenR;
            Vec3 eigenT;
            cv::cv2eigen(pose.R, eigenR);
            cv::cv2eigen(pose.t, eigenT);

            this->sophusPose = Sophus::SE3d(eigenR, eigenT);
            return *this;
        }
};



class _3DHandler {
    private:
    public:
        _3DHandler();
        Intrinsics intrinsics;

        void setCalibParams(std::string &calibFile);

        // need to have these functions usable without instantiating the class
        void getRT(const Image &img1, const Image &img2);
        bool getFundamentalMatrix(const std::vector<Matches> &matches, cv::Mat &F);
        bool getFRANSAC(std::vector<Matches> matches, cv::Mat &F, 
                      int iterations, double threshold);
        Pose disambiguateRT(const cv::Mat &E, std::vector<Matches> &matches);

        cv::Mat constructNormMatrix(std::vector<double> xVec, std::vector<double> yVec, 
                                                double xMean, double yMean);
        bool checkDepthPositive(cv::Mat &pnts3D, cv::Mat R, cv::Mat t, Pose &pose);
        float unNormalizePoint(float pt, float mean, float var);
        cv::Mat rotateMatrixZ(int rotateAngle);
        double getMeanVar(std::vector<double> &vec);

        ~_3DHandler();
};

#endif // TODOITEM_H
