#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <ctype.h>
#include <mutex>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/imgproc/imgproc.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"
#include <opencv2/core/eigen.hpp>


// common typedefs
typedef Eigen::Matrix<double, 3, 1> Vec3;


struct Camera {
    public:
        Camera() {

        }
        Camera(const cv::Mat &_cam) {
            _cam.copyTo(fullMatrix);
            K = fullMatrix(cv::Range(0, 3), cv::Range(0, 3));
        }
        cv::Mat K;
        cv::Mat fullMatrix;
        cv::Mat getK() {
            return K;
        }
        float getFx() {
            return K.at<double>(0,0);
        }
        float getFy() {
            return K.at<double>(1,1);
        }
        float getCx() {
            return K.at<double>(0,2);
        }
        float getCy() {
            return K.at<double>(1,2);
        }
        void getCalibMat() {}
        void printK() {
            for (int i = 0; i < K.rows; i++) {
                for (int j=0; j<K.cols;j++) {
                    std::cout <<  K.at<double>(i, j) << " ";
                }
                std::cout <<  "\n" << std::endl;
            }
        }
        ~Camera() {
        }
};


struct Intrinsics {
    public:
        Intrinsics() {

        }
        typedef std::shared_ptr<Intrinsics> ptr;
        Camera Left;
        Camera Right;

        ~Intrinsics() {

        }
};


void parseCalibString(std::string string, cv::Mat &cvMat);



void getCalibParams(std::string _path, Intrinsics &calib);


std::vector<boost::filesystem::path> getFilesInFolder(const std::string &path);


std::string type2str(int type);


#endif // TODOITEM_H
