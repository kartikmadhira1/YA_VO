#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <ctype.h>
#include <boost/filesystem.hpp>
#include "opencv4/opencv2/imgproc/imgproc.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"





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
        void getFx();
        void getFy();
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
        Camera Left;
        Camera Right;

        ~Intrinsics() {

        }
};


void parseCalibString(std::string string, cv::Mat &cvMat);



void getCalibParams(std::string _path, Intrinsics &calib);


std::vector<boost::filesystem::path> getFilesInFolder(const std::string &path);


std::string type2str(int type);