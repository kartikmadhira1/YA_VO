#pragma once
#include "LoopHandler.hpp"
#include "Image.hpp"
#include "FastDetector.hpp"
#include <random>
#include <algorithm>
class KeyPoint {
    public:
        KeyPoint(const int _x, const int _y, const int _id) {
            this->x = _x;
            this->y = _y;
            this->id = _id;
        }
        KeyPoint() {}
        int x;
        int y;
        int id;
        bool matched = false;
        uchar featVec[32]={};
};


class Matches {
    public:
        Matches(const KeyPoint &_pt1, const KeyPoint &_pt2, int distance) {
            this->pt1 = _pt1;
            this->pt2 = _pt2;
            this->distance = distance;
        }
    Matches() {}
    KeyPoint pt1;
    KeyPoint pt2;
    int distance;

};

class Brief {
    private:
        // std::vector<std::vector<int>> offsets;
        int patchSize;
        std::vector<std::vector<int>> offsets;
    public:
        std::vector<std::vector<int>> preComputeOffsets();

        Brief(int numTests) {
            // Means that there will be 256 tests for the patch.
            this->offsets = this->preComputeOffsets();
            this->patchSize = numTests;
        }

            
        int popCount(uchar featVec);
        inline bool checkBoundry(int x, int y, int width, int height);
        int hammingDistance(uchar featVec1[32], uchar featVec2[32]);
        void convolve2d(const Image &img, cv::Mat &kernel, cv::Mat &output);
        void gaussianBlur(const Image &img, int sigma, cv::Mat &outImage);
        void computeBrief(const std::vector<cv::Point> &detectedCornerPoints,  Image &img);
        std::vector<Matches> matchFeatures( Image &img1,  Image &img2);
        cv::Mat drawMatches( Image &img1,  Image &img2, std::vector<Matches> &matches);
        void removeOutliers(std::vector<Matches> &matches, std::vector<Matches> &newMatches, int threshold);
        ~Brief() {
            // std::cout << "Brief destructor called" << std::endl;
        }
};