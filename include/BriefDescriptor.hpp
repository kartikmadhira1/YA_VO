#pragma once
#include "LoopHandler.hpp"
#include "Image.hpp"
#include <random>

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
        uchar *featVec[32]={};
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
        Brief(int numTests) {
            // Means that there will be 256 tests for the patch.
            this->offsets = preComputeOffsets();
            this->patchSize = numTests;
        }
        int popCount(uchar featVec);
        inline bool checkBoundry(int x, int y, int width, int height);
        int hammingDistance(uchar *featVec1[32], uchar *featVec2[32]);
        std::vector<std::vector<int>> preComputeOffsets();
        void computeBrief(const std::vector<cv::Point> &detectedCornerPoints,  Image &img);
        std::vector<Matches> matchFeatures( Image &img1,  Image &img2);
};