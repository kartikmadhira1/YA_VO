#pragma once
#include "Utils.hpp"
#include "Image.hpp"
#include "LoopHandler.hpp"
#include <random>

class KeyPoint {
    public:
        KeyPoint(const int _x, const int _y, const int _id) {
            this->x = _x;
            this->y = _y;
            this->id = _id;
        }
        int x;
        int y;
        int id;
        uchar *featVec[32];
};


class Match {
    public:



}


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
        inline bool checkBoundry(int x, int y, int width, int height);
        int hammingDistance(uchar *featVec1, uchar *featVec2);
        std::vector<std::vector<int>> preComputeOffsets();
        void computeBrief(const std::vector<cv::Point> &detectedCornerPoints, const Image &img);

}