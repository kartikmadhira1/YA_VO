#include "../include/BriefDescriptor.hpp"





std::vector<std::vector<int>> Brief::preComputeOffsets() {
    // We are going to generate a vector with size numTests*4
    // Randomly generate 4 values from -8 to +8
    std::random_device seeder;
    std::mt19937 generator(seeder());
    std::uniform_int_distribution<int> dist(-8, 8);
    std::vector<std::vector<int>> retVec;
    for (int i=0;i<256;i++) {
        std::vector<int> vec;
        for(int j=0;j<4; j++) {
            int randomNum = dist(generator);
            vec.push_back(randomNum);
        }
        retVec.push_back(vec);
    }
}

void Brief::computeBrief(const std::vector<cv::Point> &detectedCornerPoints, const Image &img) {
    
    for (int i=0; i<detectedCornerPoints.size(); i++) {
        // Make this is a keyPoint
        KeyPoint newKpt(detectedCornerPoints[i].x, detectedCornerPoints[i].y, i);
        // Now for this keypoint, make a 256 bit featVec based on the offsets generated;
        if (checkBoundry(newKpt.x, newKpt.y, img.getW(), img.getH())) {
                for (int j=0; j<patchSize;j++) {

                    cv::Point P1;
                    cv::Point P2;

                    P1.x = detectedCornerPoints[i].x + offsets[j][0];
                    P1.y = detectedCornerPoints[i].y + offsets[j][1];
                    P2.x = detectedCornerPoints[i].x + offsets[j][2];
                    P2.y = detectedCornerPoints[i].y + offsets[j][3];
                    // conforms to the jth byte in the 32 byte sized uchar
                    int ithDescUcharIndex = j/8;
                    if (img.getPixelVal(P1.x, P1.y) > img.getPixelVal(P2.x, P2.y)) {
                        // Set the ith bit to 1
                        // 1 << (j%8) means setting 1 at the th indices 0,1,2,3,4,5,6,7 in the ith byte
                        newKpt.featVec[ithDescUcharIndex] |= (1 << (j%8));
                        // https://stackoverflow.com/questions/47981/how-do-i-set-clear-and-toggle-a-single-bit
                    } else {
                        newKpt.featVec[ithDescUcharIndex] &= ~(1 << (j%8));
                    }                    
                }
                
            img.keypoints.push_back(newKpt);
        }
    }
}

bool 


inline bool Brief::checkBoundry(int x, int y, int width, int height) {
    if (x-8< 0 || x+8 > width) {
        return false;
    }
    if (y-8< 0 || y+8 > height) {
        return false;
    }
    return true;
}


int Brief::hammingDistance(uchar *featVec1, uchar *featVec2) {
    int hammingDist = 0;
    for (int i=0; i<32; i++) {
        hammingDist += __builtin_popcount(featVec1[i] ^ featVec2[i]);
    }
    return false;
}


// Good explanation of the code below:
//https://stackoverflow.com/a/9867227/6195275
int Brief::popCount(uchar featVec) {
    int count = 0;
    while(featVec!=0){
        if (c & 0x1) {
            count++;
        }
        featVec = featVec >> 1;
    }
}