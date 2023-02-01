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
    return retVec;
}

void Brief::convolve2d(const Image &img, cv::Mat &kernel, cv::Mat &output) {
    // 1. Get the kernel size
    // 2. Get the image size
    // 3. For each pixel in the image
    // 4. Get the kernel size around the pixel
    // 5. Multiply the kernel with the image patch
    // 6. Sum the values and store in the output image
    // 7. Return the output image

    // Add border to the original image
    int sumKer=0;
    int kernelSize = kernel.rows;
    int imgSize = img.rawImage.rows;
    for (int k=0;k<kernelSize;k++) {
        for (int l=0;l<kernelSize;l++) {
    
            sumKer += kernel.at<float>(k, l);
        }
    }
    cv::Mat imgWithBorder = cv::Mat::zeros(img.rawImage.rows + 2, img.rawImage.cols + 2, CV_8UC1);

    
    cv::copyMakeBorder(img.rawImage, imgWithBorder, int(kernelSize/2), int(kernelSize/2), int(kernelSize/2), int(kernelSize/2), cv::BORDER_CONSTANT, 0);
    for (int i=kernelSize/2;i<img.rawImage.rows-int(kernelSize/2);i++) {
        for (int j=kernelSize/2;j<img.rawImage.cols-int(kernelSize/2);j++) {
            float sum = 0;
            for (int k=0;k<kernelSize;k++) {
                for (int l=0;l<kernelSize;l++) {
            
                    sum += kernel.at<float>(k, l) * imgWithBorder.at<uchar>(i+k-int(kernelSize/2), j+l-int(kernelSize/2));
                }
            }
            output.at<float>(i-int(kernelSize/2), j-int(kernelSize/2)) = sum;
        }
    }
}


void Brief::gaussianBlur(const Image &img, int sigma, cv::Mat &outImage) {
    // Thumb rule is the kernel size is 3*sigma
    int kernelSize = (int)3*sigma;
    cv::Mat gaussKernel1DX = cv::Mat::zeros(kernelSize, 1, CV_32FC1);

    // Create two 1D gaussian kernels - X
    for (int i=0;i<kernelSize;i++) {
        gaussKernel1DX.at<float>(i, 0) = (1/(sqrt(2*M_PI)*sigma)) * exp(-pow(i, 2)/(2*pow(sigma, 2)));
    }
    // Create two 1D gaussian kernels - Y

    cv::Mat gaussKernel1DY = cv::Mat::zeros(1, kernelSize, CV_32FC1);
    for (int i=0;i<kernelSize;i++) {
        gaussKernel1DY.at<float>(0, 1) = (1/(sqrt(2*M_PI)*sigma)) * exp(-pow(i, 2)/(2*pow(sigma, 2)));
    }
    // Ideally we should be convolving with conseqcutive 1D kernels
    // @todo: Implement convolving with 1d filter
    cv::Mat gaussKernel = gaussKernel1DX * gaussKernel1DY;

    convolve2d(img, gaussKernel, outImage);

    outImage.convertTo(outImage, CV_8UC1);

}


void Brief::computeBrief(const std::vector<cv::Point> &detectedCornerPoints, Image &img) {
    cv::Mat imgComp = cv::Mat::zeros(img.getH(), img.getW(), CV_8UC1);
    // gaussianBlur(img, 3, imgComp);
    // apply gaussian blur
    cv::GaussianBlur(img.rawImage, imgComp, cv::Size(9, 9), 2.5, 2.5);
    Image tempImg(imgComp);

    for (int i=0; i<detectedCornerPoints.size(); i++) {
        // Make this is a keyPoint
        KeyPoint newKpt(detectedCornerPoints[i].x, detectedCornerPoints[i].y, i);
        // Now for this keypoint, make a 256 bit featVec based on the offsets generated;
        if (checkBoundry(newKpt.y, newKpt.x, img.getW(), img.getH())) {
                for (int j=0; j<patchSize;j++) {

                    cv::Point P1;
                    cv::Point P2;

                    P1.x = detectedCornerPoints[i].x + offsets[j][0];
                    P1.y = detectedCornerPoints[i].y + offsets[j][1];
                    P2.x = detectedCornerPoints[i].x + offsets[j][2];
                    P2.y = detectedCornerPoints[i].y + offsets[j][3];
                    // conforms to the jth byte in the 32 byte sized uchar
                    int ithDescUcharIndex = j/8;
                    // std::cout << ithDescUcharIndex << std::endl;
                    // std::cout << P1.x  << " " <<  P1.y << "  points " << P2.x << " " << P2.y << std::endl;
                    if (tempImg.getPixelVal(P1.x, P1.y) > tempImg.getPixelVal(P2.x, P2.y)) {
                        // Set the ith bit to 1
                        // 1 << (j%8) means setting 1 at the th indices 0,1,2,3,4,5,6,7 in the ith byte
                        newKpt.featVec[ithDescUcharIndex] |= 1 << (j%8);
                        // https://stackoverflow.com/questions/47981/how-do-i-set-clear-and-toggle-a-single-bit
                    } else {
                        newKpt.featVec[ithDescUcharIndex] &= ~(1 << (j%8));
                    }                    
                }
                
            img.keypoints.push_back(newKpt);
        }
    }
}



inline bool Brief::checkBoundry(int x, int y, int width, int height) {
    if (x-8< 0 || x+8 > width) {
        return false;
    }
    if (y-8< 0 || y+8 > height) {
        return false;
    }
    return true;
}


int Brief::hammingDistance(uchar featVec1[32], uchar featVec2[32]) {
    int hammingDist = 0;
    for (int i=0; i<32; i++) {
        hammingDist += popCount(featVec1[i] ^ featVec2[i]);

    }
    return hammingDist;
}


// Good explanation of the code below:
//https://stackoverflow.com/a/9867227/6195275
int Brief::popCount(uchar featVec) {
    int count = 0;
    while(featVec!=0){
        if (featVec & 0x1) {
            count++;
        }
        featVec = featVec >> 1;
    }
    return count;
}


std::vector<Matches> Brief::matchFeatures( Image &img1,  Image &img2) {
    std::vector<Matches> matches;
    if (img1.resetKeypoints.size() != 0) {
        img1.keypoints = img1.resetKeypoints;
    }
    for (int i=0; i<img1.keypoints.size(); i++) {
        int minDist = INT_MAX;
        KeyPoint kp2(0,0,0);
        for (int j=0; j<img2.keypoints.size(); j++) {
            int hammingDist = hammingDistance(img1.keypoints[i].featVec, img2.keypoints[j].featVec);
            if (hammingDist < minDist) {
                minDist = hammingDist;
                // update match with the lowest hamming distance
                kp2.x = img2.keypoints[j].x;
                kp2.y = img2.keypoints[j].y;
                kp2.id = img2.keypoints[j].id;
            }
        }
        // create Match pair with the lowest hamming distance
        Matches match(img1.keypoints[i], kp2, minDist);
        matches.push_back(match);
    }
    return matches; 
}


cv::Mat Brief::drawMatches(Image &img1, Image &img2, std::vector<Matches> &matches) {
    cv::Mat output = cv::Mat::zeros(img1.getH(), img1.getW()+img2.getW(), CV_8UC1);
    // Copy img1 to the left half of the output
    for (int i=0;i<img1.getH();i++) {
        for (int j=0;j<img1.getW()-1;j++) {
            output.at<uchar>(i,j) = img1.getPixelVal(i,j);
        }
    }
    // Copy the next half of the output
    for (int i=0;i<img2.getH()-1;i++) {
        for (int j=0;j<img2.getW()-1;j++) {
            output.at<uchar>(i,j+img1.getW()) = img2.getPixelVal(i,j);
        }
    }

    cv::cvtColor(output, output, cv::COLOR_GRAY2RGB);

    for (Matches &match : matches) {
        cv::Point p1(match.pt1.y, match.pt1.x);
        cv::Point p2(match.pt2.y+img1.getW(), match.pt2.x);
        cv::line(output, p1, p2, cv::Scalar(255,255,255), 1);
    }

    return output;
}


void Brief::removeOutliers(std::vector<Matches> &matches, std::vector<Matches> &newMatches, int threshold=30) {
    // Get min and max distance
    int minDist = INT_MAX;
    int maxDist = INT_MIN;

    auto result = std::minmax_element(matches.begin(), matches.end(), [](const Matches &a, const Matches &b) {
        return a.distance < b.distance;
    });

    // Remove outliers

    for (auto &eachMatch : matches) {
        if (eachMatch.distance < std::max(2*result.first->distance, threshold)) {
            eachMatch.pt1.matched = true;
            eachMatch.pt2.matched = true;
            newMatches.push_back(eachMatch);
        }
    }
}