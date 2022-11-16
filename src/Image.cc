#include "../include/Image.hpp"

/**
 * @brief Construct a new Image:: Image object
 * 
 * @param img Mat object acquired
 */

Image::Image(const cv::Mat &img) {
   rawImage = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
   img.copyTo(this->rawImage);

   // Undistort image here itself in the constructor
}

uint8_t Image::getPixelVal(int i, int j) const {
   return this->rawImage.data[i*this->rawImage.cols + j];
}


int Image::getH() const {
   return this->rawImage.rows;
}

int Image::getW() const {
   return this->rawImage.cols;
}

