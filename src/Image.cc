#include "../include/Image.hpp"


/**
 * @brief Construct a new Image:: Image object
 * 
 * @param img Mat object acquired
 */


Image::Image(const cv::Mat &img) {
   img.copyTo(this->rawImage);

   // Undistort image here itself in the constructor
   

}

