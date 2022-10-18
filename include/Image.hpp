#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include "FeatureExtractor.hpp"

class Image {

    public :
        cv::Mat _img;
        FeatExt _features;
        Image::Image(const cv::Mat &_img);
        cv::Mat unDistort();
        
        Image::~Image();
        
};