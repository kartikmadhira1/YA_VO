#include "Utils.hpp"
#include "FeatureExtractor.hpp"
class Image {

    public :
        cv::Mat img;
        cv::Mat rawImage;
        FeatExt _features;
        Image(const cv::Mat &img);
        void unDistort();      
        ~Image();
        
};