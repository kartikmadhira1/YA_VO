#include "Utils.hpp"
#include "Image.hpp"



class 3DHandler {
    private:
        Intrinsics intrinsics;
    public:
        3DHandler();
        // need to have these functions usable without instantiating the class
        void getRT(const Image &img1, const Image &img2);
        bool getEssentialMatrix(const std::vector<Matches> &matches, cv::Mat &E);
        void disambiguateRT(const cv::Mat &E, const cv::Mat &u, const cv::Mat &w, cv::Mat &vt, cv::Mat &finalR, cv::Mat &finalT);
        ~3DHandler();
};