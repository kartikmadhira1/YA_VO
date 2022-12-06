#include "Utils.hpp"
#include "Image.hpp"
#include "opencv2/calib3d.hpp"


struct Pose {
    public:
        cv::Mat R;
        cv::Mat t;
        cv::Mat P;
        int numChierality = 0;
        Pose(cv::Mat R, cv::Mat t, cv::Mat P) {
            this->R = R;
            this->t = t;
            this->P = P;
        }
};



class _3DHandler {
    private:
    public:
        _3DHandler(std::string calibPath);
        Intrinsics intrinsics;

        // need to have these functions usable without instantiating the class
        void getRT(const Image &img1, const Image &img2);
        bool getFundamentalMatrix(const std::vector<Matches> &matches, cv::Mat &F);
        bool getFRANSAC(std::vector<Matches> matches, cv::Mat &F, 
        std::pair<float, float> meanVar1, std::pair<float, float> meanVar2, 
                                std::pair<float, float> meanVar3, std::pair<float, float> meanVar4, int iterations, double threshold);
        Pose disambiguateRT(const cv::Mat &E, const cv::Mat &u, const cv::Mat &w, cv::Mat &vt, std::vector<Matches> &matches);
        bool checkDepthPositive(cv::Mat &pnts3D, Pose &pose);
        float unNormalizePoint(float pt, float mean, float var);
        cv::Mat rotateMatrixZ(int rotateAngle);
        std::pair<double, double> getMeanVar(std::vector<double> &vec);

        ~_3DHandler();
};