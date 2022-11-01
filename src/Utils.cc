#include "../include/Utils.hpp"


void parseCalibString(std::string string, cv::Mat &cvMat) {
    std::vector<double> matValues;
    std::string s;
    std::istringstream f(string);

    while(getline(f, s, ' ')) {
        if (s != " ") {
                double d;
                try {
                    d = std::stod(s);
                    matValues.emplace_back(d);

                }
                catch(std::exception &e) {
                    std::cout << e.what() <<std::endl;
                }
        }
    }

    cvMat.at<double>(0,0) = matValues[0]; cvMat.at<double>(0,1) = matValues[1]; cvMat.at<double>(0,2) = matValues[2]; cvMat.at<double>(0,3) = matValues[3]; 
    cvMat.at<double>(1,0) = matValues[4]; cvMat.at<double>(1,1) = matValues[5]; cvMat.at<double>(1,2) = matValues[6]; cvMat.at<double>(1,3) = matValues[7]; 
    cvMat.at<double>(2,0) = matValues[8]; cvMat.at<double>(2,1) = matValues[9]; cvMat.at<double>(2,2) = matValues[10]; cvMat.at<double>(2,3) = matValues[11]; 
    cvMat.at<double>(3,0) = matValues[12]; cvMat.at<double>(3,1) = matValues[13]; cvMat.at<double>(3,2) = matValues[14]; cvMat.at<double>(3,3) = matValues[15]; 


}

std::vector<boost::filesystem::path> getFilesInFolder(const std::string &path) {
    std::vector<boost::filesystem::path> filesInDir;
    std::copy(boost::filesystem::directory_iterator(path), boost::filesystem::directory_iterator(), std::back_inserter(filesInDir));
    std::sort(filesInDir.begin(), filesInDir.end());
    return filesInDir;
}


void getCalibParams(std::string _path, Intrinsics &calib) {
    std::vector<std::string> stringVector;
    std::string line;
    std::ifstream _file(_path);
    int _i = 0;
    if (_file.is_open()) {
        for (int i=0; i<2;i++) {
                cv::Mat cvMat =  cv::Mat(4, 4, CV_64F);
                getline(_file, line);
                parseCalibString(line, cvMat);
                if (i == 0) {
                    Camera left(cvMat);
                    calib.Left = left;
                } else {
                    Camera right(cvMat);
                    calib.Right = right;
                }
            }
            
        }

}