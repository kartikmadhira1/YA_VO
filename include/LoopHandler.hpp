#pragma once
#include <fstream>
#include <jsoncpp/json/json.h>
#include "Image.hpp"
#include "Utils.hpp"

/*
1. Load config file - what sequence, base path, etc.
2. iterate through images
3.  Need methods like next() and iter() for image iteration -> this should be different class or should be in LoopHandler?
*/

class LoopHandler {
    private:
        std::string fullImagesPath;
    public:
        LoopHandler(const std::string &config);
        Image getNextImage();
};

