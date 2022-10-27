#include "../include/LoopHandler.hpp"




LoopHandler::LoopHandler(const std::string &config) {
    std::ifstream(config)
    Json::Reader reader;
    Json::Value value;
    reader.parse(ifs, obj);
    
}