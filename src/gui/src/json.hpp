#ifndef JSON_HPP
#define JSON_HPP

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>
#include <vector>

using json = nlohmann::json;
namespace fs = std::filesystem;

class Json
{
    public:
        void openJson(std::string json_path);
        std::vector<std::string> getKeys();
        std::vector<std::vector<std::vector<int>>> getMove(std::string move_name);
        void printJson();
        void teste();
        void moveToJson();
        void saveJson(std::vector<std::vector<std::vector<int>>> move, std::string moveName);

        // Json();
        // virtual ~Json();
    
    private:
        std::string jsonPath;
        json j;
        int getMode(std::vector<int> valueList);
        
};

#endif
