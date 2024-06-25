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

        // Json();
        // virtual ~Json();
    
    private:
        json j;
};

#endif
