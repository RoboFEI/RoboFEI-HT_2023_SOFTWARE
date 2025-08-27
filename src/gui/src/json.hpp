#ifndef JSON_HPP
#define JSON_HPP

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>
#include <vector>
#include <regex>

using json = nlohmann::ordered_json;
namespace fs = std::filesystem;

class Json
{
    public:
        void openJson(std::string json_path);
        std::vector<std::string> getKeys();
        std::vector<std::vector<std::vector<int>>> getMove(std::string move_name);
        void printJson();
        void teste();

        bool contains(const std::string& move_name) const{
            return j.contains(move_name);
        }

        json& getMoveJson(const std::string& move_name){
            return j[move_name];
        }

        void setMoveValue(const std::string& move_name, const std::string& key, const json& value){
            j[move_name][key] = value;
        }

        void saveJson(const std::string& path){
            std::ofstream file(path);
            if(file.is_open()){
                // Salva com indentação mínima, sem quebra de linha em arrays.
                file << j.dump(4) << std::endl;
                file.close();
            }
        }
        json& getJson();

        

        
    
    private:
        json j;
};

// Função de pós-processamento
void postProcessFile(const std::string& path);
 
#endif