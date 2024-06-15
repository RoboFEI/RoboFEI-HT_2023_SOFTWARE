#include "json.hpp"

std::string folder_path2 = fs::current_path();

// Json::Json()
// {
// }

// Json::~Json()
// {
// }

int Json::openJson(std::string json_path)
{
    std::ifstream fJson(json_path);
    j = json::parse(fJson);

}

std::vector<std::string> Json::getKeys()
{
    std::vector<std::string> keys;
    for (auto it = j.begin(); it != j.end(); ++it)
    {
        keys.push_back(it.key());
    }
    return keys;
}

void Json::getMove(std::string move_name)
{
    std::cout << "loading move: " << move_name << "\n";
    std::cout << "number of movements: " << j[move_name]["number of movements"] << "\n";

    for(int i=1; i<j[move_name]["number of movements"]; i++)
    {
        std::string address = "address" + std::to_string(i);
        std::cout << "Address " << j[move_name][address] << "\n";
    }
}


void Json::printJson()
{
    // for (auto it = j.begin(); it != j.end(); ++it)
    // {
    //     std::cout << "key: " << it.key() << '\n';
    // }

    std::cout << "key: " << j["Stand Still"]["number of movements"] << '\n';
}

void Json::teste()
{
    std::cout << "teste";
}