#include "json.hpp"

#include <typeinfo>

std::string folder_path2 = fs::current_path();

// Json::Json()
// {
// }

// Json::~Json()
// {
// }

void Json::openJson(std::string json_path)
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

std::vector<std::vector<std::vector<int>>> Json::getMove(std::string move_name)
{
    std::vector<std::vector<std::vector<int>>> moves;
    std::vector<int> position;
    std::vector<int> velocity = std::vector<int>(18, 32);
    std::vector<int> sleep = std::vector<int>(1, 0);
    int move = 0;

    std::cout << "loading move: " << move_name << "\n";
    std::cout << "number of movements: " << j[move_name]["number of movements"] << "\n";
    // std::cout << "position1: " << j[move_name]["position1"] << "\n";

    // position = std::vector<int>(j[move_name]["position1"].begin(), j[move_name]["position1"].end());
    // for(auto pos : position)
    // {
    //     std::cout << "position1: " << pos << "\n";
    // }

    

    for(int i=1; i<=j[move_name]["number of movements"]; i++)
    {
        moves.push_back(std::vector<std::vector<int>>(3));

        std::string address_json = "address" + std::to_string(i);

        while(j[move_name][address_json] != 116) // to get the velocity
        {
            std::string id_json = "id" + std::to_string(i);
            std::string velocity_json = "velocity" + std::to_string(i);
            
            int id = int(j[move_name][id_json]) - 1;
            if(j[move_name][id_json] == 254) velocity = std::vector<int>(20, j[move_name][velocity_json]);
            else velocity[id] = j[move_name][velocity_json];

            i++;
            address_json = "address" + std::to_string(i);
        }

        //to get position
        std::string position_json =  "position" + std::to_string(i);
        position = std::vector<int>(j[move_name][position_json]);

        std::string sleep_json = "sleep" + std::to_string(i);
        float aux = j[move_name][sleep_json];
        sleep[0] = (int)(aux*1000);
        
        //Debug
        for(auto pos : position)
        {
            std::cout << pos << " ";
        }
        std::cout << "\n";

        for(auto vel : velocity)
        {
            std::cout << vel << " ";
        }
        std::cout << "\n";

        for(auto delay : sleep)
        {
            std::cout << delay << " ";
        }
        std::cout << "\n";
        std::cout << "\n";
        std::cout << "\n";

        moves[move][0] = position;
        moves[move][1] = velocity;
        moves[move][2] = sleep;
        move++;
    }

    

    for(auto i : moves)
    {
        std::cout << "position: ";
        for(auto j : i[0]) std::cout << j << " ";
        std::cout << "\n";

        std::cout << "velocity: ";
        for(auto j : i[1]) std::cout << j << " ";
        std::cout << "\n";

        std::cout << "sleep: ";
        for(auto j : i[2]) std::cout << j << " ";
        std::cout << "\n";
        std::cout << "\n";
        
    }

    return moves;
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
