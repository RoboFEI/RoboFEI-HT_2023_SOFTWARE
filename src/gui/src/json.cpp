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

    int lower_limiter = 3;
    int upper_limiter = int(j[move_name]["number of movements"]) - 1;
    if(move_name == "Stand Still")
    {
        lower_limiter = 1;
        upper_limiter = j[move_name]["number of movements"];
    } 

    for(int i=lower_limiter; i<=upper_limiter; i++)
    {
        moves.push_back(std::vector<std::vector<int>>(3));

        std::string address_json = "address" + std::to_string(i);

        while(j[move_name][address_json] != 116) // to get the velocity
        {
            std::string id_json = "id" + std::to_string(i);
            std::string velocity_json = "velocity" + std::to_string(i);
            
            int id = int(j[move_name][id_json]) - 1;
            if(j[move_name][id_json] == 254) velocity = std::vector<int>(18, j[move_name][velocity_json]);
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

    return moves;
}

void Json::saveJson(std::vector<std::vector<std::vector<int>>> move, std::string moveName)
{
    j[moveName].clear();

    std::vector<int> lastVel = move[0][1];
    std::vector<int> newVel;
    int numOfMoves = 1;

    if(moveName != "Stand Still")
    {
        j[moveName] = j["Stand Still"];
        lastVel = std::vector<int> (18, j["Stand Still"]["velocity1"]);
        numOfMoves = 3;
    }

    std::string address = "address";
    std::string position = "position";
    std::string id = "id";
    std::string vel = "velocity";
    std::string sleep = "sleep";
    
    for(int i=0; i<move.size(); i++)
    {
        newVel = move[i][1];
        if(lastVel != newVel || numOfMoves == 1)
        {
            if(std::count(newVel.begin(), newVel.end(), newVel[0]) == newVel.size()) // Todas as veocidades são diferentes
            {
                j[moveName][address+std::to_string(numOfMoves)] = 112;
                j[moveName][id+std::to_string(numOfMoves)] = 254;
                j[moveName][vel+std::to_string(numOfMoves)] = newVel[0];
                numOfMoves++;
            }
            else
            {
                if(getMode(newVel) == getMode(lastVel)) // Maioria das velocidades são iguais
                {
                    for(int i=0; i<newVel.size(); i++)
                    {
                        if(newVel[i] != lastVel[i])
                        {
                            j[moveName][address+std::to_string(numOfMoves)] = 112;
                            j[moveName][id+std::to_string(numOfMoves)] = i+1;
                            j[moveName][vel+std::to_string(numOfMoves)] = newVel[i];
                            numOfMoves++;
                        }
                    }
                }
                else 
                {
                    int newMode = getMode(newVel);
                    j[moveName][address+std::to_string(numOfMoves)] = 112;
                    j[moveName][id+std::to_string(numOfMoves)] = 254;
                    j[moveName][vel+std::to_string(numOfMoves)] = newMode;
                    numOfMoves++;

                    for(int i=0; i<newVel.size(); i++)
                    {
                        if(newVel[i] != newMode)
                        {
                            j[moveName][address+std::to_string(numOfMoves)] = 112;
                            j[moveName][id+std::to_string(numOfMoves)] = i+1;
                            j[moveName][vel+std::to_string(numOfMoves)] = newVel[i];
                            numOfMoves++;
                        }
                    }
                }
            }

        }

        j[moveName][address+std::to_string(numOfMoves)] = 116;
        j[moveName][position+std::to_string(numOfMoves)] = move[i][0];
        j[moveName][sleep+std::to_string(numOfMoves)] = move[i][2][0]/1000.0;
        numOfMoves++;

        lastVel = newVel;

    }

    if(moveName != "Stand Still")
    {

        j[moveName][address+std::to_string(numOfMoves)] = 116;
        j[moveName][position+std::to_string(numOfMoves)] = j["Stand Still"]["position2"];
        j[moveName][sleep+std::to_string(numOfMoves)] = j["Stand Still"]["sleep2"];
        numOfMoves++;
    }

    j[moveName]["number of movements"] = numOfMoves-1;

    std::cout << j << std::endl;

    // j.dump(4);

    // // j["Stand Still"];

    // fTeste >> saida;    
    std::ofstream file("motion_teste.json");

    file << std::setw(4) << j;

}

int Json::getMode(std::vector<int> valueList)
{
    int mode = valueList[0];
    int qtd = std::count(valueList.begin(), valueList.end(), valueList[0]);

    for(int item : valueList)
    {
        if(std::count(valueList.begin(), valueList.end(), item) > qtd)
        {
            qtd = std::count(valueList.begin(), valueList.end(), item);
            mode = item;
        }
    }
    return mode;
}

void Json::printJson()
{
    // for (auto it = j.begin(); it != j.end(); ++it)
    // {
    //     std::cout << "key: " << it.key() << '\n';
    // }

    std::cout << "key: " << j["Stand Still"]["number of movements"] << '\n';
}

void Json::moveToJson()
{
    
}

void Json::teste()
{
    std::cout << "teste";
}
