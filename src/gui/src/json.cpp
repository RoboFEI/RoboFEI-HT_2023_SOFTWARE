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
    
    //added a try cath to validate if the path is valid.
    try{
        j = json::parse(fJson);
    }
    catch(const std:: exception &e){
        std::cerr<<"Erro ao abrir o json"<< e.what()<<std::endl;
    }
    
    //added a try cath to validate if the path is valid.
    try{
        j = json::parse(fJson);
    }
    catch(const std:: exception &e){
        std::cerr<<"Erro ao abrir o json"<< e.what()<<std::endl;
    }
    
    std::cout<<"Debbug: arquivo aberto com sucesso: "<< json_path << std::endl;
    std::cout<<"Debbug: arquivo aberto com sucesso: "<< json_path << std::endl;
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
    std::cout << "position1: " << j[move_name]["position1"] << "\n";
    std::cout << "position1: " << j[move_name]["position1"] << "\n";

    // position = std::vector<int>(j[move_name]["position1"].begin(), j[move_name]["position1"].end());
    // for(auto pos : position)
    // {
    //     std::cout << "position1: " << pos << "\n";
    // }   
    // }   

    int total = j[move_name]["number of movements"];
    int i = 1;

    while(i <= total)
    int total = j[move_name]["number of movements"];
    int i = 1;

    while(i <= total)
    {
        moves.push_back(std::vector<std::vector<int>>(3));

        int current_i = i;

        std::string address_json = "address" + std::to_string(current_i);        
        
        while(j[move_name].contains(address_json) && j[move_name][address_json] != 116) // to get the velocity
        int current_i = i;

        std::string address_json = "address" + std::to_string(current_i);        
        
        while(j[move_name].contains(address_json) && j[move_name][address_json] != 116) // to get the velocity
        {
            std::string id_json = "id" + std::to_string(current_i);
            std::string velocity_json = "velocity" + std::to_string(current_i);
            std::string id_json = "id" + std::to_string(current_i);
            std::string velocity_json = "velocity" + std::to_string(current_i);
            
            if(j[move_name].contains(id_json) && j[move_name].contains(velocity_json)){
                int id = int(j[move_name][id_json]) - 1;

                if(j[move_name][id_json] == 254) velocity = std::vector<int>(20, j[move_name][velocity_json]);
                else velocity[id] = j[move_name][velocity_json];
            }
            
            current_i++;
            address_json = "address" + std::to_string(current_i);
            if(j[move_name].contains(id_json) && j[move_name].contains(velocity_json)){
                int id = int(j[move_name][id_json]) - 1;

                if(j[move_name][id_json] == 254) velocity = std::vector<int>(20, j[move_name][velocity_json]);
                else velocity[id] = j[move_name][velocity_json];
            }
            
            current_i++;
            address_json = "address" + std::to_string(current_i);
        }

        //to get position
        std::string position_json =  "position" + std::to_string(current_i);
        std::string sleep_json = "sleep" + std::to_string(current_i);

        //verify if contains position_json
        if(j[move_name].contains(position_json)){
            position =std::vector<int>(j[move_name][position_json]);
        }
        std::string position_json =  "position" + std::to_string(current_i);
        std::string sleep_json = "sleep" + std::to_string(current_i);

        //verify if contains position_json
        if(j[move_name].contains(position_json)){
            position =std::vector<int>(j[move_name][position_json]);
        }

        //verify if contains sleep_json
        if(j[move_name].contains(sleep_json)){
            float aux = j[move_name][sleep_json];
            sleep[0] = (int)(aux*1000);
        }
        //verify if contains sleep_json
        if(j[move_name].contains(sleep_json)){
            float aux = j[move_name][sleep_json];
            sleep[0] = (int)(aux*1000);
        }
        
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

        i = current_i + 1;


        i = current_i + 1;

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
    for (auto it = j.begin(); it != j.end(); ++it)
    {
         std::cout << "key: " << it.key() << '\n';
    }
    for (auto it = j.begin(); it != j.end(); ++it)
    {
         std::cout << "key: " << it.key() << '\n';
    }

    std::cout << "key: " << j["Right Kick"]["number of movements"] << '\n';
    std::cout << "key: " << j["Right Kick"]["number of movements"] << '\n';
}

// Função de pós-processamento
void postProcessFile(const std::string& path) {
    // Abrir o arquivo original
    std::ifstream in(path);
    std::string content((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());
    in.close();

    // Adiciona uma linha em branco antes e depois de todo o conteúdo JSON
    content = "\n" + content + "\n";

    // Substitui arrays formatados em múltiplas linhas por arrays em uma linha
    std::regex re(R"(\[\s*(\d+(?:,\s*\d+)*)\s*\])");
    content = std::regex_replace(content, re, "[$1]");

    // Substitui múltiplos espaços por um único espaço dentro dos arrays
    content = std::regex_replace(content, std::regex("\\s+"), " ");

    // Reabre o arquivo para escrever o conteúdo modificado
    std::ofstream out(path);
    out << content;
    out.close();
}

/* void Json::teste()
{
    std::cout << "teste";
} */


//used for debug the json.cpp file

//compile using g++ json.cpp on terminal

/* 
int main(){

    Json j;
    std::string path = "/home/robo/RoboFEI-HT_2023_SOFTWARE/src/control/Data/motion1.json";
    j.openJson(path);
    j.printJson();
    j.getMove("Right Kick");
} */