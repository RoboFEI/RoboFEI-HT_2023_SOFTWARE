#include "json.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <regex>
#include <filesystem>
#include <typeinfo>
#include <string>

namespace fs = std::filesystem;

std::string folder_path2 = fs::current_path();

void Json::openJson(std::string json_path)
{
    std::ifstream fJson(json_path);
    try {
        j = json::parse(fJson);
        std::cout << "Debug: arquivo aberto com sucesso: " << json_path << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Erro ao abrir o json: " << e.what() << std::endl;
    }
}

json& Json::getJson() {
    return j;
}

std::vector<std::string> Json::getKeys()
{
    std::vector<std::string> keys;  
    for (auto it = j.begin(); it != j.end(); ++it) {
        keys.push_back(it.key());
    }
    return keys;
}

std::vector<std::vector<std::vector<int>>> Json::getMove(std::string move_name)
{
    std::vector<std::vector<std::vector<int>>> moves;
    std::vector<int> position;
    std::vector<int> velocity(18, 32);
    std::vector<int> sleep(1, 0);
    int move = 0;

    std::cout << "Loading move: " << move_name << "\n";
    std::cout << "Number of movements: " << j[move_name]["number of movements"] << "\n";

    int total = j[move_name]["number of movements"];
    int i = 1;

    while (i <= total)
    {
        int current_i = i;

        std::string address_json = "address" + std::to_string(current_i);

        // Processa blocos de configuração (id/velocity)
        while (j[move_name].contains(address_json) && j[move_name][address_json] != 116)
        {
            std::string id_json = "id" + std::to_string(current_i);
            std::string velocity_json = "velocity" + std::to_string(current_i);

            if (j[move_name].contains(id_json) && j[move_name].contains(velocity_json)) {
                int id = int(j[move_name][id_json]) - 1;

                if (j[move_name][id_json] == 254)
                    velocity = std::vector<int>(20, j[move_name][velocity_json]);
                else if (id >= 0 && id < static_cast<int>(velocity.size()))
                    velocity[id] = j[move_name][velocity_json];
            }

            current_i++;
            address_json = "address" + std::to_string(current_i);
        }

        // Pega positionX se existir
        std::string position_json = "position" + std::to_string(current_i);
        if (j[move_name].contains(position_json) && j[move_name][position_json].is_array()) {
            position = std::vector<int>(j[move_name][position_json]);
        } else {
            position.clear();  // garante que não estamos usando lixo
        }

        // Pega sleepX
        std::string sleep_json = "sleep" + std::to_string(current_i);
        if (j[move_name].contains(sleep_json)) {
            float aux = j[move_name][sleep_json];
            sleep[0] = static_cast<int>(aux * 1000);
        }

        // Debug
        if (!position.empty()) {
            for (auto pos : position) std::cout << pos << " ";
            std::cout << "\n";
        }

        for (auto vel : velocity) std::cout << vel << " ";
        std::cout << "\n";

        for (auto delay : sleep) std::cout << delay << " ";
        std::cout << "\n\n";

        // Adiciona movimento se tiver posição
        if (!position.empty()) {
            moves.push_back(std::vector<std::vector<int>>(3));
            moves[move][0] = position;
            moves[move][1] = velocity;
            moves[move][2] = sleep;
            move++;
        }

        i = current_i + 1;
        position.clear();  // limpa para o próximo passo
    }

    // Print final
    for (const auto& i : moves)
    {
        std::cout << "Position: ";
        for (auto j : i[0]) std::cout << j << " ";
        std::cout << "\nVelocity: ";
        for (auto j : i[1]) std::cout << j << " ";
        std::cout << "\nSleep: ";
        for (auto j : i[2]) std::cout << j << " ";
        std::cout << "\n\n";
    }

    return moves;
}

void Json::printJson()
{
    for (auto it = j.begin(); it != j.end(); ++it) {
        std::cout << "Key: " << it.key() << '\n';
    }

    std::cout << "Right Kick - number of movements: " << j["Right Kick"]["number of movements"] << '\n';
}



void postProcessFile(const std::string& path)
{
    std::ifstream in(path);
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();

    // Adiciona quebra no começo e fim
    content = "\n" + content + "\n";

    // 1. Compacta arrays em linha única
    std::regex array_regex(R"(\[\s*((?:\d+\s*,\s*)*\d+)\s*\])");
    std::smatch match;

    std::string processed;
    std::string::const_iterator searchStart(content.cbegin());

    while (std::regex_search(searchStart, content.cend(), match, array_regex)) {
        processed.append(match.prefix());

        std::string inside = match[1].str();
        inside = std::regex_replace(inside, std::regex(R"([\n\r\t]+)"), " ");
        inside = std::regex_replace(inside, std::regex(" {2,}"), " ");

        processed.append("[" + inside + "]");
        searchStart = match.suffix().first;
    }

    processed.append(searchStart, content.cend());

    // 2. Adiciona quebra de linha DEPOIS de velocityN e sleepN sem vírgula extra
    processed = std::regex_replace(processed, std::regex(R"(("velocity\d+"\s*:\s*\d+))"), "$1");
    processed = std::regex_replace(processed, std::regex(R"(("sleep\d+"\s*:\s*[\d.]+))"), "$1");

    // 3. Quebra de linha depois deles (sem adicionar vírgula)
    processed = std::regex_replace(processed, std::regex(R"(("velocity\d+":\s*\d+))"), "$1\n        ");
    processed = std::regex_replace(processed, std::regex(R"(("sleep\d+":\s*[\d.]+))"), "$1\n        ");

    // 4. Remove vírgulas duplicadas
    processed = std::regex_replace(processed, std::regex(R"(,\s*,)"), ",");
    processed = std::regex_replace(processed, std::regex(R"(,\s*})"), " }");  // remove vírgula antes de fechar bloco

    // 5. Corrige vírgula final no último campo de objetos
    processed = std::regex_replace(processed, std::regex(R"(,\s*\n\s*})"), "\n    }");

    // 6. Corrige vírgulas duplas deixadas por erro de bloco
    processed = std::regex_replace(processed, std::regex(R"(,\s*,)"), ",");

    std::ofstream out(path);
    out << processed;
    out.close();
}