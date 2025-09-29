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

void renumberJsonFieldsInFile(const std::string& path, const std::string& moveName, int startFrom)
{
    // 1. Abre o arquivo para leitura
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) {
        std::cerr << "Erro ao abrir arquivo para renumeracao: " << path << std::endl;
        return;
    }
    
    // 2. Lê o JSON completo do arquivo
    json fullJson;
    try {
        inputFile >> fullJson;
    } catch (const std::exception& e) {
        std::cerr << "Erro ao parsear JSON: " << e.what() << std::endl;
        inputFile.close();
        return;
    }
    inputFile.close();
    
    // 3. Verifica se o movimento existe
    if (!fullJson.contains(moveName)) {
        std::cerr << "Movimento '" << moveName << "' nao encontrado no JSON" << std::endl;
        return;
    }
    
    // 4. Acessa o movimento específico
    json& moveJson = fullJson[moveName];
    
    // 5. Verifica se tem o campo "number of movements"
    if (!moveJson.contains("number of movements")) {
        std::cerr << "Campo 'number of movements' nao encontrado" << std::endl;
        return;
    }
    
    int totalSuffix = moveJson["number of movements"].get<int>();
    
    // 6. Renumera os campos (sua lógica original)
    std::vector<std::string> allFields;
    for (auto& [key, value] : moveJson.items()) {
        allFields.push_back(key);
    }
    
    for (const auto& field : allFields) {
        for (int i = totalSuffix; i >= startFrom; --i) {
            std::string oldSuffix = std::to_string(i);
            std::string newSuffix = std::to_string(i + 1);
            
            if (field.find(oldSuffix) != std::string::npos) {
                std::string newField = field;
                size_t pos = newField.find(oldSuffix);
                newField.replace(pos, oldSuffix.length(), newSuffix);
                
                moveJson[newField] = moveJson[field];
                moveJson.erase(field);
                break;
            }
        }
    }
    
    // 7. Atualiza o número total de movimentos
    moveJson["number of movements"] = totalSuffix + 1;
    
    // 8. Abre o arquivo para escrita (sobrescreve)
    std::ofstream outputFile(path);
    if (!outputFile.is_open()) {
        std::cerr << "Erro ao abrir arquivo para escrita: " << path << std::endl;
        return;
    }
    
    // 9. Escreve o JSON modificado de volta no arquivo
    try {
        outputFile << fullJson.dump(4) << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Erro ao escrever JSON: " << e.what() << std::endl;
    }
    
    outputFile.close();
    
    std::cout << "Campos renumerados com sucesso no arquivo: " << path << std::endl;
}

// 1. Inserção no final (cenário mais fácil)
void insertStepAtEnd(json& moveJson, int copyFromSuffix) {

    int totalSuffix = moveJson["number of movements"].get<int>();
    int newSuffix = totalSuffix + 1;
    
    // Copia do último step
    moveJson["address" + std::to_string(newSuffix)] = 116;
    
    if (moveJson.contains("position" + std::to_string(copyFromSuffix))) {
        moveJson["position" + std::to_string(newSuffix)] = 
            moveJson["position" + std::to_string(copyFromSuffix)];
    }
    
    if (moveJson.contains("sleep" + std::to_string(copyFromSuffix))) {
        moveJson["sleep" + std::to_string(newSuffix)] = 
            moveJson["sleep" + std::to_string(copyFromSuffix)];
    }
    
    moveJson["number of movements"] = newSuffix;
}

// 2. Inserção no meio (cenário complexo)
void insertStepInMiddle(json& moveJson, int insertAfterSuffix)
{
    // ✅ CORREÇÃO: Converter JSON para int
    int totalSuffix = moveJson["number of movements"].get<int>();
    
    // 1. Primeiro: desloca todos os steps para frente NA ORDEM CORRETA
    // (do último para o primeiro, mas na ordem crescente de renomeação)
    for (int i = totalSuffix; i >= insertAfterSuffix + 1; --i) {
        int currentSuffix = i;
        int newSuffix = i + 1;
        
        // Move address
        std::string currentAddrKey = "address" + std::to_string(currentSuffix);
        std::string newAddrKey = "address" + std::to_string(newSuffix);
        if (moveJson.contains(currentAddrKey)) {
            moveJson[newAddrKey] = moveJson[currentAddrKey];
            moveJson.erase(currentAddrKey);
        }
        
        // Move position
        std::string currentPosKey = "position" + std::to_string(currentSuffix);
        std::string newPosKey = "position" + std::to_string(newSuffix);
        if (moveJson.contains(currentPosKey)) {
            moveJson[newPosKey] = moveJson[currentPosKey];
            moveJson.erase(currentPosKey);
        }
        
        // Move sleep
        std::string currentSleepKey = "sleep" + std::to_string(currentSuffix);
        std::string newSleepKey = "sleep" + std::to_string(newSuffix);
        if (moveJson.contains(currentSleepKey)) {
            moveJson[newSleepKey] = moveJson[currentSleepKey];
            moveJson.erase(currentSleepKey);
        }
        
        // Move velocity (se existir)
        std::string currentVelKey = "velocity" + std::to_string(currentSuffix);
        std::string newVelKey = "velocity" + std::to_string(newSuffix);
        if (moveJson.contains(currentVelKey)) {
            moveJson[newVelKey] = moveJson[currentVelKey];
            moveJson.erase(currentVelKey);
        }
        
        // Move id (se existir)
        std::string currentIdKey = "id" + std::to_string(currentSuffix);
        std::string newIdKey = "id" + std::to_string(newSuffix);
        if (moveJson.contains(currentIdKey)) {
            moveJson[newIdKey] = moveJson[currentIdKey];
            moveJson.erase(currentIdKey);
        }
    }
    
    // 2. Segundo: cria o novo step copiando do anterior
    int newSuffix = insertAfterSuffix + 1;
    moveJson["address" + std::to_string(newSuffix)] = 116;
    
    if (moveJson.contains("position" + std::to_string(insertAfterSuffix))) {
        moveJson["position" + std::to_string(newSuffix)] = 
            moveJson["position" + std::to_string(insertAfterSuffix)];
    }
    
    if (moveJson.contains("sleep" + std::to_string(insertAfterSuffix))) {
        moveJson["sleep" + std::to_string(newSuffix)] = 
            moveJson["sleep" + std::to_string(insertAfterSuffix)];
    }
    
    // 3. Atualiza o número total de movimentos
    moveJson["number of movements"] = totalSuffix + 1;
}

// 3. Primeiro step (usa posição stand still)
void createFirstStep(json& moveJson) {
    // Define uma posição padrão stand still (ajuste conforme seu robô)
    std::vector<int> standStillPosition = {
        2150, 2004, 2148, 1918, 1748, 2348,  // Junta 1-6
        2055, 2041, 2127, 1979, 1731, 2365,  // Junta 7-12  
        2662, 1434, 1701, 2332, 2114, 1986   // Junta 13-18
    };
    
    moveJson["number of movements"] = 1;
    moveJson["address1"] = 116;
    moveJson["position1"] = standStillPosition;
    moveJson["sleep1"] = 1.0;
}

// Função principal que decide qual cenário usar
void insertNewStep(json& moveJson, int afterStepNumber) {
    int totalSuffix = moveJson["number of movements"];
    
    // Conta quantos steps de posição existem
    int posStepsCount = 0;
    for (int i = 1; i <= totalSuffix; ++i) {
        std::string addrKey = "address" + std::to_string(i);
        if (moveJson.contains(addrKey) && moveJson[addrKey] == 116) {
            posStepsCount++;
        }
    }
    
    if (posStepsCount == 0) {
        // Cenário 1: Não há steps anteriores
        createFirstStep(moveJson);
    } else if (afterStepNumber == posStepsCount) {
        // Cenário 2: Inserção no final
        // Encontra o último sufixo de posição
        int lastPosSuffix = 0;
        for (int i = totalSuffix; i >= 1; --i) {
            std::string addrKey = "address" + std::to_string(i);
            if (moveJson.contains(addrKey) && moveJson[addrKey] == 116) {
                lastPosSuffix = i;
                break;
            }
        }
        insertStepAtEnd(moveJson, lastPosSuffix);
    } else {
        // Cenário 3: Inserção no meio
        // Encontra o sufixo real correspondente ao step
        int targetSuffix = 0;
        int posCount = 0;
        for (int i = 1; i <= totalSuffix; ++i) {
            std::string addrKey = "address" + std::to_string(i);
            if (moveJson.contains(addrKey) && moveJson[addrKey] == 116) {
                posCount++;
                if (posCount == afterStepNumber) {
                    targetSuffix = i;
                    break;
                }
            }
        }
        insertStepInMiddle(moveJson, targetSuffix);
    }
}

int countPositionSteps(const json& moveJson)
{
    int totalSuffix = moveJson["number of movements"].get<int>();
    int count = 0;
    
    for (int i = 1; i <= totalSuffix; ++i) {
        std::string addrKey = "address" + std::to_string(i);
        if (moveJson.contains(addrKey) && moveJson[addrKey].get<int>() == 116) {
            count++;
        }
    }
    return count;
}

// Versão para arquivo
void insertNewStepInFile(const std::string& path, const std::string& moveName, int afterStepNumber) {
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) return;
    
    json fullJson;
    inputFile >> fullJson;
    inputFile.close();
    
    if (!fullJson.contains(moveName)) return;
    
    json& moveJson = fullJson[moveName];
    insertNewStep(moveJson, afterStepNumber);
    
    std::ofstream outputFile(path);
    if (outputFile.is_open()) {
        outputFile << fullJson.dump(4) << std::endl;
        outputFile.close();
    }
    
    // Opcional: formata o arquivo
    postProcessFile(path);
}

//implementar
/*

#include "json.hpp"
#include <iostream>

// ==================== FUNÇÕES AUXILIARES ====================

int countPositionSteps(const json& moveJson) {
    if (!moveJson.contains("number of movements")) return 0;
    
    int totalSuffix = moveJson["number of movements"].get<int>();
    int count = 0;
    
    for (int i = 1; i <= totalSuffix; ++i) {
        std::string addrKey = "address" + std::to_string(i);
        if (moveJson.contains(addrKey) && moveJson[addrKey].get<int>() == 116) {
            count++;
        }
    }
    return count;
}

void createFirstStep(json& moveJson) {
    std::vector<int> standStillPosition = {
        2048, 2048, 2048, 2048, 2048, 2048,
        2048, 2048, 2048, 2048, 2048, 2048,
        2048, 2048, 2048, 2048, 2048, 2048
    };
    
    moveJson["number of movements"] = 1;
    moveJson["address1"] = 116;
    moveJson["position1"] = standStillPosition;
    moveJson["sleep1"] = 1.0;
}

void insertStepAtEnd(json& moveJson, int copyFromSuffix) {
    int totalSuffix = moveJson["number of movements"].get<int>();
    int newSuffix = totalSuffix + 1;
    
    moveJson["address" + std::to_string(newSuffix)] = 116;
    
    std::string posKey = "position" + std::to_string(copyFromSuffix);
    if (moveJson.contains(posKey)) {
        moveJson["position" + std::to_string(newSuffix)] = moveJson[posKey];
    }
    
    std::string sleepKey = "sleep" + std::to_string(copyFromSuffix);
    if (moveJson.contains(sleepKey)) {
        moveJson["sleep" + std::to_string(newSuffix)] = moveJson[sleepKey];
    }
    
    moveJson["number of movements"] = newSuffix;
}

void insertStepInMiddle(json& moveJson, int insertAfterSuffix) {
    int totalSuffix = moveJson["number of movements"].get<int>();
    
    // 1. Primeiro criar um novo JSON com a estrutura correta
    json newMoveJson;
    newMoveJson["number of movements"] = totalSuffix + 1;
    
    // 2. Copiar todos os campos, inserindo o novo step no lugar certo
    for (int i = 1; i <= totalSuffix; ++i) {
        // Determinar o novo sufixo
        int newSuffix = (i <= insertAfterSuffix) ? i : i + 1;
        
        // Copiar address
        std::string oldAddrKey = "address" + std::to_string(i);
        if (moveJson.contains(oldAddrKey)) {
            newMoveJson["address" + std::to_string(newSuffix)] = moveJson[oldAddrKey];
        }
        
        // Copiar position
        std::string oldPosKey = "position" + std::to_string(i);
        if (moveJson.contains(oldPosKey)) {
            newMoveJson["position" + std::to_string(newSuffix)] = moveJson[oldPosKey];
        }
        
        // Copiar sleep
        std::string oldSleepKey = "sleep" + std::to_string(i);
        if (moveJson.contains(oldSleepKey)) {
            newMoveJson["sleep" + std::to_string(newSuffix)] = moveJson[oldSleepKey];
        }
        
        // Copiar velocity (se existir)
        std::string oldVelKey = "velocity" + std::to_string(i);
        if (moveJson.contains(oldVelKey)) {
            newMoveJson["velocity" + std::to_string(newSuffix)] = moveJson[oldVelKey];
        }
        
        // Copiar id (se existir)
        std::string oldIdKey = "id" + std::to_string(i);
        if (moveJson.contains(oldIdKey)) {
            newMoveJson["id" + std::to_string(newSuffix)] = moveJson[oldIdKey];
        }
    }
    
    // 3. Inserir o novo step (cópia do step anterior)
    int newStepSuffix = insertAfterSuffix + 1;
    newMoveJson["address" + std::to_string(newStepSuffix)] = 116;
    
    std::string copyPosKey = "position" + std::to_string(insertAfterSuffix);
    if (moveJson.contains(copyPosKey)) {
        newMoveJson["position" + std::to_string(newStepSuffix)] = moveJson[copyPosKey];
    }
    
    std::string copySleepKey = "sleep" + std::to_string(insertAfterSuffix);
    if (moveJson.contains(copySleepKey)) {
        newMoveJson["sleep" + std::to_string(newStepSuffix)] = moveJson[copySleepKey];
    }
    
    // 4. Substituir o moveJson pelo novo
    moveJson = newMoveJson;
}

void insertNewStep(json& moveJson, int afterStepNumber) {
    int totalSuffix = moveJson["number of movements"].get<int>();
    int posStepsCount = countPositionSteps(moveJson);
    
    if (posStepsCount == 0) {
        // Cenário 1: Não há steps anteriores
        createFirstStep(moveJson);
    } else {
        // Encontrar o sufixo real correspondente ao step
        int targetSuffix = 0;
        int posCount = 0;
        
        for (int i = 1; i <= totalSuffix; ++i) {
            std::string addrKey = "address" + std::to_string(i);
            if (moveJson.contains(addrKey) && moveJson[addrKey].get<int>() == 116) {
                posCount++;
                if (posCount == afterStepNumber) {
                    targetSuffix = i;
                    break;
                }
            }
        }
        
        if (targetSuffix == 0) {
            std::cerr << "Não encontrou sufixo para o step " << afterStepNumber << std::endl;
            return;
        }
        
        if (afterStepNumber == posStepsCount) {
            // Cenário 2: Inserção no final
            insertStepAtEnd(moveJson, targetSuffix);
        } else {
            // Cenário 3: Inserção no meio
            insertStepInMiddle(moveJson, targetSuffix);
        }
    }
}

void insertNewStepInFile(const std::string& path, const std::string& moveName, int afterStepNumber) {
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) {
        std::cerr << "Erro ao abrir arquivo: " << path << std::endl;
        return;
    }
    
    json fullJson;
    inputFile >> fullJson;
    inputFile.close();
    
    if (!fullJson.contains(moveName)) {
        std::cerr << "Movimento não encontrado: " << moveName << std::endl;
        return;
    }
    
    json& moveJson = fullJson[moveName];
    insertNewStep(moveJson, afterStepNumber);
    
    std::ofstream outputFile(path);
    if (outputFile.is_open()) {
        outputFile << fullJson.dump(4) << std::endl;
        outputFile.close();
    }
    
    // Pós-processamento opcional
    postProcessFile(path);
}

void postProcessFile(const std::string& path) {
    // Sua implementação existente de pós-processamento
    std::ifstream inputFile(path);
    if (!inputFile.is_open()) return;
    
    json j;
    inputFile >> j;
    inputFile.close();
    
    std::ofstream outputFile(path);
    if (outputFile.is_open()) {
        outputFile << j.dump(4) << std::endl;
        outputFile.close();
    }
}


*/