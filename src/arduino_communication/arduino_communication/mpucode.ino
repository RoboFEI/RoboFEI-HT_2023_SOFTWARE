#include <ArduinoJson.h>
#include <Wire.h>
#include <MPU6050.h>

#define BUFFER_SIZE_RECEIVER 300
#define END_OF_JSON_CHAR '}'

#define ENCODER_PERIOD 50
#define FILTER_SIZE 10

int16_t ax_history[FILTER_SIZE], ay_history[FILTER_SIZE], az_history[FILTER_SIZE];
int16_t gx_history[FILTER_SIZE], gy_history[FILTER_SIZE], gz_history[FILTER_SIZE];

int filterIndex = 0; 

MPU6050 mpu;


char buffer[BUFFER_SIZE_RECEIVER]; // Buffer para armazenar a mensagem
int buffer_index = 0; // Indicador de posição no buffer

long encoder_timer = 0;

float velMotors[2] = {0,0};  // {Right , Left}

// Função para calcular a média das leituras armazenadas
int16_t calculateAverage(int16_t* history) {
  long sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += history[i];
  }
  return sum / FILTER_SIZE;
}

//Usado para as velocidades dos motores
struct Vel {
  struct Linear {
    float x;
    float y;
    float z;
  } linear;
  struct Angular {
    float x;
    float y;
    float z;
  } angular;
};

// Cria uma variável do tipo Vel
Vel vel;

// Esta função será chamada sempre que dados estiverem disponíveis na porta serial
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    buffer[buffer_index] = inChar;
    buffer_index++;
    // Verifica se a mensagem está completa
    if (inChar == END_OF_JSON_CHAR) {
      buffer[buffer_index] = '\0'; // Adiciona o caractere de terminação nulo para tornar o buffer uma string válida
      processMessage();
      buffer_index = 0; // Reseta o buffer
    }
  }
}

// Esta função deserializa a mensagem JSON e processa os dados
void processMessage() {
  StaticJsonDocument<BUFFER_SIZE_RECEIVER> doc_subscription_cmd_vel;
  DeserializationError error = deserializeJson(doc_subscription_cmd_vel, buffer);
  if (error) {

  } else {
    if (doc_subscription_cmd_vel.containsKey("linear_x") && doc_subscription_cmd_vel.containsKey("angular_z")){
      float linear_x = doc_subscription_cmd_vel["linear_x"];
      float angular_z = doc_subscription_cmd_vel["angular_z"];
     
        // Define os valores
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;
    }
  }
}


void setup() {
  Serial.begin(115200);

  // Inicializa a comunicação I2C
  Wire.begin();

  // Inicializa o MPU6050
  Serial.println("Inicializando MPU6050...");
  mpu.initialize();

  // Verifica a conexão
  if (!mpu.testConnection()) {
    Serial.println("Erro ao inicializar MPU6050. Verifique as conexões.");
    while (1); // Para o código caso haja erro
  }

  Serial.println("MPU6050 inicializado com sucesso!");

  
  // Inicializa os buffers de histórico com valores nulos (ou zero)
  memset(ax_history, 0, sizeof(ax_history));
  memset(ay_history, 0, sizeof(ay_history));
  memset(az_history, 0, sizeof(az_history));
  memset(gx_history, 0, sizeof(gx_history));
  memset(gy_history, 0, sizeof(gy_history));
  memset(gz_history, 0, sizeof(gz_history));

    // Define os valores
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

   // Verifica se existem novos dados na porta serial
  if (Serial.available() > 0) {
      serialEvent();
  }

  if(millis() - encoder_timer >=  ENCODER_PERIOD){
    encoder_timer = millis();

    // Leitura de aceleração e giroscópio
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    // Armazena as leituras no histórico de dados
    ax_history[filterIndex] = ax;
    ay_history[filterIndex] = ay;
    az_history[filterIndex] = az;
    gx_history[filterIndex] = gx;
    gy_history[filterIndex] = gy;
    gz_history[filterIndex] = gz;

    // Avança o índice do filtro circular
    filterIndex = (filterIndex + 1) % FILTER_SIZE;

    // Calcula a média das leituras
    int16_t ax_filtered = calculateAverage(ax_history);
    int16_t ay_filtered = calculateAverage(ay_history);
    int16_t az_filtered = calculateAverage(az_history);
    int16_t gx_filtered = calculateAverage(gx_history);
    int16_t gy_filtered = calculateAverage(gy_history);
    int16_t gz_filtered = calculateAverage(gz_history);

    // Cria um documento JSON para enviar as informações
    StaticJsonDocument<300> doc;
    JsonArray array_encoders = doc.createNestedArray("encoders");

    array_encoders.add(ax_filtered);
    array_encoders.add(ay_filtered);
    array_encoders.add(az_filtered);
    array_encoders.add(gx_filtered);
    array_encoders.add(gy_filtered);
    array_encoders.add(gz_filtered);

    // Serializa e envia o documento JSON
    serializeJson(doc, Serial);
    Serial.println();  // Adiciona uma nova linha
   
  }

}
