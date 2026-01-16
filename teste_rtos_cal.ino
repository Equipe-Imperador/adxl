#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// --- Estruturas e Definições ---
struct DataPacket {
  uint32_t timestamp;
  float x1, y1, z1; // Sensor 0x1D (Calibrado)
  float x2, y2, z2; // Sensor 0x53 (Calibrado)
};

#define SD_CS 5
QueueHandle_t dataQueue;
File dataFile;
char filename[32];

// Instâncias dos sensores
Adafruit_ADXL345_Unified accel1D = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel53 = Adafruit_ADXL345_Unified(0x53);

// --- Matrizes de Calibração (Fieis ao seu modelo inicial) ---
const float Ainv1D[3][3] = {{0.952875, 0.006993, 0.003366}, {0.006993, 0.952172, -0.000777}, {0.003366, -0.000777, 0.987901}};
const float b1D[3]       = {0.094440, -0.361731, 0.911032};

const float Ainv53[3][3] = {{0.970592, -0.048166, 0.008696}, {-0.048166, 0.988076, -0.025318}, {0.008696, -0.025318, 0.984936}};
const float b53[3]       = {0.931123, -0.218200, 0.350909};

// Protótipos das tarefas
void TaskSensor(void *pvParameters);
void TaskSD(void *pvParameters);

// --- Função de Calibração (Matematicamente idêntica à original) ---
void aplicarCalibracao(sensors_event_t &event, const float Ainv[3][3], const float b[3], float &ox, float &oy, float &oz) {
    // f = h - b
    float f0 = event.acceleration.x - b[0];
    float f1 = event.acceleration.y - b[1];
    float f2 = event.acceleration.z - b[2];
    
    // acalibrado = Ainv * f
    ox = Ainv[0][0]*f0 + Ainv[0][1]*f1 + Ainv[0][2]*f2;
    oy = Ainv[1][0]*f0 + Ainv[1][1]*f1 + Ainv[1][2]*f2;
    oz = Ainv[2][0]*f0 + Ainv[2][1]*f1 + Ainv[2][2]*f2;
}

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000); // I2C a 400kHz

  // 1. Inicialização dos Sensores
  if(!accel1D.begin()) { Serial.println("Erro ADXL 0x1D!"); while(1); }
  if(!accel53.begin()) { Serial.println("Erro ADXL 0x53!"); while(1); }
  
  accel1D.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel1D.setRange(ADXL345_RANGE_16_G);
  accel53.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel53.setRange(ADXL345_RANGE_16_G);

  // 2. Inicialização do SD Card e Nome de Arquivo Dinâmico
  if (!SD.begin(SD_CS)) { 
    Serial.println("Erro SD!"); 
    while(1); 
  }

  uint32_t t_us = micros();
  sprintf(filename, "/log_%u.csv", t_us); // Nome baseado no tempo de boot
  
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    Serial.printf("Log iniciado: %s\n", filename);
    dataFile.println("us;x1;y1;z1;x2;y2;z2"); // Cabeçalho CSV
  } else {
    Serial.println("Erro ao criar arquivo no SD!");
    while(1);
  }

  // 3. Criação da Fila e Tarefas RTOS
  dataQueue = xQueueCreate(200, sizeof(DataPacket));

  if (dataQueue != NULL) {
    // TaskSensor no CORE 1 (Prioridade alta para garantir 1kHz)
    xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 5, NULL, 1);
    
    // TaskSD no CORE 0 (Prioridade menor para escrita em background)
    xTaskCreatePinnedToCore(TaskSD, "TaskSD", 8192, NULL, 2, NULL, 0);
  }
}

// ---------------------------------------------------------
// CORE 1: LEITURA E CALIBRAÇÃO (1000Hz)
// ---------------------------------------------------------
void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // Intervalo de 1ms

  for (;;) {
    DataPacket packet;
    sensors_event_t ev1, ev2;
    
    packet.timestamp = micros();
    
    // Leitura I2C
    accel1D.getEvent(&ev1);
    accel53.getEvent(&ev2);
    
    // Processamento da Calibração
    aplicarCalibracao(ev1, Ainv1D, b1D, packet.x1, packet.y1, packet.z1);
    aplicarCalibracao(ev2, Ainv53, b53, packet.x2, packet.y2, packet.z2);

    // Envia para a fila (timeout 0 para não bloquear a leitura)
    if (xQueueSend(dataQueue, &packet, 0) != pdPASS) {
      // Se chegar aqui, a TaskSD está lenta para gravar
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------------------------------------------------------
// CORE 0: ESCRITA NO CARTÃO SD
// ---------------------------------------------------------
void TaskSD(void *pvParameters) {
  DataPacket rec;
  int flushCounter = 0;

  for (;;) {
    // Espera por dados na fila indefinidamente
    if (xQueueReceive(dataQueue, &rec, portMAX_DELAY) == pdPASS) {
      if (dataFile) {
        dataFile.printf("%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f\n", 
                        rec.timestamp, 
                        rec.x1, rec.y1, rec.z1, 
                        rec.x2, rec.y2, rec.z2);
        
        // Sincronização periódica para evitar perda de dados em quedas de energia
        if (++flushCounter >= 500) {
          dataFile.flush();
          flushCounter = 0;
        }
      }
    }
  }
}

void loop() {
  // O loop principal é deletado para liberar recursos, as Tasks cuidam de tudo
  vTaskDelete(NULL);
}
