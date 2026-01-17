#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Pino do sensor de pressão (ADC1_CH4)
#define PIN_FREIO 32

struct DataPacket {
  uint32_t us;
  float x1, y1, z1; 
  float x2, y2, z2;
  float freio; // Tensão ou Pressão calculada
};

#define SD_CS 5
QueueHandle_t dataQueue;
File dataFile;
char filename[32];

Adafruit_ADXL345_Unified accel1D = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel53 = Adafruit_ADXL345_Unified(0x53);

// --- MATRIZES DE CALIBRAÇÃO ---
const float Ainv1D[3][3] = {{0.952875, 0.006993, 0.003366}, {0.006993, 0.952172, -0.000777}, {0.003366, -0.000777, 0.987901}};
const float b1D[3]       = {0.094440, -0.361731, 0.911032};
const float Ainv53[3][3] = {{0.970592, -0.048166, 0.008696}, {-0.048166, 0.988076, -0.025318}, {0.008696, -0.025318, 0.984936}};
const float b53[3]       = {0.931123, -0.218200, 0.350909};

void TaskAcquisition(void *pvParameters);
void TaskStorage(void *pvParameters);

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000);

  // Configuração do pino analógico
  pinMode(PIN_FREIO, INPUT);
  analogReadResolution(12); // Garante 12 bits (0-4095)

  if(!accel1D.begin()) { Serial.println("ERRO 0x1D"); while(1); }
  if(!accel53.begin()) { Serial.println("ERRO 0x53"); while(1); }
  
  accel1D.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel1D.setRange(ADXL345_RANGE_16_G);
  accel53.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel53.setRange(ADXL345_RANGE_16_G);

  if (!SD.begin(SD_CS)) { Serial.println("Erro SD!"); while(1); }
  sprintf(filename, "/baja_%u.csv", micros());
  dataFile = SD.open(filename, FILE_WRITE);
  
  if(dataFile) {
    // Cabeçalho atualizado com "freio"
    dataFile.println("us;x1;y1;z1;x2;y2;z2;freio");
  }

  dataQueue = xQueueCreate(300, sizeof(DataPacket));
  
  xTaskCreatePinnedToCore(TaskAcquisition, "TaskAcq", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(TaskStorage, "TaskStore", 8192, NULL, 2, NULL, 0);
}

// --- CORE 1: LEITURA DOS SENSORES (ACELERÔMETROS + FREIO) ---
void TaskAcquisition(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); 

  for (;;) {
    DataPacket p;
    sensors_event_t ev1, ev2;
    p.us = micros();

    // 1. Leitura do Pedal de Freio (ADC)
    int rawADC = analogRead(PIN_FREIO);
    p.freio = (rawADC * 3.3) / 4095.0; // Converte para Volts

    // 2. Leitura dos Acelerômetros
    accel1D.getEvent(&ev1);
    accel53.getEvent(&ev2);

    // 3. Calibração SENSOR 1
    float f1x = ev1.acceleration.x - b1D[0];
    float f1y = ev1.acceleration.y - b1D[1];
    float f1z = ev1.acceleration.z - b1D[2];
    p.x1 = Ainv1D[0][0]*f1x + Ainv1D[0][1]*f1y + Ainv1D[0][2]*f1z;
    p.y1 = Ainv1D[1][0]*f1x + Ainv1D[1][1]*f1y + Ainv1D[1][2]*f1z;
    p.z1 = Ainv1D[2][0]*f1x + Ainv1D[2][1]*f1y + Ainv1D[2][2]*f1z;

    // 4. Calibração SENSOR 2
    float f2x = ev2.acceleration.x - b53[0];
    float f2y = ev2.acceleration.y - b53[1];
    float f2z = ev2.acceleration.z - b53[2];
    p.x2 = Ainv53[0][0]*f2x + Ainv53[0][1]*f2y + Ainv53[0][2]*f2z;
    p.y2 = Ainv53[1][0]*f2x + Ainv53[1][1]*f2y + Ainv53[1][2]*f2z;
    p.z2 = Ainv53[2][0]*f2x + Ainv53[2][1]*f2y + Ainv53[2][2]*f2z;

    xQueueSend(dataQueue, &p, 0);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// --- CORE 0: GRAVAÇÃO E MONITORAMENTO ---
void TaskStorage(void *pvParameters) {
  DataPacket r;
  uint32_t count = 0;
  uint32_t timerRelatorio = millis();
  int printDiv = 0;

  for (;;) {
    if (xQueueReceive(dataQueue, &r, portMAX_DELAY)) {
      if (dataFile) {
        // Gravação no SD com o novo campo de freio
        dataFile.printf("%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.2f\n", 
                        r.us, r.x1, r.y1, r.z1, r.x2, r.y2, r.z2, r.freio);
        count++;
      }

      // Visualização no Monitor Serial
      if (++printDiv >= 100) {
        Serial.printf("S1[Z]:%.2f | S2[Z]:%.2f | FREIO:%.2fV | FREQ:%uHz\n", 
                       r.z1, r.z2, r.freio, count);
        printDiv = 0;
      }

      if (millis() - timerRelatorio >= 1000) {
        dataFile.flush();
        count = 0;
        timerRelatorio = millis();
      }
    }
  }
}

void loop() { vTaskDelete(NULL); }
