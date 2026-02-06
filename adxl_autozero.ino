#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Pino do sensor de pressão (ADC1_CH4)
#define PIN_FREIO 32
#define SD_CS 5

struct DataPacket {
  uint32_t us;
  float x1, y1, z1; 
  float x2, y2, z2;
  float freio; 
};

QueueHandle_t dataQueue;
File dataFile;
char filename[32]; 

Adafruit_ADXL345_Unified accel1D = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel53 = Adafruit_ADXL345_Unified(0x53);

// --- MATRIZES DE GANHO (Fixas - Característica do chip) ---
const float Ainv1D[3][3] = {{0.952875, 0.006993, 0.003366}, {0.006993, 0.952172, -0.000777}, {0.003366, -0.000777, 0.987901}};
const float Ainv53[3][3] = {{0.970592, -0.048166, 0.008696}, {-0.048166, 0.988076, -0.025318}, {0.008696, -0.025318, 0.984936}};

// --- VETORES DE BIAS (Agora variáveis para permitir o Auto-Zero) ---
float b1D[3] = {0.0, 0.0, 0.0};
float b53[3] = {0.0, 0.0, 0.0};

void TaskAcquisition(void *pvParameters);
void TaskStorage(void *pvParameters);

// --- FUNÇÃO AUTO-ZERO ---
// Lê os sensores por 2 segundos e define o ponto zero baseado na posição atual do carro
void performAutoZero() {
  Serial.println(">>> INICIANDO AUTO-ZERO. MANTENHA O CARRO PARADO E NIVELADO...");
  
  float sumX1 = 0, sumY1 = 0, sumZ1 = 0;
  float sumX2 = 0, sumY2 = 0, sumZ2 = 0;
  const int samples = 200; 

  for (int i = 0; i < samples; i++) {
    sensors_event_t ev1, ev2;
    accel1D.getEvent(&ev1);
    accel53.getEvent(&ev2);
    
    sumX1 += ev1.acceleration.x; sumY1 += ev1.acceleration.y; sumZ1 += ev1.acceleration.z;
    sumX2 += ev2.acceleration.x; sumY2 += ev2.acceleration.y; sumZ2 += ev2.acceleration.z;
    
    delay(10); // 100Hz para amostragem do zero
  }

  // Define os novos Offsets (Bias)
  // No eixo Z, subtraímos a gravidade (9.81) pois o carro está parado
  b1D[0] = sumX1 / samples;
  b1D[1] = sumY1 / samples;
  b1D[2] = (sumZ1 / samples) - 9.80665;

  b53[0] = sumX2 / samples;
  b53[1] = sumY2 / samples;
  b53[2] = (sumZ2 / samples) - 9.80665;

  Serial.println(">>> AUTO-ZERO CONCLUIDO COM SUCESSO!");
  Serial.printf("Bias S1 (1D): [%.3f, %.3f, %.3f]\n", b1D[0], b1D[1], b1D[2]);
  Serial.printf("Bias S2 (53): [%.3f, %.3f, %.3f]\n", b53[0], b53[1], b53[2]);
}

void prepareFile() {
  if (!SD.begin(SD_CS)) {
    Serial.println("Erro ao iniciar SD!");
    while(1);
  }

  for (int i = 0; i < 1000; i++) {
    sprintf(filename, "/adxl%02d.csv", i);
    if (!SD.exists(filename)) {
      Serial.printf("Arquivo definido: %s\n", filename);
      break;
    }
  }

  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.println("us;x1;y1;z1;x2;y2;z2;freio");
    Serial.println("Cabeçalho gravado com sucesso.");
  } else {
    Serial.println("Erro ao criar o arquivo!");
    while(1);
  }
}

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(PIN_FREIO, INPUT);
  analogReadResolution(12);

  if(!accel1D.begin()) { Serial.println("ERRO 0x1D"); while(1); }
  if(!accel53.begin()) { Serial.println("ERRO 0x53"); while(1); }
  
  accel1D.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel1D.setRange(ADXL345_RANGE_16_G);
  accel53.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel53.setRange(ADXL345_RANGE_16_G);

  // 1. Calibra o ponto zero antes de abrir o arquivo e iniciar as tasks
  performAutoZero();

  // 2. Prepara o SD
  prepareFile();

  dataQueue = xQueueCreate(300, sizeof(DataPacket));
  
  xTaskCreatePinnedToCore(TaskAcquisition, "TaskAcq", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(TaskStorage, "TaskStore", 8192, NULL, 2, NULL, 0);
}

// --- CORE 1: LEITURA DOS SENSORES (PRODUTOR) ---
void TaskAcquisition(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); 

  for (;;) {
    DataPacket p;
    sensors_event_t ev1, ev2;
    p.us = micros();

    int rawADC = analogRead(PIN_FREIO);
    p.freio = (rawADC * 3.3) / 4095.0;

    accel1D.getEvent(&ev1);
    accel53.getEvent(&ev2);

    // Usa os valores de bias calculados no Auto-Zero
    float f1x = ev1.acceleration.x - b1D[0];
    float f1y = ev1.acceleration.y - b1D[1];
    float f1z = ev1.acceleration.z - b1D[2];
    p.x1 = Ainv1D[0][0]*f1x + Ainv1D[0][1]*f1y + Ainv1D[0][2]*f1z;
    p.y1 = Ainv1D[1][0]*f1x + Ainv1D[1][1]*f1y + Ainv1D[1][2]*f1z;
    p.z1 = Ainv1D[2][0]*f1x + Ainv1D[2][1]*f1y + Ainv1D[2][2]*f1z;

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

// --- CORE 0: GRAVAÇÃO E MONITORAMENTO (CONSUMIDOR) ---
void TaskStorage(void *pvParameters) {
  DataPacket r;
  uint32_t count = 0;
  uint32_t timerRelatorio = millis();
  int printDiv = 0;

  for (;;) {
    if (xQueueReceive(dataQueue, &r, portMAX_DELAY)) {
      if (dataFile) {
        dataFile.printf("%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.2f\n", 
                        r.us, r.x1, r.y1, r.z1, r.x2, r.y2, r.z2, r.freio);
        count++;
      }

      if (++printDiv >= 100) {
        Serial.printf("[%s] S1[Z]:%.2f | S2[Z]:%.2f | FREIO:%.2fV | FPS:%uHz\n", 
                       filename, r.z1, r.z2, r.freio, count);
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
