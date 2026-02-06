// --- [CÓDIGO DE COLETA PARA CALIBRAÇÃO] ---
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define SD_CS 5
#define PIN_FREIO 32

struct DataPacket {
  uint32_t us;
  float x1, y1, z1; 
  float x2, y2, z2;
};

QueueHandle_t dataQueue;
File dataFile;
char filename[32];

Adafruit_ADXL345_Unified accel1D = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel53 = Adafruit_ADXL345_Unified(0x53);

void prepareFile() {
  if (!SD.begin(SD_CS)) { Serial.println("Erro SD!"); while(1); }
  for (int i = 0; i < 1000; i++) {
    sprintf(filename, "/calib%02d.csv", i);
    if (!SD.exists(filename)) break;
  }
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    // Cabeçalho simples para o Magneto
    dataFile.println("x;y;z"); 
    Serial.printf("Gravando dados brutos em: %s\n", filename);
  }
}

void setup() {
  Serial.begin(921600);
  Wire.begin();
  if(!accel1D.begin()) { Serial.println("Erro 1D"); while(1); }
  if(!accel53.begin()) { Serial.println("Erro 53"); while(1); }
  
  accel1D.setRange(ADXL345_RANGE_16_G);
  accel53.setRange(ADXL345_RANGE_16_G);

  prepareFile();
  dataQueue = xQueueCreate(300, sizeof(DataPacket));
  
  xTaskCreatePinnedToCore(TaskAcquisition, "TaskAcq", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(TaskStorage, "TaskStore", 8192, NULL, 2, NULL, 0);
}

void TaskAcquisition(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    DataPacket p;
    sensors_event_t ev2; // Focando no sensor 0x53 que mudou de lugar
    accel53.getEvent(&ev2);

    // COLETANDO APENAS O VALOR BRUTO (RAW)
    p.x2 = ev2.acceleration.x;
    p.y2 = ev2.acceleration.y;
    p.z2 = ev2.acceleration.z;

    xQueueSend(dataQueue, &p, 0);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz é suficiente para calibração
  }
}

void TaskStorage(void *pvParameters) {
  DataPacket r;
  for (;;) {
    if (xQueueReceive(dataQueue, &r, portMAX_DELAY)) {
      if (dataFile) {
        // Formato para o Magneto ler facilmente (apenas o sensor alvo)
        dataFile.printf("%.4f;%.4f;%.4f\n", r.x2, r.y2, r.z2);
      }
      if (millis() % 1000 < 10) dataFile.flush();
    }
  }
}

void loop() { vTaskDelete(NULL); }
