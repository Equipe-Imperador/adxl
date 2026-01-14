#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>


struct DataPacket {
  uint32_t timestamp;
  float x, y, z;
};


#define SD_CS 5
QueueHandle_t dataQueue;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
File dataFile;

//tarefas
void TaskSensor(void *pvParameters);
void TaskSD(void *pvParameters);

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000);

  if(!accel.begin()) { Serial.println("Erro ADXL!"); while(1); }
  accel.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel.setRange(ADXL345_RANGE_16_G);

  if (!SD.begin(SD_CS)) { Serial.println("Erro SD!"); while(1); }
  dataFile = SD.open("/data_dualcore.csv", FILE_WRITE);
  dataFile.println("us;x;y;z");

 
  dataQueue = xQueueCreate(100, sizeof(DataPacket));

  if (dataQueue != NULL) {
    // Core 1 - Prioridade 
    xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 3, NULL, 1);
    
    // Core 0
    xTaskCreatePinnedToCore(TaskSD, "TaskSD", 4096, NULL, 2, NULL, 0);
  }
}

// ---------------------------------------------------------
// CORE 1: LEITURA DO SENSOR (PRODUTOR)
// ---------------------------------------------------------
void TaskSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms = 1000Hz

  for (;;) {
    DataPacket packet;
    sensors_event_t event;
    
    accel.getEvent(&event);
    
    packet.timestamp = micros();
    packet.x = event.acceleration.x;
    packet.y = event.acceleration.y;
    packet.z = event.acceleration.z;

    // Envia para a fila 
    if (xQueueSend(dataQueue, &packet, 0) != pdPASS) {
      // Opcional: Serial.println("Fila cheia!"); 
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ---------------------------------------------------------
// CORE 0: GRAVAÇÃO NO SD (CONSUMIDOR)
// ---------------------------------------------------------
void TaskSD(void *pvParameters) {
  DataPacket receivedPacket;
  int counter = 0;

  for (;;) {
    
    if (xQueueReceive(dataQueue, &receivedPacket, portMAX_DELAY) == pdPASS) {
      if (dataFile) {
        dataFile.printf("%u;%.2f;%.2f;%.2f\n", 
                        receivedPacket.timestamp, 
                        receivedPacket.x, 
                        receivedPacket.y, 
                        receivedPacket.z);
        
        counter++;
        // Flush a cada 500 amostras para garantir segurança
        if (counter >= 500) {
          dataFile.flush();
          counter = 0;
        }
      }
    }
  }
}

void loop() {
  // O loop fica vazio ou pode ser usado para monitorar o status
  vTaskDelay(pdMS_TO_TICKS(1000));
}
