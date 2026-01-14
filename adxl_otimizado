#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define SD_CS   5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18


const unsigned long INTERVALO_LEITURA = 1000; 
unsigned long tempoProximaLeitura = 0;

File dataFile;
char filename[64];
int logCounter = 0;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000); // I2C Fast Mode

  if(!accel.begin()) {
    Serial.println("Erro ADXL!");
    while(1);
  }


  accel.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel.setRange(ADXL345_RANGE_16_G);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("Erro SD!");
    while(1);
  }

  sprintf(filename, "/data_%lu.csv", millis());
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.println("us;x;y;z");

  tempoProximaLeitura = micros();
}

void loop() {
  // Controle de tempo rigoroso
  if (micros() >= tempoProximaLeitura) {
    tempoProximaLeitura += INTERVALO_LEITURA;

    sensors_event_t event;
    accel.getEvent(&event);

    // sem flush repetitivo
    if (dataFile) {
      dataFile.print(micros()); dataFile.print(";");
      dataFile.print(event.acceleration.x, 2); dataFile.print(";");
      dataFile.print(event.acceleration.y, 2); dataFile.print(";");
      dataFile.println(event.acceleration.z, 2);
      
      logCounter++;
    }

    //aprox. 1 segundo
    if (logCounter >= 1000) {
      dataFile.flush();
      logCounter = 0;
      Serial.println("Bloco de 1000 amostras salvo...");
    }
  }
}
