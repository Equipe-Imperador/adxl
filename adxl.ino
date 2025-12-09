#include <Wire.h>
#include <SPI.h>
#include <SD.h>  
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ------------------------------
// I2C Pinos ESP8266
// ------------------------------
#define I2C_SDA D2
#define I2C_SCL D1

// ------------------------------
// CONFIG SD (ESP8266 - SPI FIXO)
// ------------------------------
//#define SD_SCK   D5      // --- SD ---
//#define SD_MISO  D6      // --- SD ---
//#define SD_MOSI  D7      // --- SD ---
//#define SD_CS    D8      // --- SD ---

//File dataFile;           // --- SD ---
//char filename[48];       // --- SD ---

// ------------------------------
// VARIÁVEIS ADXL / FREQUÊNCIA
// ------------------------------
unsigned long lastTimeCheck = 0;
int readingCount = 0;
float currentFrequency = 0;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
uint8_t adxlAddress = 0;

// ------------------------------
// BUFFER DE ESCRITA (512 bytes)
// ------------------------------
//String sdBuffer = "";          // --- SD ---
//const int BUFFER_LIMIT = 512;  // --- SD ---

// -------------------------------------------------------------------
// DETECTAR ADXL345
// -------------------------------------------------------------------
bool detectADXL345() {
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {

      Wire.beginTransmission(addr);
      Wire.write(0x00); // DEVID
      Wire.endTransmission();

      Wire.requestFrom(addr, (uint8_t)1);
      if (Wire.available() && Wire.read() == 0xE5) {
        adxlAddress = addr;
        return true;
      }
    }
  }
  return false;
}

// -------------------------------------------------------------------
// INICIALIZAR SD (COMENTADO)
// -------------------------------------------------------------------
/*
bool initSDcard() {
  SPI.begin();   // ESP8266 usa pinos SPI fixos

  if (!SD.begin(SD_CS)) return false;

  unsigned long long t_us = micros();
  sprintf(filename, "/log_%llu.csv", t_us);

  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) return false;

  dataFile.println("Timestamp(us);Ax;Ay;Az;Freq(Hz)");
  dataFile.flush();

  return true;
}
*/

// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // I2C rápido

  Serial.println("Iniciando detecção do ADXL345...");

  if (!detectADXL345()) {
    Serial.println("ADXL345 não encontrado!");
    while (1) yield();
  }

  Serial.print("✔ ADXL345 encontrado no endereço: 0x");
  Serial.println(adxlAddress, HEX);

  if (!accel.begin(adxlAddress)) {
    Serial.println("Falha ao iniciar ADXL345!");
    while (1) yield();
  }

  Serial.println("✔ ADXL345 inicializado!");

  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_1600_HZ);

  // --- SD ---
  /*
  if (!initSDcard()) {
    Serial.println("Falha ao iniciar SD!");
    while (1) yield();
  }
  */

  lastTimeCheck = millis();
}

// -------------------------------------------------------------------
// LOOP PRINCIPAL (COM SD COMENTADO)
// -------------------------------------------------------------------
void loop() {
  readingCount++;

  sensors_event_t event;
  accel.getEvent(&event);

  // Atualiza frequência real medida
  if (millis() - lastTimeCheck >= 1000) {
    currentFrequency = readingCount;
    readingCount = 0;
    lastTimeCheck = millis();
  }

  // PRINT SERIAL PARA TESTE (ADXL FUNCIONANDO)
  Serial.print(micros()); Serial.print("; ");
  Serial.print(event.acceleration.x, 3); Serial.print("; ");
  Serial.print(event.acceleration.y, 3); Serial.print("; ");
  Serial.print(event.acceleration.z, 3); Serial.print("; ");
  Serial.println(currentFrequency, 1);

  // --- SD ---  (comentado)
  /*
  String line =
    String(micros()) + ";" +
    String(event.acceleration.x, 3) + ";" +
    String(event.acceleration.y, 3) + ";" +
    String(event.acceleration.z, 3) + ";" +
    String(currentFrequency, 1) + "\n";

  sdBuffer += line;

  if (sdBuffer.length() >= BUFFER_LIMIT) {
    dataFile.print(sdBuffer);
    dataFile.flush();
    sdBuffer = "";
  }

  static uint32_t lastFlush = 0;
  if (millis() - lastFlush >= 200) {
    if (sdBuffer.length() > 0) {
      dataFile.print(sdBuffer);
      dataFile.flush();
      sdBuffer = "";
    }
    lastFlush = millis();
  }
  */

  yield(); // evita reset por watchdog
}
