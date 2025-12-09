#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ------------------------------
// CONFIG SD (ESP32)
// ------------------------------
#define SD_CS   5
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCK  18

File dataFile;
char filename[64];

// ------------------------------
// VARIÁVEIS ADXL / FREQUÊNCIA
// ------------------------------
unsigned long lastTimeCheck = 0;
int readingCount = 0;
float currentFrequency = 0;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
uint8_t adxlAddress = 0;

// -------------------------------------------------------------------
// FUNÇÃO: SCAN I2C E DETECTAR ADXL345 PELO DEVID (0xE5)
// -------------------------------------------------------------------
bool detectADXL345() {
  Serial.println("🔍 Varredura I2C para detectar ADXL345...");

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {

      Wire.beginTransmission(addr);
      Wire.write(0x00); // DEVID
      Wire.endTransmission();

      Wire.requestFrom(addr, (uint8_t)1);
      if (Wire.available()) {
        uint8_t devid = Wire.read();

        if (devid == 0xE5) {       
          Serial.printf("✔ ADXL345 encontrado no endereço 0x%02X\n", addr);
          adxlAddress = addr;
          return true;
        }
      }
    }
  }

  Serial.println(" Nenhum ADXL345 encontrado no I2C!");
  return false;
}

// -------------------------------------------------------------------
// SD 
// -------------------------------------------------------------------
bool initSDcard() {
  Serial.println("Inicializando SD...");

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println(" Falha ao iniciar cartão SD!");
    return false;
  }

  unsigned long long t_us = micros();
  sprintf(filename, "/log_%llu.csv", t_us);

  dataFile = SD.open(filename, FILE_WRITE);

  if (!dataFile) {
    Serial.println(" ERRO ao criar arquivo!");
    return false;
  }

  Serial.print(" Arquivo criado: ");
  Serial.println(filename);

  dataFile.println("Timestamp(us);Ax;Ay;Az;Freq(Hz)");
  dataFile.flush();

  return true;
}

// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000);

  // Detectar ADXL345
  if (!detectADXL345()) while (1) delay(1000);

  // Inicializar ADXL345
  if (!accel.begin(adxlAddress)) {
    Serial.println(" Falha ao iniciar ADXL345!");
    while (1) delay(1000);
  }

  Serial.printf("✔ ADXL345 iniciado em 0x%02X\n", adxlAddress);

  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_3200_HZ);

  // Iniciar SD
  if (!initSDcard()) while (1) delay(1000);

  lastTimeCheck = millis();
}

// -------------------------------------------------------------------
// LOOP PRINCIPAL
// -------------------------------------------------------------------
void loop() {
  readingCount++;

  sensors_event_t event;
  accel.getEvent(&event);

  // Atualiza freq (Hz)
  if (millis() - lastTimeCheck >= 1000) {
    currentFrequency = readingCount;
    readingCount = 0;
    lastTimeCheck = millis();
  }

  // -------------------------
  // SALVAR NO SD
  // -------------------------
  if (dataFile) {
    dataFile.print(micros());               dataFile.print(";");
    dataFile.print(event.acceleration.x);   dataFile.print(";");
    dataFile.print(event.acceleration.y);   dataFile.print(";");
    dataFile.print(event.acceleration.z);   dataFile.print(";");
    dataFile.println(currentFrequency);
    dataFile.flush();
  }

  // -------------------------
  // PRINT SERIAL (debug)
  // -------------------------
  Serial.print(event.acceleration.x); Serial.print(" ");
  Serial.print(event.acceleration.y); Serial.print(" ");
  Serial.print(event.acceleration.z); Serial.print(" ");
  Serial.println(currentFrequency);
}
