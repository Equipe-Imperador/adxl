#include <Wire.h>
#include <SPI.h>
//#include <SD.h>   // --- SD --- descomente quando tiver o módulo
// -------------------------------------------------------------

// Pinos I2C ESP8266
#define I2C_SDA D2
#define I2C_SCL D1

// --- SD --- Pinos SPI (hardware fixo no ESP8266)
//#define SD_SCK   D5
//#define SD_MISO  D6
//#define SD_MOSI  D7
//#define SD_CS    D8
//File dataFile;
//char filename[48];
//String sdBuffer = "";
//const int BUFFER_LIMIT = 512;

// -------------------------------------------------------------
// Registradores ADXL345
#define ADXL345_ADDR   0x53
#define REG_DEVID      0x00
#define REG_POWER_CTL  0x2D
#define REG_DATA_FORMAT 0x31
#define REG_BW_RATE    0x2C
#define REG_FIFO_CTL   0x38
#define REG_FIFO_STATUS 0x39
#define REG_DATAX0      0x32

// -------------------------------------------------------------
// Escrever em um registrador
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Leitura de um registrador
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(ADXL345_ADDR, (uint8_t)1);
  return Wire.read();
}

// -------------------------------------------------------------
// Inicialização TURBO do ADXL345
void initADXL345() {
  uint8_t devid = readRegister(REG_DEVID);
  if (devid != 0xE5) {
    Serial.print("❌ ADXL345 não detectado! DEVID = ");
    Serial.println(devid, HEX);
    while (1) yield();
  }

  Serial.println("✔ ADXL345 detectado!");

  // ±16g, full resolution
  writeRegister(REG_DATA_FORMAT, 0x0B);

  // 800 Hz
  writeRegister(REG_BW_RATE, 0x0D);

  // FIFO modo STREAM (até 32 amostras)
  writeRegister(REG_FIFO_CTL, 0x9F);

  // Power ON
  writeRegister(REG_POWER_CTL, 0x08);

  Serial.println("✔ ADXL345 inicializado em modo TURBO!");
}

// -------------------------------------------------------------
// Leitura RAW dos 3 eixos
void readAccelRaw(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(REG_DATAX0);
  Wire.endTransmission(false);

  Wire.requestFrom(ADXL345_ADDR, (uint8_t)6);

  x = (Wire.read() | (Wire.read() << 8));
  y = (Wire.read() | (Wire.read() << 8));
  z = (Wire.read() | (Wire.read() << 8));
}

float toG(int16_t raw) {
  return raw * 0.0039;
}

// ------------------------------
// Controle de frequência
unsigned long lastFreq = 0;
int count = 0;
float freqHz = 0;

// -------------------------------------------------------------------
// --- SD --- inicialização COMENTADA
/*
bool initSDcard() {
  SPI.begin();

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

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  initADXL345();

  lastFreq = millis();

  // --- SD ---
  /*
  if (!initSDcard()) {
    Serial.println("❌ Falha ao iniciar SD!");
    while (1) yield();
  }
  */
}

void loop() {

  // Quantidade de amostras no FIFO
  uint8_t fifoSamples = readRegister(REG_FIFO_STATUS) & 0x3F;

  while (fifoSamples--) {
    int16_t xRaw, yRaw, zRaw;
    readAccelRaw(xRaw, yRaw, zRaw);

    float ax = toG(xRaw) * 9.80665;//conversao para m/s2
    float ay = toG(yRaw) * 9.80665;
    float az = toG(zRaw) * 9.80665;

    // PRINT SERIAL PARA TESTE
    Serial.print(micros()); Serial.print("; ");
    Serial.print(ax, 3); Serial.print("; ");
    Serial.print(ay, 3); Serial.print("; ");
    Serial.print(az, 3); Serial.print("; ");
    Serial.println(freqHz, 1);

    count++;

    // PREPARO PARA SD (comentado)
    /*
    String line = 
      String(micros()) + ";" +
      String(ax, 3) + ";" +
      String(ay, 3) + ";" +
      String(az, 3) + ";" +
      String(freqHz, 1) + "\n";

    sdBuffer += line;

    if (sdBuffer.length() >= BUFFER_LIMIT) {
      dataFile.print(sdBuffer);
      dataFile.flush();
      sdBuffer = "";
    }
    */
  }

  // Atualiza frequência real
  if (millis() - lastFreq >= 1000) {
    freqHz = count;
    count = 0;
    lastFreq = millis();
  }

  // Flush periódico do SD (comentado)
  /*
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

  yield();
}
