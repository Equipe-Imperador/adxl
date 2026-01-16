#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define SD_CS 5

const unsigned long INTERVALO_LEITURA_US = 1000; 
unsigned long tempoProximaLeitura = 0;
int logCounter = 0;

Adafruit_ADXL345_Unified accel1D = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel53 = Adafruit_ADXL345_Unified(0x53);
File dataFile;
char filename[32];

// --- Matrizes de Calibração ---
const float Ainv1D[3][3] = {{0.952875, 0.006993, 0.003366}, {0.006993, 0.952172, -0.000777}, {0.003366, -0.000777, 0.987901}};
const float b1D[3]       = {0.094440, -0.361731, 0.911032};

const float Ainv53[3][3] = {{0.970592, -0.048166, 0.008696}, {-0.048166, 0.988076, -0.025318}, {0.008696, -0.025318, 0.984936}};
const float b53[3]       = {0.931123, -0.218200, 0.350909};

void setup() {
  Serial.begin(921600); // Velocidade alta necessária para 1kHz
  Wire.begin();
  Wire.setClock(400000); 

  if(!accel1D.begin() || !accel53.begin()) {
    Serial.println("Erro nos ADXLs!");
    while(1);
  }
  
  accel1D.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel1D.setRange(ADXL345_RANGE_16_G);
  accel53.setDataRate(ADXL345_DATARATE_1600_HZ);
  accel53.setRange(ADXL345_RANGE_16_G);

  if (!SD.begin(SD_CS)) {
    Serial.println("Erro no SD!");
    while(1);
  }

  sprintf(filename, "/log_%u.csv", micros());
  dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    dataFile.println("us;x1;y1;z1;x2;y2;z2");
  }

  tempoProximaLeitura = micros();
}

void loop() {
  if (micros() >= tempoProximaLeitura) {
    tempoProximaLeitura += INTERVALO_LEITURA_US;

    sensors_event_t ev1, ev2;
    accel1D.getEvent(&ev1);
    accel53.getEvent(&ev2);

    // 1. Subtração do Bias (f = h - b)
    float f1[3] = {ev1.acceleration.x - b1D[0], ev1.acceleration.y - b1D[1], ev1.acceleration.z - b1D[2]};
    float f2[3] = {ev2.acceleration.x - b53[0], ev2.acceleration.y - b53[1], ev2.acceleration.z - b53[2]};

    // 2. Cálculo dos valores calibrados (Ainv * f)
    float x1 = Ainv1D[0][0]*f1[0] + Ainv1D[0][1]*f1[1] + Ainv1D[0][2]*f1[2];
    float y1 = Ainv1D[1][0]*f1[0] + Ainv1D[1][1]*f1[1] + Ainv1D[1][2]*f1[2];
    float z1 = Ainv1D[2][0]*f1[0] + Ainv1D[2][1]*f1[1] + Ainv1D[2][2]*f1[2];

    float x2 = Ainv53[0][0]*f2[0] + Ainv53[0][1]*f2[1] + Ainv53[0][2]*f2[2];
    float y2 = Ainv53[1][0]*f2[0] + Ainv53[1][1]*f2[1] + Ainv53[1][2]*f2[2];
    float z2 = Ainv53[2][0]*f2[0] + Ainv53[2][1]*f2[1] + Ainv53[2][2]*f2[2];

    // 3. Print para o Serial Monitor (Valores Calibrados)
    // Mostra: [S1] x, y, z | [S2] x, y, z
    Serial.printf("S1: %.2f, %.2f, %.2f | S2: %.2f, %.2f, %.2f\n", x1, y1, z1, x2, y2, z2);

    // 4. Gravação no SD
    if (dataFile) {
      dataFile.printf("%u;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f\n", micros(), x1, y1, z1, x2, y2, z2);
      logCounter++;
      if (logCounter >= 500) {
        dataFile.flush();
        logCounter = 0;
      }
    }
  }
}
