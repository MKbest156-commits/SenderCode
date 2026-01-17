/*
====================================================================
AMC Team – CanSat Österreich 2026
Telemetrie-Sender (ESP32-C3) – Sensor-Messung + CSV über LoRa
====================================================================

Projekt/Team:
  - AMC Team (CanSat 2026)
  - Subsystem: Telemetrie / Onboard-Datenerfassung

Was macht dieses Programm?
  - Liest 3 I2C-Sensoren zyklisch mit 2 Hz (alle 500 ms):
      1) BMI160  (Gyro + Beschleunigung)
      2) BMP380  (Temperatur + Luftdruck)
      3) BME688  (Temperatur + Luftdruck + rel. Feuchte + Gaswiderstand)
  - Formatiert die Messwerte als eine CSV-Zeile (Text)
  - Gibt dieselbe CSV-Zeile
      a) am Serial Monitor aus (Debug/Logging am PC)
      b) und sendet sie als LoRa-Paket (Telemetry Downlink)

Warum CSV?
  - Einfach zu testen (Serial Monitor)
  - Kann am PC direkt geloggt und in Excel/Python verarbeitet werden

Verdrahtung / Pins
------------------
ESP32-C3 (I2C):
  - SDA = GPIO 8
  - SCL = GPIO 9
  - I2C Clock = 100 kHz (stabil)

Sensoren (I2C):
  - BMI160  (Addr 0x69)  -> SDA/SCL + 3.3V + GND
  - BMP380  (Addr 0x76/0x77, hier 0x77) -> SDA/SCL + 3.3V + GND
  - BME688  (Addr 0x76 oder 0x77, wird automatisch probiert) -> SDA/SCL + 3.3V + GND

LoRa (RFM95W / SX1276) mit Arduino LoRa Library (LoRa.h):
  - NSS/CS = GPIO 3
  - RST    = GPIO 2
  - DIO0   = GPIO 1
  - VCC    = 3.3V (WICHTIG: nicht 5V!)
  - GND    = GND

  Zusätzlich SPI (Board-abhängig):
  - SCK / MISO / MOSI müssen korrekt mit dem ESP32-C3 verbunden sein.
    (Diese Pins hängen von Eurem ESP32-C3 Board ab – bei vielen Boards sind
     sie bereits als Standard-SPI verdrahtet.)

LoRa Parameter:
  - Frequenz: 868 MHz (EU)
  - SyncWord: 0xF3 (muss beim Empfänger identisch sein)
  - Sendeintervall: 500 ms (2 Hz)

Hinweise aus der Praxis (kurz):
  - Wenn LoRa Reichweite/Qualität schlecht wird: SF erhöhen oder seltener senden.
  - CSV-Payload ist relativ lang, aber bei SF7 meist noch ok.

Autor:
  - AMC Team
  - Stand: 13.1.2026
====================================================================
*/

#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#include <SPI.h>
#include <LoRa.h>

// ---------------- I2C Pins ----------------
static const int I2C_SDA = 8;
static const int I2C_SCL = 9;
static const uint32_t I2C_HZ = 100000;

// ---------------- Messrate ----------------
// 500 ms => 2 Messungen pro Sekunde
static const uint32_t PERIOD_MS = 500;

// ---------------- I2C Addresses ----------------
static const int8_t  BMI_ADDR = 0x69;  // BMI160 Adresse (je nach Breakout 0x68/0x69)
static const uint8_t BMP_ADDR = 0x77; // BMP380: oft 0x76 oder 0x77 (hier fix 0x77)
static uint8_t BME_ADDR = 0;          // BME688: wird durch beginBME688() erkannt

// ---------------- LoRa Pins ----------------
// Wie in Deinem Beispielcode (Random Nerd Tutorials)
#define LORA_SS   3
#define LORA_RST  2
#define LORA_DIO0 1

// ---------------- LoRa Settings ----------------
static const long    LORA_FREQ     = 868E6;
static const uint8_t LORA_SYNCWORD = 0xF3;

// ---------------- Sensor-Objekte ----------------
DFRobot_BMI160  bmi160;  // IMU (Gyro + Acc)
Adafruit_BMP3XX bmp;     // Druck + Temp
Adafruit_BME680 bme;     // Temp + Druck + RH + Gas (BME688 kompatibel)

// BME688 kann je nach Board auf 0x76 oder 0x77 liegen -> wir probieren beides
bool beginBME688() {
  if (bme.begin(0x76, &Wire)) { BME_ADDR = 0x76; return true; }
  if (bme.begin(0x77, &Wire)) { BME_ADDR = 0x77; return true; }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ---------- I2C initialisieren ----------
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeout(50);  // falls sich ein Sensor "aufhängt", nicht ewig blockieren

  // ---------- BMI160 ----------
  Serial.print("BMI160 init... ");
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("FAIL (softReset)");
    while (1) delay(1000);
  }
  if (bmi160.I2cInit(BMI_ADDR) != BMI160_OK) {
    Serial.println("FAIL (I2cInit)");
    while (1) delay(1000);
  }
  Serial.println("OK");

  // ---------- BMP380 ----------
  Serial.print("BMP380 init... ");
  if (!bmp.begin_I2C(BMP_ADDR, &Wire)) {
    Serial.println("FAIL (Addr 0x76/0x77 pruefen)");
    while (1) delay(1000);
  }
  // Oversampling/Filter so gewählt, dass es stabil ist und schnell genug
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("OK");

  // ---------- BME688 ----------
  Serial.print("BME688 init... ");
  if (!beginBME688()) {
    Serial.println("FAIL (Addr 0x76/0x77 pruefen)");
    while (1) delay(1000);
  }
  // Oversampling + Filter + Heater (Gas-Sensor braucht Heater)
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
  Serial.print("OK @0x"); Serial.println(BME_ADDR, HEX);

  // ---------- LoRa ----------
  Serial.print("LoRa init... ");
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Startet das Modul; wenn es nicht geht, meist Pins/Versorgung/SPI falsch
  while (!LoRa.begin(LORA_FREQ)) {
    Serial.print(".");
    delay(500);
  }
  LoRa.setSyncWord(LORA_SYNCWORD);
  Serial.println("OK");

  // Header für CSV (damit man’s im Log/Excel gleich weiß)
  Serial.println("t_ms,gx,gy,gz,ax,ay,az,bmp_T,bmp_P,bme_T,bme_P,bme_RH,bme_gas");
}

void loop() {
  // Timing ohne delay(): wir triggern alle PERIOD_MS
  static uint32_t tNext = 0;
  uint32_t now = millis();
  if ((int32_t)(now - tNext) < 0) return;
  tNext = now + PERIOD_MS;

  // ==========================================================
  // 1) BMI160 (Gyro + Acc) lesen
  // ==========================================================
  int16_t ag[6] = {0};
  bool okIMU = (bmi160.getAccelGyroData(ag) == 0);

  // Default: NAN (damit man im Log sieht, wenn Sensor nicht ok ist)
  float gx = NAN, gy = NAN, gz = NAN;
  float ax = NAN, ay = NAN, az = NAN;

  if (okIMU) {
    // DFRobot Library: ag[0..2] = Gyro, ag[3..5] = Acc

    // Gyro: deg/s -> rad/s
    gx = (ag[0] * 3.1415926f) / 180.0f;
    gy = (ag[1] * 3.1415926f) / 180.0f;
    gz = (ag[2] * 3.1415926f) / 180.0f;

    // Acc: raw -> g (wie im Originalcode)
    ax = ag[3] / 16384.0f;
    ay = ag[4] / 16384.0f;
    az = ag[5] / 16384.0f;
  }

  // ==========================================================
  // 2) BMP380 lesen (Temp + Pressure)
  // ==========================================================
  float bmpT = NAN, bmpP = NAN;
  bool okBMP = bmp.performReading();
  if (okBMP) {
    bmpT = bmp.temperature;
    bmpP = bmp.pressure; // Pa
  }

  // ==========================================================
  // 3) BME688 lesen (Temp + Pressure + RH + Gas)
  // ==========================================================
  float bmeT = NAN, bmeP = NAN, bmeRH = NAN, bmeGas = NAN;
  bool okBME = bme.performReading();
  if (okBME) {
    bmeT = bme.temperature;
    bmeP = bme.pressure;                 // Pa
    bmeRH = bme.humidity;                // %
    bmeGas = (float)bme.gas_resistance;  // Ohm
  }

  // ==========================================================
  // 4) CSV-Zeile bauen (Serial + LoRa sollen identisch sein)
  // ==========================================================
  String line;
  line.reserve(220);

  // Zeitstempel in ms seit Start
  line += String(now);
  line += ",";

  // Gyro
  if (okIMU) {
    line += String(gx, 6) + "," + String(gy, 6) + "," + String(gz, 6);
  } else {
    line += "nan,nan,nan";
  }

  line += ",";

  // Acc
  if (okIMU) {
    line += String(ax, 6) + "," + String(ay, 6) + "," + String(az, 6);
  } else {
    line += "nan,nan,nan";
  }

  line += ",";

  // BMP380
  if (okBMP) {
    line += String(bmpT, 2) + "," + String(bmpP, 0);
  } else {
    line += "nan,nan";
  }

  line += ",";

  // BME688
  if (okBME) {
    line += String(bmeT, 2) + ",";
    line += String(bmeP, 0) + ",";
    line += String(bmeRH, 2) + ",";
    line += String(bmeGas, 0);
  } else {
    line += "nan,nan,nan,nan";
  }

  // ==========================================================
  // 5) Ausgabe: Serial + LoRa
  // ==========================================================
  Serial.println(line);

  // LoRa-Paket senden (blockiert kurz bis TX fertig ist)
  LoRa.beginPacket();
  LoRa.print(line);
  LoRa.endPacket();
}
