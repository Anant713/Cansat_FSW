/* 
  BMP280 altitude (AHT20 + BMP280 "SimpleRobot" board)
  Board : ESP32 DevKit V1
  
  Wiring (I2C):
    VDD -> 3.3V (NOT 5V: there is no regulator on this board, BMP280 max is 3.6 V)
    GND -> GND
    SDA -> GPIO 21
    SCL -> GPIO 22
    
  Serial Monitor: 115200 baud
  Commands: send z to set the current height as the new zero.
  Needs the "Adafruit BMP280 Library" (Library Manager).
*/
#include <Wire.h>
#include <Adafruit_BMP280.h>

// Today's sea-level pressure (QNH) for your area, in hPa. 
const float SEA_LEVEL_HPA = 1013.25;
const unsigned long PRINT_MS = 200; 
const int ZERO_SAMPLES = 40;        
const float SMOOTH = 0.2;           

Adafruit_BMP280 bmp;
uint8_t sensorAddr = 0, sensorId = 0;
float groundHpa = 0;
float relSmooth = 0;
bool smoothInit = false;

// Standard barometric formula (troposphere, ISA)
float altitudeM(float hpa, float refHpa) {
  return 44330.0f * (1.0f - pow(hpa / refHpa, 0.190295f));
}

uint8_t readChipId(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  if (Wire.endTransmission() != 0) return 0;
  
  // ESP32 requires size_t or explicit signature match for requestFrom
  Wire.requestFrom(addr, (size_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool startSensor() {
  // ESP32 automatically maps Wire to default pins: SDA (21) and SCL (22)
  Wire.begin(); 
  
  const uint8_t addrs[] = {0x77, 0x76};
  for (uint8_t a : addrs) {
    uint8_t id = readChipId(a);
    if (id == 0) continue;
    
    Serial.print(F("Found chip ID 0x"));
    Serial.print(id, HEX);
    Serial.print(F(" at 0x"));
    Serial.println(a, HEX);
    
    if (bmp.begin(a, id)) {
      sensorAddr = a;
      sensorId = id;
      return true;
    }
  }
  return false;
}

void zeroHere() {
  float sum = 0;
  for (int i = 0; i < ZERO_SAMPLES; i++) {
    sum += bmp.readPressure() / 100.0f;
    delay(50);
  }
  groundHpa = sum / ZERO_SAMPLES;
  smoothInit = false;
  Serial.print(F("Zero set at "));
  Serial.print(groundHpa, 2);
  Serial.println(F(" hPa"));
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nBMP280 altitude"));
  
  if (!startSensor()) {
    Serial.println(F("No BMP280 found. Check wiring: VDD-3.3V, GND-GND, SDA-GPIO21, SCL-GPIO22."));
    while (true) { delay(1000); } // Added delay to prevent ESP32 watchdog triggers
  }

  // Highest resolution + strongest internal filter: about 0.2 m noise
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,  // temperature
                  Adafruit_BMP280::SAMPLING_X16, // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);
                  
  delay(1000); // let the internal filter settle
  Serial.println(F("Hold still, setting zero..."));
  zeroHere();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') zeroHere();
  }

  static unsigned long last = 0;
  if (millis() - last < PRINT_MS) return;
  last = millis();

  if (readChipId(sensorAddr) != sensorId) {
    Serial.println(F("SENSOR NOT RESPONDING - check SDA/SCL wires"));
    return;
  }

  uint8_t ctrl = 0;
  Wire.beginTransmission(sensorAddr);
  Wire.write(0xF4);
  Wire.endTransmission();
  Wire.requestFrom(sensorAddr, (size_t)1);
  
  if (Wire.available()) ctrl = Wire.read();
  
  if ((ctrl & 0x03) != 0x03) {
    Serial.print(F("Sensor asleep (ctrl 0x"));
    Serial.print(ctrl, HEX);
    Serial.println(F("), restarting it"));
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_1);
    return;
  }

  float tempC = bmp.readTemperature();
  float hpa = bmp.readPressure() / 100.0f;
  float rel = altitudeM(hpa, groundHpa);
  float abs_ = altitudeM(hpa, SEA_LEVEL_HPA);

  if (!smoothInit) {
    relSmooth = rel;
    smoothInit = true;
  }
  relSmooth += SMOOTH * (rel - relSmooth);

  Serial.print(F("T "));
  Serial.print(tempC, 2);
  Serial.print(F(" C | P "));
  Serial.print(hpa, 2);
  Serial.print(F(" hPa | rel "));
  Serial.print(relSmooth, 2);
  Serial.print(F(" m | abs "));
  Serial.print(abs_, 1);
  Serial.println(F(" m"));
}
