#include <Homie.h>
#include <SPI.h>
#include <Wire.h>
#include <BME280I2C.h>

#define SERIAL_DEBUG false

const int sensorInterval = 250;
const int ldrPin = A0;

BME280I2C bme(
  // * Temperature Oversampling Rate (tosr): uint8_t, default = 0x1
  //   values: B000 = Skipped, B001 = x1, B010 = x2, B011 = x4, B100 = x8, B101/other = x16
  B001,

  // * Humidity Oversampling Rate (hosr): uint8_t, default = 0x1
  //   values: B000 = Skipped, B001 = x1, B010 = x2, B011 = x4, B100 = x8, B101/other = x16
  B001,

  // * Pressure Oversampling Rate (posr): uint8_t, default = 0x1
  //   values: B000 = Skipped, B001 = x1, B010 = x2, B011 = x4, B100 = x8, B101/other = x16
  B100,

  // * Mode: uint8_t, default = Normal
  //   values: Sleep = B00, Forced = B01 and B10, Normal = B11
  B001,

  // * Standby Time (st): uint8_t, default = 1000ms
  //   values: B000 = 0.5ms, B001 = 62.5ms, B010 = 125ms, B011 = 250ms, B100 = 250ms, B101 = 1000ms, B110 = 10ms, B111 = 20ms
  B001,

  // * Filter: uint8_t, default = None
  //   values: B000 = off, B001 = 2, B010 = 4, B011 = 8, B100/other = 16
  B000,

  // * SPI Enable: bool, default = false
  //   values: true = enable, false = disable
  false,

  // * BME280 Address: uint8_t, default = 0x76
  //   values: any uint8_t
  0x76
);



struct SensorValue {
  float sigDiff;
  float offset;
  long maxInterval;
  HomieNode node;
  unsigned long lastTime;
  float lastValue;

  // TODO: fix
  // void init() {
    // node.advertise("unit");
    // node.advertise("value");
  // }

  void sensed(unsigned long senseTime, float value) {
    bool significant = abs(value - lastValue) >= sigDiff;
    bool aboutTime = senseTime - lastTime >= maxInterval*1000;

    if (significant || aboutTime) {
#if SERIAL_DEBUG
      Serial.print("Sensor ");
      Serial.print(topic);
      Serial.print(" ");
      Serial.print(value);
      Serial.print(" ");
      Serial.print(significant ? "sig" : "x");
      Serial.print(" ");
      Serial.print(aboutTime ? "time" : "x");
      Serial.println();
#endif
      
      if (offset != 0) {
        node.setProperty("valueRaw").send(String(value));
        node.setProperty("value").send(String(value + offset));
      } else {
        node.setProperty("value").send(String(value));
      }
      
      lastValue = value;
      lastTime = senseTime;
    }
    
  }
};

SensorValue tempSensor  = { 0.02, -6.745, 600, { "temperature", "temperature" } };
SensorValue pressSensor = { 0.10,      0, 600, { "pressure", "pressure" } };
SensorValue humidSensor = { 0.30,      0, 600, { "humidity", "humidity" } };
SensorValue lightSensor = { 0.02,      0, 600, { "light", "light" } };

unsigned long lastLoop = 0;

void setupHandler() {
  pinMode(ldrPin, INPUT);
  while (!bme.begin()) {  
    Homie.getLogger() << "Unable to find BME280 sensor!" << endl;
    delay(1000);
  }
}

void loopHandler() {

  unsigned long now = millis();

  if (now - lastLoop >= sensorInterval) {

    int lightRaw = analogRead(ldrPin);
    float light = lightRaw/1023.0;

    float temperature(NAN), humidity(NAN), pressure(NAN);
    // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
    uint8_t pressureUnit(B001);
    bme.read(pressure, temperature, humidity, true, pressureUnit);

    tempSensor.sensed(now, temperature);
    humidSensor.sensed(now, humidity);
    pressSensor.sensed(now, pressure);
    lightSensor.sensed(now, light*100);
    
    lastLoop = now;
  }
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;

  Homie_setFirmware("envsense", "1.3.0"); // The underscore is not a typo! See Magic bytes
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
  Homie.setup();
}

void loop() {
  Homie.loop();
}