#include <ArduinoJson.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//const char* ssid = "in-berlin-events";
//const char* password = "ADAkqXXxevuS";
//const char* mqtt_server = "test.mosquitto.org";
const char* ssid = "mehzilla";
const char* password = "bananasplit";
const char* mqtt_server = "192.168.1.12";

const char* mqtt_name = "envsense-v1.1";
const char* mqtt_user = "iot";
const char* mqtt_pass = "internetofbanana";
//long reportInterval = 1000;
long reportInterval = 30000;

WiFiClient espClient;
PubSubClient client(espClient);
char msg[200];

int ldrPin = A0;

#include <BME280I2C.h>
BME280I2C bme;
// Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

//#include <OneWire.h>
//#include <DallasTemperature.h>
//#define ONE_WIRE_TEMP D4
//OneWire oneWire(ONE_WIRE_TEMP);
//DallasTemperature tempSensor(&oneWire);

float lightToLux(float val) {
  //return 121.0329 - (-0.0004676414 / -0.02646726) * (1.0 - pow(2.718281828459045, 0.02646726 * val));
  //return 3.710976e-29*pow(val, 12.01577);
  return 2212302000 + (146.1507 - 2212302000)/(1 + pow(val/1335.899, 12.53834));
}

void setup() {
  pinMode(A0, INPUT);
  
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(mqtt_name);
  
  while (!bme.begin()) {  
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
  //tempSensor.begin();
  
  setupWifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setupWifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println(" done");
  Serial.print("IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.println("\n");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Connecting to MQTT... ");
    // Attempt to connect
    if (client.connect(mqtt_name, mqtt_user, mqtt_pass)) {
      Serial.println("done\n");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

long lastReport = 0;

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  
  if (now - lastReport > reportInterval) {
    lastReport = now;

    /*
    // One wire temp
    tempSensor.requestTemperatures();
    float tempC = tempSensor.getTempCByIndex(0);
  
    if (tempC != DEVICE_DISCONNECTED_C) {
      snprintf(msg, 50, "%.2f", tempC);
      client.publish("envsense/temp", msg);
      Serial.println(msg);
    }
    */

    int lightRaw = analogRead(ldrPin);
    float light = lightRaw/1023.0;
    float lux = lightToLux(lightRaw);

    float temperature(NAN), humidity(NAN), pressure(NAN);
    uint8_t pressureUnit(B001);
    // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
    bme.read(pressure, temperature, humidity, true, pressureUnit);

    if (!isnan(temperature) && !isnan(humidity) && !isnan(pressure)) {
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      char pres[16];
      snprintf(pres, sizeof(pres), "%.6f", pressure);
      json.set("pressure", RawJson(pres));
      json.set("temperature", temperature, 2);
      json.set("humidity", humidity, 2);
      json.set("light", light*100, 2);
      char lx[16];
      snprintf(lx, sizeof(lx), "%.0f", lux);
      json.set("lux", RawJson(lx));
      char msg[200];
      json.printTo(msg, sizeof(msg));
      client.publish("envsense/sensors", msg);
    }
  }
  
  delay(1000);
  
}


