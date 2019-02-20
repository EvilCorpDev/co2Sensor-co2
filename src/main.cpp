#include <Esp.h>
#include <FS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include "Adafruit_BME280.h"

AsyncWebServer server(80);
HardwareSerial co2Sensor(1);
byte co2Request[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned char co2Response[9];
unsigned int ppm;

const char* WIFI_SSID; //set wifi name here
const char* WIFI_PASSWORD; //set wifi pass here

#define ALTITUDE 152.0

#define I2C_SDA 27
#define I2C_SCL 26
#define BME280_ADDRESS 0x76

float temperature = 0;
float humidity = 0;
float pressure = 0;

Adafruit_BME280 bme(I2C_SDA, I2C_SCL);

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "application/json", "{\"error\":\"Page not found\"}");
}

void co2PpmHandler() {
    server.on("/co2-ppm", HTTP_GET, [](AsyncWebServerRequest *request){
        char str[255];
        sprintf(str, "{\"ppm\":%d}", ppm);
        request->send(200, "application/json", str);
    });
}

void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void readCo2SensorValueToCo2Response() {
    co2Sensor.write(co2Request, 9);
    memset(co2Response, 0, 9);
    if(co2Sensor.available()) {
        co2Sensor.readBytes(co2Response, 9);
    }
}

void printHexRespond() {
    Serial.print("Respond: ");
    for(byte i = 0; i < 9; i++) {
        Serial.print(co2Response[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
}

void calculateAndPrintPpm() {
    unsigned int responseHigh = (unsigned int) co2Response[2];
    unsigned int responseLow = (unsigned int) co2Response[3];
    ppm = (256*responseHigh) + responseLow;
    Serial.print("\n");
    Serial.println(ppm);
    Serial.print("\n");
}

void initBmeSensor() {
    bool status = bme.begin(BME280_ADDRESS);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
}

void messureTemperature() {
    temperature = bme.readTemperature();
    Serial.println(temperature);
}

void messureHumidity() {
    humidity = bme.readHumidity();
    Serial.println(humidity);
}

void messurePressure() {
    pressure = bme.readPressure();
    pressure = bme.seaLevelForAltitude(ALTITUDE, pressure);
    pressure = pressure/100.0F;
    Serial.println(pressure);
}

void setup() {
    Serial.begin(115200);
    connectWifi();
    co2Sensor.begin(9600, SERIAL_8N1, 16, 17);

    //urlMapping
    co2PpmHandler();
    server.onNotFound(notFound);
    server.begin();

    initBmeSensor();
}

void loop() {
    readCo2SensorValueToCo2Response();
    printHexRespond();
    calculateAndPrintPpm();

    messureTemperature();
    messureHumidity();
    messurePressure();
    delay(10000);
}