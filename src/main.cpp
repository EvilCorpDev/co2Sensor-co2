#include <Esp.h>
#include <FS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>

AsyncWebServer server(80);
HardwareSerial co2Sensor(1);
byte co2Request[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
unsigned char co2Response[9];
unsigned int ppm;

const char* WIFI_SSID; //set wifi name here
const char* WIFI_PASSWORD; //set wifi pass here

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

void setup() {
    Serial.begin(115200);
    connectWifi();
    co2Sensor.begin(9600, SERIAL_8N1, 16, 17);

    //urlMapping
    co2PpmHandler();
    server.onNotFound(notFound);
    server.begin();
}

void loop() {
    readCo2SensorValueToCo2Response();
    printHexRespond();
    calculateAndPrintPpm();
    delay(10000);
}