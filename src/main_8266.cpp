/*#include <Esp.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SoftwareSerial.h> //Программный UART для MH-Z19

AsyncWebServer server(80);
SoftwareSerial co2Sensor(5, 4);
byte co2Request[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte co2Response[9];
unsigned int ppm;

const char* WIFI_SSID = "The_new_home";
const char* WIFI_PASSWORD = "lalochka_palochka";

const char* JSON_RESP = "{\"ppm\":%d}";

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "application/json", "{\"error\":\"Page not found\"}");
}

void testHandler() {
    server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request){
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

//pio run --target upload

void setup() {
    Serial.begin(9600);
    connectWifi();
    co2Sensor.begin(9600);

    //urlMapping
    testHandler();
    server.onNotFound(notFound);
    server.begin();
}

void loop() {
    for (int i = 0; i < 9; ++i) {
        co2Sensor.write(co2Request[i]); //Запрашиваем данные у MH-Z19
    }
    delay(100);
    memset(co2Response, 0, 9); //Чистим переменную от предыдущих значений
    //co2Response = co2Sensor.readBytes(9, co2Response); //Записываем свежий ответ от MH-Z19
    for (int i = 0; i < 9; ++i) {
        co2Response[i] = co2Sensor.read();
        Serial.print(co2Response[i], HEX);
        //Serial.print(co2Response[i]);
        Serial.print(" ");
    }
    unsigned int responseHigh = (unsigned int) co2Response[2];
    unsigned int responseLow = (unsigned int) co2Response[3];
    ppm = (256*responseHigh) + responseLow;
    Serial.print("\n");
    Serial.println(ppm);
    delay(10000);
}*/