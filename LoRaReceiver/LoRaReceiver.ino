#include <LoRa.h>
#include "boards.h"
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>                // SSD1396
#include <Wire.h>               // SSD1396
#include <Adafruit_Sensor.h>

namespace rainGaugeEspServer
{
	/// @brief Converts a JSON document to its string representation.
	/// @tparam JSON_SIZE The size of the JSON document.
	/// @param json The JSON document to be converted.
	/// @return The string representation of the JSON document.
	template<size_t JSON_SIZE>
	auto fromJsonToString(StaticJsonDocument<JSON_SIZE> const& json) -> String;
}

namespace rainGaugeEspServer
{
	template<size_t JSON_SIZE>
	auto fromJsonToString(StaticJsonDocument<JSON_SIZE> const& json) -> String
	{
		auto string = String();
		serializeJson(json, string);
		return string;
	}
}

using rainGaugeEspServer::fromJsonToString;

// #define WIFI_SSID  "EdsonWifi"
// #define WIFI_PASS  "cjxe6131"


#define WIFI_SSID  "lab120"
#define WIFI_PASS  "labredes120"

const char* SERVER_URL = "http://192.168.0.112:8000/routersloraip/";

constexpr auto webServerPort = 80;

constexpr auto jsonDocumentSize = size_t(10000);

WebServer webServer(webServerPort);

auto json_temp = StaticJsonDocument<jsonDocumentSize>();
auto json_umi = StaticJsonDocument<jsonDocumentSize>();
auto json_pluv = StaticJsonDocument<jsonDocumentSize>();
auto json_anem = StaticJsonDocument<jsonDocumentSize>();

bool auxStatusT = false;
bool auxStatusU = false;
bool auxStatusV = false;
bool auxStatusA = false;

auto counter_temp = 0;
auto counter_umi = 0;
auto counter_pluv = 0;
auto counter_anem = 0;

String recv_temp = "";
String recv_umid = "";
String recv_anem = "";
String recv_pluv = "";

String auxtempStatus;
String auxumidStatus;
String auxpluvStatus;
String auxanemStatus;

void handleRequest_temp();
void handleRequest_umid();
void handleRequest_pluv();
void handleRequest_anem();

void compare_temp();
void compare_umi();
void compare_pluv();
void compare_anem();

void setUpWiFi() {
	WiFi.begin(WIFI_SSID, WIFI_PASS);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(1000);
		Serial.printf("Connecting to Wi-Fi network \"%s\"...\n", WIFI_SSID);
	}

	Serial.print("Connected to Wi-Fi network at ");
  Serial.print("Endereço IP: ");
	Serial.print(WiFi.localIP());
	Serial.print(".\n");
}

// Envia mensagem json::: 
void handleRequest_temp() {
	auto const clientIPAddress = webServer.client().remoteIP();

	Serial.print("A request was made from: ");
	Serial.print(clientIPAddress);
	Serial.print(".\n");
  
  //chamar a função "dado" criar ela compare vai virar dado
  compare_temp();

  webServer.send(200, "application/json", fromJsonToString(json_temp));
}

// Envia mensagem json::: 
void handleRequest_umid() {
	auto const clientIPAddress = webServer.client().remoteIP();

	Serial.print("A request was made from: ");
	Serial.print(clientIPAddress);
	Serial.print(".\n");
  
  //chamar a função "dado" criar ela compare vai virar dado
  compare_umi();

  webServer.send(200, "application/json", fromJsonToString(json_umi));
}

// Envia mensagem json::: 
void handleRequest_pluv() {
	auto const clientIPAddress = webServer.client().remoteIP();

	Serial.print("A request was made from: ");
	Serial.print(clientIPAddress);
	Serial.print(".\n");
  
  //chamar a função "dado" criar ela compare vai virar dado
  compare_pluv();

  webServer.send(200, "application/json", fromJsonToString(json_pluv));
}

// Envia mensagem json::: 
void handleRequest_anem() {
	auto const clientIPAddress = webServer.client().remoteIP();

	Serial.print("A request was made from: ");
	Serial.print(clientIPAddress);
	Serial.print(".\n");
  
  //chamar a função "dado" criar ela compare vai virar dado
  compare_anem();

  webServer.send(200, "application/json", fromJsonToString(json_anem));
}

// crie um compare pra cada um (compare tem que virar nome dado())
void compare_temp() {
    if (auxStatusT == true){
      auto object_temp = json_temp["temp_dict"].createNestedObject();
      object_temp["id"] = counter_umi;
      object_temp["temp_status"] = auxtempStatus;
      // handlePost();
      counter_temp++;
      auxStatusT = false;
    }
}

// crie um compare pra cada um (compare tem que virar nome dado())
void compare_umi() {
    if (auxStatusU == true){
      auto object_umi = json_umi["umi_dict"].createNestedObject();
      object_umi["id"] = counter_umi;
      object_umi["umi_status"] = auxumidStatus;
      // handlePost();
      counter_umi++;
      auxStatusU = false;
    }
}

// crie um compare pra cada um (compare tem que virar nome dado())
void compare_pluv() {
    if (auxStatusV == true){
      auto object_pluv = json_pluv["pluv_dict"].createNestedObject();
      object_pluv["id"] = counter_pluv;
      object_pluv["pluv_status"] = auxpluvStatus;
      // handlePost();
      counter_pluv++;
      auxStatusV = false;
    } 
}

// crie um compare pra cada um (compare tem que virar nome dado())
void compare_anem() {
    if (auxStatusA == true){
    auto object_anem = json_anem["anem_dict"].createNestedObject();
      object_anem["id"] = counter_anem;
      object_anem["anem_status"] = auxanemStatus;
      // handlePost();
      counter_anem++;
      auxStatusA = false;
    }
}

void setUpWebServer() {

  webServer.on("/temp", handleRequest_temp);
  webServer.on("/umid", handleRequest_umid);
  webServer.on("/pluv", handleRequest_pluv);
  webServer.on("/anem", handleRequest_anem);
  webServer.begin();

	Serial.print("Web server started at: ");
	Serial.print(WiFi.localIP());
	Serial.printf(":%d.\n", webServerPort);
}

auto setUpSerial() -> void
{
	constexpr auto baudRate = 115200;
	Serial.begin(baudRate);
}

void receive_packets(){
   int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.print("Received packet '");

        byte packet[packetSize];
        for (int i = 0; i < packetSize; i++) {
          packet[i] = LoRa.read();
        }

        // Processamento do pacote recebido
        float temperature = packet[0];
        float humidity = packet[1];
        float rain = packet[2];
        float windSpeed = packet[3];
        
        // read packet
        recv_temp = "";
        recv_temp = temperature;
        auxtempStatus = recv_temp;
        auxStatusT = true;
        handleRequest_temp();
      
        recv_umid = "";
        recv_umid = humidity;
        auxumidStatus = recv_umid;
        auxStatusU = true;
        handleRequest_umid();

        recv_pluv = "";
        recv_pluv = rain;
        auxpluvStatus = recv_pluv;
        auxStatusV = true;
        handleRequest_pluv();

        recv_anem = "";
        recv_anem = windSpeed;
        auxanemStatus = recv_anem;
        auxStatusA = true;
        handleRequest_anem();
        } else {
          auxStatusT = false;
          auxStatusU = false;
          auxStatusV = false;
          auxStatusA = false;
        }

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
}

void setup(){
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    Serial.println("LoRa Receiver");

    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

  json_temp.createNestedArray("temp_dict");
  json_umi.createNestedArray("umi_dict");
  json_pluv.createNestedArray("pluv_dict");
  json_anem.createNestedArray("anem_dict");

  setUpSerial();
  setUpWiFi();
  setUpWebServer();
  
  // Fazer post do ip do lora para o django se conectar

  // Pega ip do lora
  auto const clientIPAddress = WiFi.localIP();
  
  // Criar uma string com o tamanho necessário
  String postData = clientIPAddress.toString() + ":" + String(webServerPort);
  
  // Instancia de httpclient
  HTTPClient http;

  // Begin com o servidor do django 
  http.begin("http://192.168.0.112:8000/routersloraip/");
  http.addHeader("Content-Type", "application/json");

  // Constroi a string a ser enviada  
  DynamicJsonDocument jsonPayload(50);
  jsonPayload["url"] = postData;

  char buffer[50];
  serializeJson(jsonPayload, buffer);
  
  printf("Valor como string: %s\n", buffer);

  int httpResponseCode = http.POST(buffer);

  if (httpResponseCode == HTTP_CODE_OK) {
  String response = http.getString();
  Serial.println(httpResponseCode);
  Serial.println(response);
} else {
  Serial.print("Erro na requisição: ");
  Serial.println(httpResponseCode);
}


  http.end();
}

void loop()
{
    // try to parse packet
  receive_packets();

  webServer.handleClient();

#ifdef HAS_DISPLAY
        if (u8g2) {
            u8g2->clearBuffer();
            char buf[256];
            // u8g2->drawStr(0, 12, "Received OK!");
            // snprintf(buf, sizeof(buf), "RSSI:%i", LoRa.packetRssi());
            // u8g2->drawStr(0, 40, buf);
            if (-120 <= LoRa.packetRssi() && LoRa.packetRssi() < -80) {
              snprintf(buf, sizeof(buf), "Sinal Fraco (%i)", LoRa.packetRssi());
              u8g2->drawStr(0, 12, buf);
            }
            else if (-80 <= LoRa.packetRssi() && LoRa.packetRssi() < -40) {
              snprintf(buf, sizeof(buf), "Sinal Medio (%i)", LoRa.packetRssi());
              u8g2->drawStr(0, 12, buf);
            }
            else if (-40 <= LoRa.packetRssi() && LoRa.packetRssi() < 0 ) {
              snprintf(buf, sizeof(buf), "Sinal Forte (%i)", LoRa.packetRssi());
              u8g2->drawStr(0, 12, buf);
            }

            // u8g2->clearDisplay();
            // u8g2->clearBuffer();
            // Temperature display
            snprintf(buf, sizeof(buf), "T: %s", recv_temp.c_str());
            u8g2->drawStr(0, 40, buf);

            // Humidity display
            snprintf(buf, sizeof(buf), "U: %s", recv_umid.c_str());
            u8g2->drawStr(64, 56, buf);

            // Pluviometer display
            snprintf(buf, sizeof(buf), "P: %s", recv_pluv.c_str());
            u8g2->drawStr(64, 40, buf);

            // Anemometer display
            snprintf(buf, sizeof(buf), "A: %s", recv_anem.c_str());
            u8g2->drawStr(0, 56, buf);
            u8g2->sendBuffer();
        }
#endif
}