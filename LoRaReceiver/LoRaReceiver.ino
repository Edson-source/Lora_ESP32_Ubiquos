#include <LoRa.h>
#include "boards.h"
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include <HTTPClient.h>


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

#define WIFI_SSID  "EdsonWifi"
#define WIFI_PASS  "cjxe6131"

constexpr auto webServerPort = 80;

constexpr auto jsonDocumentSize = size_t(10000);

WebServer webServer(webServerPort);

auto json_temp = StaticJsonDocument<jsonDocumentSize>();
auto json_umi = StaticJsonDocument<jsonDocumentSize>();
auto json_pluv = StaticJsonDocument<jsonDocumentSize>();
auto json_anem = StaticJsonDocument<jsonDocumentSize>();

auto counter_temp = 0;
auto counter_umi = 0;
auto counter_pluv = 0;
auto counter_anem = 0;

String auxtempStatus;
String auxumiStatus;
String auxpluvStatus;
String auxanemStatus;

void handleRequest_temp();
void handleRequest_umi();
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
void handleRequest_umi() {
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

    auto object_temp = json_temp["temp_dict"].createNestedObject();
    object_temp["id"] = counter_umi;
    object_temp["temp_status"] = auxtempStatus;
    // handlePost();
    counter_temp++;
    
}

// crie um compare pra cada um (compare tem que virar nome dado())
void compare_umi() {

    auto object_umi = json_umi["umi_dict"].createNestedObject();
    object_umi["id"] = counter_umi;
    object_umi["umi_status"] = auxumiStatus;
    // handlePost();
    counter_umi++;
}

// crie um compare pra cada um (compare tem que virar nome dado())
void compare_pluv() {

    auto object_pluv = json_pluv["pluv_dict"].createNestedObject();
    object_pluv["id"] = counter_pluv;
    object_pluv["pluv_status"] = auxpluvStatus;
    // handlePost();
    counter_pluv++;
    
}

// crie um compare pra cada um (compare tem que virar nome dado())
void compare_anem() {

    auto object_anem = json_anem["anem_dict"].createNestedObject();
    object_anem["id"] = counter_anem;
    object_anem["anem_status"] = auxanemStatus;
    // handlePost();
    counter_anem++;
    
}

void setUpWebServer() {

  webServer.on("/temp", handleRequest_temp);
  webServer.on("/umi", handleRequest_umi);
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


void receive_packet_temp(){
   int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.print("Received packet '");

        String recv_temp = "";
        // read packet
        // add um if
        char recv = (char)LoRa.read();
        if (recv == 'T')
        {
          while (LoRa.available()) {
              recv_temp += (char)LoRa.read();
          }
        }
        auxtempStatus = recv_temp;
        // chamar a funcção handleRequest
        handleRequest_temp();
        // Serial.println(recv);
        delay(2000);

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
    }
}

void receive_packet_umi(){
   int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.print("Received packet '");

        String recv_umi = "";
        // read packet
        // add um if 
        // se a recv == temp { atualiza variavel global temp chama hendle reqquest }
        char recv = (char)LoRa.read();
        if (recv == 'U') {
          while (LoRa.available()) {
              recv_umi += (char)LoRa.read();
          }
        }
        auxumiStatus = recv_umi;
        // chamar a funcção handleRequest
        handleRequest_umi();
        // Serial.println(recv);
        delay(2000);

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
    }
}

void receive_packet_pluv(){
   int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.print("Received packet '");

        String recv_pluv = "";
        // read packet
        // add um if 
        // se a recv == temp { atualiza variavel global temp chama hendle reqquest }
        char recv = (char)LoRa.read();
        if (recv == 'V') {
          while (LoRa.available()) {
              recv_pluv += (char)LoRa.read();
          }
        }
        auxpluvStatus = recv_pluv;
        // chamar a funcção handleRequest
        handleRequest_pluv();
        // Serial.println(recv);
        delay(2000);

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
    }
}

void receive_packet_anem(){
   int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // received a packet
        Serial.print("Received packet '");

        String recv_anem = "";
        // read packet
        // add um if 
        // se a recv == temp { atualiza variavel global temp chama hendle reqquest }
        char recv = (char)LoRa.read();
        if (recv == 'A') {
          while (LoRa.available()) {
              recv_anem += (char)LoRa.read();
          }
        }
        auxanemStatus = recv_anem;
        // chamar a funcção handleRequest
        handleRequest_anem();
        // Serial.println(recv);
        delay(2000);

        // print RSSI of packet
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
    }
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
    
}

void loop()
{
    // try to parse packet
  receive_packet_temp();
  receive_packet_umi();
  receive_packet_pluv();
  receive_packet_anem();

  webServer.handleClient();

#ifdef HAS_DISPLAY
        if (u8g2) {
            u8g2->clearBuffer();
            char buf[256];
            u8g2->drawStr(0, 12, "Received OK!");
            u8g2->drawStr(0, 26, recv.c_str());
            snprintf(buf, sizeof(buf), "RSSI:%i", LoRa.packetRssi());
            u8g2->drawStr(0, 40, buf);
            snprintf(buf, sizeof(buf), "SNR:%.1f", LoRa.packetSnr());
            u8g2->drawStr(0, 56, buf);
            u8g2->sendBuffer();
        }
#endif
}


// void handlePost() {
//   if (WiFi.status() == WL_CONNECTED) {
//     WiFiClient client;
//     HTTPClient http;

//     http.begin(client, "http://127.0.0.1:8000/lamp/"); // URL da requisição POST
//     http.addHeader("Content-Type", "application/json"); // Tipo de conteúdo

//     String requestBody = "{\"status\": " + String(aux) + "}"; // Corpo da requisição
//     int httpResponseCode = http.POST(requestBody); // Envia a requisição e recebe a resposta

//     if (httpResponseCode > 0) {
//       String response = http.getString(); // Lê a resposta
//       Serial.println(httpResponseCode);
//       Serial.println(response);
//     } else {
//       Serial.println("Erro na requisição POST");
//     }

//     http.end(); // Encerra a conexão
//   }
// }

// void handleRootPost() {
//   String html = "<html><body><h1>Controle de Relé</h1>";
//   html += "<form method='POST' action='/relay'>";
//   html += "<input type='text' name='relay' value=''>Ligar<br>";
//   html += "<input type='submit' value='Enviar'>";
//   html += "</form></body></html>";
//   webServer.send(200, "text/html", html);
// }

// void handleRelay() {
//   String relayStatus = webServer.arg("relay");

//   if (relayStatus == "1") {
//     digitalWrite(RELAY_PIN, HIGH);
//     ledState = HIGH;
//     auxledStatus = 1;
//   } else if (relayStatus == "0") {
//     auxledStatus = 0;
//     digitalWrite(RELAY_PIN, LOW);
//     ledState = LOW;
//   }

//   handleRequest();

//   webServer.send(200, "text/plain", (ledState == HIGH) ? "Ligado" : "Desligado");
// }