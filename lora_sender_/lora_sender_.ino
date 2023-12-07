#include <LoRa.h>
#include "boards.h"
#include <DHT.h>                // Biblioteca DHT
#include <SPI.h>                // SSD1396
#include <Wire.h>               // SSD1396
#include <Adafruit_Sensor.h>
#include "painlessMesh.h"

/* Defines do projeto */
#define REED          2    // pin onde o sensor magnetico esta conectado
#define DHTPIN        4    // pin onde dht esta conectado
#define WindSensorPin 0    // The pin location of the anemometer sensor
#define NIVEL         33   // pin onde o sensor de pressão está instalado
// #define DIRECAO       35   // pin onde o sensor de pressão está instalado


#define DIAMETRO 125       // diametro interno do balde
#define RAIO     6.25      // raio interno do balde
#define VOLUME   3.05      // volume da bascula (em cm3) (1cm3 == 1ml) (1ml == 1000mm3)

#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

#define VERSION 1

// Configura a rede mesh
#define MESH_PREFIX     "TCC_Edson"
#define MESH_PASSWORD   "coxinha123"
#define MESH_PORT       5555

Scheduler userScheduler; // escalonador para tarefas
painlessMesh  mesh;    

unsigned long lastSend;

float id = 1;

// Variáveis DHT
float temperatura_lida = 0;
float umidade_lida = 0;

// Variáveis pluviometro:
int val = 0;
int old_val = 0;
volatile unsigned long REEDCOUNT = 0;
float volume_coletado;

// Variaveis Sensor Nivel
float value = 0;
float metros = 0;


// Variable definitions
unsigned int Sample = 0;   // Sample number
unsigned int counter = 0; // magnet counter for sensor
unsigned long RPM = 0;            //Rotações por minuto
float speedwind = 0;             //Velocidade do vento (km/h)
float windspeed = 0;             //Velocidade do vento (m/s)

// Variaveis Anemometro
const float pi = 3.14159265;     //Número de pi
int period = 5000;               //Tempo de medida(miliseconds)
int delaytime = 2000;            //Invervalo entre as amostras (miliseconds)
int radius = 120;                //Raio do anemometro(mm)

// ------Variaveis dados recebidos--------
float recv_temp = 0;
float recv_umid = 0;
float recv_anem = 0;
float recv_pluv = 0;
float recv_nivel = 0;
float recv_id = 0;

float recv_temp_ant = 0;

void sendMessage();

// Define tarefa de envio de mensagens
Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );

void sendMessage() {
  String msg = "Hello from node ";
  msg += mesh.getNodeId();
  mesh.sendBroadcast( msg );
  taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

void MeshInit() {
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

// This is the function that the interrupt calls to increment the rotation count
void IRAM_ATTR isr_rain () {
  if ((millis() - ContactBounce) > 50 ) { // debounce the switch contact.
    REEDCOUNT = REEDCOUNT + 1;              // Adiciona 1 à cntagem de pulsos
    ContactBounce = millis();
    // Serial.println("funcao interrupcao chuva");
  }
}

void addcount(){
  counter++;
} 

// Measure wind speed
void windvelocity(){
  speedwind = 0;
  windspeed = 0;
  
  counter = 0;  
  attachInterrupt(0, addcount, RISING);
  unsigned long millis();       
  long startTime = millis();
  while(millis() < startTime + period) {
  }
}

//Função para calcular o RPM
void RPMcalc() {
  RPM = ((counter) * 60) / (period / 1000); // Calculate revolutions per minute (RPM)
}

//Velocidade do vento em m/s
void WindSpeed() {
  windspeed = ((4 * pi * radius * RPM) / 60) / 1000; //Calcula a velocidade do vento em m/s
} //end WindSpeed

//Velocidade do vento em km/h
void SpeedWind() {
  speedwind = (((4 * pi * radius * RPM) / 60) / 1000) * 3.6; //Calcula velocidade do vento em km/h
} //end SpeedWind

void get_temp(){
    temperatura_lida = dht.readTemperature();

    Serial.print("T: ");                    //ESCREVE O TEXTO NO DISPLAY
    Serial.println(temperatura_lida);
}

void get_umi(){
    umidade_lida = dht.readHumidity();  

    Serial.print("U: ");                    //ESCREVE O TEXTO NO DISPLAY   
    Serial.println(umidade_lida);
}

void get_rain(){
    // float area_recipiente = 3.14159265 * (RAIO * RAIO); // área da seção transversal do recipiente em cm²
    // float volume_por_virada = (VOLUME/area_recipiente);
    volume_coletado = (REEDCOUNT * 0.25) * 10; // volume total coletado em cm³

    Serial.print("Viradas: ");
    Serial.println(REEDCOUNT);

    Serial.print("Chuva: ");
    Serial.print (volume_coletado);
    Serial.println(" mm");
}

void get_wind(){
    Sample++;
    Serial.print(Sample);
    Serial.print(": Start measurement...");
    windvelocity();
    Serial.println("   finished.");
    Serial.print("Counter: ");
    Serial.print(counter);
  
    RPMcalc();
    Serial.print("RPM: ");
    Serial.println(RPM);

    WindSpeed();
    Serial.print("WindSpeed [m/s]: ");
    Serial.println(windspeed);

    SpeedWind();
    Serial.print("WindSpeed [km/h]: ");
    Serial.println(speedwind);
}

void get_nivel() {
  value = analogRead(NIVEL);
  value = (float)value * 5 / 1023;

  Serial.printf("Tensão [V]: "); // Printa no monitor o valor lido pelo sensor
  Serial.println(val);

  metros = 0.0119*value - 0.2449; //Esse é o algoritmo que fizemos baseado nos dados que obtemos de forma "analogica", anotando os valores das tensões medidadas conforme aumentamos a profundidada do sensor, então essa função não é puramente linear
  Serial.printf("Metros:");     // Printa o valor convertido em metros
  Serial.println(metros);
}

void send_packets(){
  // Crie um array de bytes para armazenar os dados
  byte buffer[sizeof(float) * 6];

  memcpy(buffer, &temperatura_lida, sizeof(float));
  memcpy(buffer + sizeof(float), &umidade_lida, sizeof(float));
  memcpy(buffer + sizeof(float) * 2, &volume_coletado, sizeof(float));
  memcpy(buffer + sizeof(float) * 3, &windspeed, sizeof(float));
  memcpy(buffer + sizeof(float) * 4, &metros, sizeof(float));
  memcpy(buffer + sizeof(float) * 5, &id, sizeof(float));


  LoRa.beginPacket();
  LoRa.write(buffer, sizeof(float) * 6);
  LoRa.endPacket();
};

void receive_packets(){
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println("Packets Received");

    byte buffer_recv[sizeof(float) * 6];

    for (int i = 0; i < packetSize; i++) {
      buffer_recv[i] = LoRa.read();
    }

    float temperature, humidity, rain, windSpeed, nivel, id_;

    memcpy(&temperature, buffer_recv, sizeof(float));
    memcpy(&humidity, buffer_recv + sizeof(float), sizeof(float));
    memcpy(&rain, buffer_recv + sizeof(float) * 2, sizeof(float));
    memcpy(&windSpeed, buffer_recv + sizeof(float) * 3, sizeof(float));
    memcpy(&nivel, buffer_recv + sizeof(float) * 3, sizeof(float));
    memcpy(&id_, buffer_recv + sizeof(float) * 4, sizeof(float));
    
    // read packet
    recv_temp = temperature;
    recv_umid = humidity;
    recv_pluv = rain;
    recv_anem = windSpeed;
    recv_nivel = nivel;
    recv_id = id_;
  }
}

void send_packets_received(){
  // Crie um array de bytes para armazenar os dados
  byte buffer[sizeof(float) * 6];

  if(recv_id < id && (recv_temp != 0.00 || recv_temp != recv_temp_ant)) {
    memcpy(buffer, &recv_temp, sizeof(float));
    memcpy(buffer + sizeof(float), &recv_umid, sizeof(float));
    memcpy(buffer + sizeof(float) * 2, &recv_pluv, sizeof(float));
    memcpy(buffer + sizeof(float) * 3, &recv_anem, sizeof(float));
    memcpy(buffer + sizeof(float) * 4, &recv_nivel, sizeof(float));
    memcpy(buffer + sizeof(float) * 5, &recv_id, sizeof(float));

    LoRa.beginPacket();
    LoRa.write(buffer, sizeof(float) * 6);
    LoRa.endPacket();

    recv_temp_ant = recv_temp;
    }
  

};

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);
    
    /* Inicializa sensor de temperatura e umidade relativa do ar */
    dht.begin();

    /* Inicializa rede mesh*/
    MeshInit();

    lastSend = 0;

    Serial.println("LoRa Sender");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
        // while (1);
    }

    LoRa.setSpreadingFactor(8);           // ranges from 6-12,default 7 see API docs
    
    pinMode(WindSensor, INPUT);
    digitalWrite(WindSensor, HIGH);     //internall pull-up active
    pinMode(REED, INPUT_PULLUP);
    pinMode(NIVEL, INPUT_PULLUP);
    attachInterrupt(REED, isr_rain, FALLING);
}

void loop()
{
    // receive_packets();

    get_temp();
    get_umi();
    get_rain();
    get_nivel();
    if (millis() - lastSend > 5000){
      get_wind();
      Serial.println("Sending packet !!!");

      // send_packets_received();
      delay(100);
      
      send_packets(); // Envio de pacotes através da rede LoraWan

      sendMessage();  // Envio de mensagem através da rede Mesh
      lastSend = millis();
    }
    mesh.update();  // Update para entrada e saida de nodos na rede Mesh
}
