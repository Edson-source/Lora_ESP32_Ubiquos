#include <LoRa.h>
#include "boards.h"
#include <DHT.h>                // Biblioteca DHT
#include <SPI.h>                // SSD1396
#include <Wire.h>               // SSD1396
#include <Adafruit_Sensor.h>

/* Defines do projeto */
#define REED 2           // pin onde o sensor magnetico esta conectado
#define DHTPIN 4         // pin onde dht esta conectado
#define WindSensorPin 0 // The pin location of the anemometer sensor

#define DIAMETRO 125       // diametro interno do balde
#define RAIO 6.25           // raio interno do balde
#define VOLUME 3.05         // volume da bascula (em cm3) (1cm3 == 1ml) (1ml == 1000mm3)
  
// Defina o índice para cada valor de sensor
#define TEMPERATURE_INDEX 0
#define HUMIDITY_INDEX 1
#define RAIN_INDEX 2
#define WIND_SPEED_INDEX 3

#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

unsigned long lastSend;

// Variáveis DHT
float temperatura_lida = 0.0;
float umidade_lida = 0.0;

// Variáveis pluviometro:
int val = 0;
int old_val = 0;
volatile unsigned long REEDCOUNT = 0;

// --- Constantes ---
const float pi = 3.14159265;     //Número de pi
int period = 5000;               //Tempo de medida(miliseconds)
int delaytime = 2000;            //Invervalo entre as amostras (miliseconds)
int radius = 240;                //Raio do anemometro(mm)

volatile unsigned long Rotations = 0; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime = 0; // Timer to avoid contact bounce in interrupt routine
volatile unsigned long ContactBounce = 0;

unsigned long RPM = 0;            //Rotações por minuto
float speedwind = 0;             //Velocidade do vento (km/h)
float windspeed = 0;             //Velocidade do vento (m/s)

float volume_coletado;
// This is the function that the interrupt calls to increment the rotation count
void IRAM_ATTR isr_rotation () {
  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
    // Serial.print("funcao rotacao");
}

// This is the function that the interrupt calls to increment the rotation count
void IRAM_ATTR isr_rain () {
  if ((millis() - ContactBounce) > 50 ) { // debounce the switch contact.
    REEDCOUNT = REEDCOUNT + 1;              // Adiciona 1 à cntagem de pulsos
    ContactBounce = millis();
    // Serial.println("funcao interrupcao chuva");
  }
}

//Função para calcular o RPM
void RPMcalc() {
  Rotations = Rotations/37;         //37 Dentes no interior do anemômetro
  RPM = ((Rotations) * 60) / (period / 1000); // Calculate revolutions per minute (RPM)
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
    
    float area_recipiente = 3.14159265 * (RAIO * RAIO); // área da seção transversal do recipiente em cm²
    float volume_por_virada = (VOLUME/area_recipiente);
    volume_coletado = (REEDCOUNT * volume_por_virada) * 10; // volume total coletado em cm³



    Serial.print("Viradas: ");
    Serial.println(REEDCOUNT);

    Serial.print("Chuva: ");
    Serial.print (volume_coletado);
    Serial.println(" mm");
}

void get_wind(){
    Rotations = 0; // Set Rotations count to 0 ready for calculations

    sei(); // Enables interrupts
    delay(period); // Wait 5 seconds to average
    cli(); // Disable interrupts

    Serial.print("Rotações: ");
    Serial.println(Rotations);

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

void send_packet_temp(){
  // send packet
    LoRa.beginPacket();
    LoRa.print("T ");
    LoRa.print(temperatura_lida );
    LoRa.endPacket();
};

void send_packet_umi(){
  // send packet
    LoRa.beginPacket();
    LoRa.print("U ");
    LoRa.print(umidade_lida );
    LoRa.endPacket();
};

void send_packet_pluv(){
  // send packet
    LoRa.beginPacket();
    LoRa.print("V ");
    LoRa.print(volume_coletado );
    LoRa.endPacket();
};

void send_packets(){
  byte packet[4];

  packet[0] = (byte)temperatura_lida;
  packet[1] = (byte)umidade_lida;
  packet[2] = (byte)volume_coletado;
  packet[3] = (byte)windspeed;

  LoRa.beginPacket();
  LoRa.write(packet, sizeof(packet));
  LoRa.endPacket();
};

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);
    
    /* Inicializa sensor de temperatura e umidade relativa do ar */
    dht.begin();
    lastSend = 0;

    Serial.println("LoRa Sender");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
        // while (1);
    }                              //DEFINE BRILHO ('0' PARA MAXIMO E '1' PARA MINIMO)

    pinMode(REED, INPUT_PULLUP);
    pinMode(WindSensorPin, INPUT_PULLUP);
    attachInterrupt(REED, isr_rain, FALLING);
    attachInterrupt(WindSensorPin, isr_rotation, FALLING);
}

void loop()
{

    // adicionar calculo_rain que calcula o valor do pluviometro e ai chama o get_rain pra enviar pro jaosn no outro lora
    get_temp();
    get_umi();
    get_rain();
    get_wind();
    if (millis() - lastSend > 10000){
      Serial.println("Sending packet !!!");
      
      // send_packet_temp();
      // delay(20);
      // send_packet_umi();
      // delay(20);
      // send_packet_pluv();
      // delay(20);
      send_packets();

      lastSend = millis();
    }
}
