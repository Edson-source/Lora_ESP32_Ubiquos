#include <LoRa.h>
#include "boards.h"
#include <DHT.h>                // Biblioteca DHT
#include <SPI.h>                // SSD1396
#include <Wire.h>               // SSD1396
#include <Adafruit_Sensor.h>

/* Defines do projeto */
#define REED 34           // pin onde o sensor magnetico esta conectado
#define DHTPIN 4         // pin onde dht esta conectado
#define WindSensorPin (14) // The pin location of the anemometer sensor

#define DIAMETRO 125       // diametro interno do balde
#define RAIO 6.25           // raio interno do balde
#define VOLUME 3.72         // volume da bascula (em cm3) (1cm3 == 1ml) (1ml == 1000mm3)
  
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// Variáveis DHT
float temperatura_lida = 0.0;
float umidade_lida = 0.0;

// Variáveis pluviometro:
int val = 0;
int old_val = 0;
int REEDCOUNT = 0;

// --- Constantes ---
const float pi = 3.14159265;     //Número de pi
int period = 5000;               //Tempo de medida(miliseconds)
int delaytime = 2000;            //Invervalo entre as amostras (miliseconds)
int radius = 240;                //Raio do anemometro(mm)

volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime = 0; // Timer to avoid contact bounce in interrupt routine

unsigned long RPM = 0;            //Rotações por minuto
float speedwind = 0;             //Velocidade do vento (km/h)
float windspeed = 0;             //Velocidade do vento (m/s)

float volume_coletado;
// This is the function that the interrupt calls to increment the rotation count
IRAM_ATTR void isr_rotation () {
  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}

//Função para calcular o RPM
void RPMcalc() {

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

    send_packet_temp();
}

void get_umi(){
    umidade_lida = dht.readHumidity();  

    Serial.print("U: ");                    //ESCREVE O TEXTO NO DISPLAY   
    Serial.println(umidade_lida);
    send_packet_umi();
}

void get_rain(){
    bool val = digitalRead(REED);      // Lê o Status do Reed Switch

    if ((val == LOW && old_val == HIGH)) {    // Verefica se o Status mudou
        delay(10);                              // Atraso colocado para lidar com qualquer "salto" no switch.
        REEDCOUNT = REEDCOUNT + 1;              // Adiciona 1 à cntagem de pulsos
        old_val = val;                          //Iguala o valor antigo com o atual
    }
    else {
        old_val = val;               //If the status hasn't changed then do nothing
    }
    
    float area_recipiente = 3.1415 * (RAIO * RAIO); // área da seção transversal do recipiente em cm²
    float volume_por_virada = (VOLUME/area_recipiente);
    volume_coletado = (REEDCOUNT * volume_por_virada) * 10; // volume total coletado em cm³



    Serial.println("Viradas: ");
    Serial.println(REEDCOUNT);

    Serial.println("Chuva: ");
    Serial.print(volume_coletado);
    Serial.println(" mm");

    send_packet_pluv();

}

void get_wind(){

    Rotations = 0; // Set Rotations count to 0 ready for calculations

    sei(); // Enables interrupts
    delay(period); // Wait 5 seconds to average
    cli(); // Disable interrupts

    Serial.print("Rotações: ");
    Serial.print(Rotations);

    RPMcalc();
    Serial.print("RPM: ");
    Serial.print(RPM);

    WindSpeed();
    Serial.print("WindSpeed [m/s]: ");
    Serial.print(windspeed);

    SpeedWind();
    Serial.print("WindSpeed [km/h]: ");
    Serial.print(speedwind);

    // send_packet_anem();
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

// void send_packet_anem(){
//   // send packet
//     LoRa.beginPacket();
//     LoRa.print("Velocidade do vento ");
//     LoRa.print(windspeed);
//     LoRa.endPacket();
// };

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);
    
    /* Inicializa sensor de temperatura e umidade relativa do ar */
    dht.begin();
    
    Serial.println("LoRa Sender");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }                              //DEFINE BRILHO ('0' PARA MAXIMO E '1' PARA MINIMO)

    pinMode(REED, INPUT_PULLUP);
    //pinMode(WindSensorPin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, RISING);
}

void loop()
{
    Serial.println("Sending packet !!!");

    /* Garante que a conexão wi-fi esteja ativa */
    get_temp();
    get_umi();
    get_rain();
    //get_wind();
}
