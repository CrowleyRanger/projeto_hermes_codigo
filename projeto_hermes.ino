#include <Wire.h>
int DATA_REQUEST_DELAY = 900000; // 15min // A cada tempo DATA_REQUEST_DELAY os dados sao adquiridos e salvos num arquivo
int loop_num = 1; // Contagem da quantidade de loops executados para aquisição de dados

// SD Card
#include <SD.h>
#include <SPI.h>
File saveFile;
int pinSS = 10;

// LoRa SX1276
//#include <SPI.h>
#include <LoRa.h>

// Sensor SHT22: Temperatura do ar e umidade relativa
#include "DFRobot_SHT20.h" // Importando a biblioteca ./lib/DFRobot_SHT20
DFRobot_SHT20 sht20; // Inicializando objeto do tipo DFRobot_SHT20

// Sensor PB10: Pluviometro
const int REED = 6;
int pb10_val = 0;
int pb10_old_val = 0;
int REEDCOUNT = 0;

// Sensor SV10: Velocidade do vento
#define Hall sensor 2 // Pino digital 2
const float pi = 3.14159265; // Numero pi
int sv10_period = 5000; // Tempo de medida (ms)
int sv10_radius = 147; // Raio do anemometro (mm)
unsigned int sv10_sample = 0; // Numero da amostra
unsigned int sv10_counter = 0; // Contador do magnetometro para o sensor
unsigned int sv10_RPM = 0; // Revolucoes por minuto
float windspeed_ms = 0; // Velocidade do vento (m/s)
float windspeed_kmh = 0; // Velocidade do vento (km/h)

// Sensor BMP280: Pressão barométrica
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // Inicializando objeto do tipo Adafruit_BMP280 (I2C)

// Sensor DS18B20: Temperatura da água
#include <OneWire.h>
#include <DallasTemperature.h>
#define ds18b20_data 2
OneWire oneWire(ds18b20_data);
DallasTemperature ds18b20_sensor(&oneWire);

// Classe Meliah Summers
class theBuoy { // Sera utilizado para printar no Serial, SD Card e LoRa
  public:
    template <typename T>
    void print(T x) {
      Serial.print(x);
      saveFile.print(x);
      LoRa.beginPacket(); LoRa.print(x); LoRa.endPacket();
      return 0;
    }
    template <typename T>
    void println(T x) {
      Serial.println(x);
      saveFile.println(x);
      LoRa.beginPacket(); LoRa.println(x); LoRa.endPacket();
      return 0;
    }
};
theBuoy Meliah;

// **************************************** VOID SETUP **************************************** //
void setup() {
  Serial.begin(9600);

  // SD Card
  pinMode(pinSS, OUTPUT); // Declara pinoSS como saída
 
  if (SD.begin()) { // Inicializa o SD Card
    Serial.println("SD Card READY."); // Imprime na tela
    Meliah.print("===== DATA REQUEST "); Meliah.print(loop_num); Meliah.println(" =====");
    loop_num++;
  }
  else {
    Serial.println("SD Card initialization FAIL.");
    return;
  }

  // LoRa SX1276
  while(!Serial);
  Serial.println("LoRa initialized!");
  if (!LoRa.begin(915E6)) { // Inicializa o LoRa na frequencia 915MHz
    Serial.println("LoRa initialization FAIL!");
  }

  // SHT20
  sht20.initSHT20(); // Inicializando o SHT20

  // PB10
  pinMode(REED, INPUT_PULLUP);

  // SV10
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);

  // DS18B20
  ds18b20_sensor.begin(); // Inicia a biblioteca DallasTemperature
  
}

// **************************************** VOID LOOP **************************************** //
void loop() {

  // ********** SD Card OPEN ********** //
  
  saveFile = SD.open("saveFile.txt", FILE_WRITE); // Cria e/ou abre arquivo .txt
  if (saveFile) { // Se o arquivo abrir imprime:
    Serial.println("Data will be saved at \'.txt\' file."); // Imprime na tela
  }
  else {     // Se o arquivo não abrir
    Serial.println("Save file opening ERROR!"); // Imprime na tela
  }

  // ********** SHT20 ********** //

  // SHT20: Temperatura do ar
  float air_temp = sht20.readTemperature();
  Meliah.print("AIR TEMPERATURE: "); Meliah.print(air_temp); Meliah.println("C");

  // SHT20: Umidade relativa do ar
  float humidity = sht20.readHumidity();
  Meliah.print("HUMIDITY: "); Meliah.print(humidity); Meliah.println("%");
  delay(1000);

  // ********** PB10 ********** //

  pb10_val = digitalRead(REED); // Le o status do reed switch
  if ((pb10_val == LOW) && (pb10_old_val == HIGH)) { // Verifica se o status mudou
    delay(10);
    REEDCOUNT += 1; // Adiciona 1 a contagem de pulsos
    pb10_old_val = pb10_val; // igual o valor antigo com o atual

    //Serial.print("Pluviometric measure (counter): "); Serial.print(REEDCOUNT); Serial.println(" pulse(s)");
    Meliah.print("PLUVIOMETRIC HEIGHT: "); Meliah.print(REEDCOUNT * 0.25); Meliah.println(" mm");
  }
  else {
    pb10_old_val = pb10_val; // Nao realizar nada, caso o status nao mude
  }

  // ********** SV10 ********** //

  sv10_sample++;
  windvelocity();

  // SV10: RPM
  RPMcalc();
  Meliah.print("SV10 ROTATIONS PER MINUTE: "); Meliah.print(sv10_RPM); Meliah.println(" RPM"); // Calcular e imprimir RPM do anemometro
  
  // SV10: Imprimir m/s
  Meliah.print("WIND SPEED: ");
  WindSpeed_ms(); // Calcular vel. do vento em m/s
  Meliah.print(windspeed_ms); Meliah.print(" m/s; ");
  
  // SV10: Imprimir km/s
  WindSpeed_kmh(); // Calcular vel. do vento em km/h
  Meliah.print(windspeed_kmh); Meliah.println(" km/h");
  delay(2000);

  // ********** BMP280 ********** //

  //Serial.print(F("Temperature: ")); Serial.print(bmp.readTemperature()); Serial.println(" *C");
  //Serial.print(F("Aprox. altitude: ")); Serial.print(bmp.readAltitude(1013.25),0); Serial.println(" m");
  float barometric_pressure = bmp.readPressure(); // Ler valor de pressao barometrica
  Meliah.print("PRESSURE: "); Meliah.print(barometric_pressure); Meliah.println(" Pa");
  delay(2000);

  // ********** DS18B20 ********** //

  // DS18B20: Temperatura da agua
  ds18b20_sensor.requestTemperatures();
  float water_temp = ds18b20_sensor.getTempCByIndex(0);
  Meliah.print("WATER TEMPERATURE: "); Meliah.print(water_temp); Meliah.println(" C");
  
  // ********** SD Card CLOSE ********** //

  saveFile.close(); // Fecha o arquivo após escrever

  // DATA REQUEST DELAY //
  delay(DATA_REQUEST_DELAY);
}

// **************************************** FUNCTIONS **************************************** //

// ***** SV10 ***** //
void windvelocity(){
  windspeed_ms = 0;
  windspeed_kmh = 0;
  
  sv10_counter = 0;  
  attachInterrupt(0, addcount, RISING);
  unsigned long millis();       
  long startTime = millis();
  while(millis() < startTime + sv10_period) {
  }
}
void RPMcalc(){
  sv10_RPM=((sv10_counter)*60)/(sv10_period/1000);  // Calculo de RPM
}
void WindSpeed_ms(){
  windspeed_ms = ((4 * pi * sv10_radius * sv10_RPM)/60) / 1000;  // Calculo da velocidade do vento em m/s
 
}
void WindSpeed_kmh(){
  windspeed_kmh = (((4 * pi * sv10_radius * sv10_RPM)/60) / 1000)*3.6;  // Calculo da velocidade do vento em km/h
 
}
void addcount(){
  sv10_counter++;
}
