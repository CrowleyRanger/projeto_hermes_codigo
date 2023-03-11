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

// Sensor DHT22: Temperatura do ar e umidade relativa
#include "DHT.h" // Importando a biblioteca ./lib/DHT_sensor_library-1.4.4
#define DHTPIN 8 // Definindo para o pino 8
#define DHTTYPE DHT22 // Definindo o tipo do sensor DHT
DHT dht(DHTPIN, DHTTYPE); // Inicializando objeto do tipo DHT

// Sensor PB10: Pluviometro
const int REED = 6;
int pb10_val = 0;
int pb10_old_val = 0;
int REEDCOUNT = 0;

// Sensor SV10: Velocidade do vento
#define Hall sensor 3 // Pino digital 3
const float pi = 3.14159265; // Numero pi
int sv10_period = 5000; // Tempo de medida (ms)
int sv10_radius = 147; // Raio do anemometro (mm)--
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

// Sensor MPU6050: Acelerometro
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 mpu;
double altitude, amplitude_minima, amplitude_maxima, amplitude_onda, mean_point, smudge_factor;
byte escaped_smudge, mpu_started;
unsigned long mpu_startTime, mpu_endTime, frequencia;
float mean_period = -1;

// Classe Merliah Summers
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
theBuoy Merliah;

// **************************************** VOID SETUP **************************************** //
void setup() {
  Serial.begin(9600);

  // SD Card
  pinMode(pinSS, OUTPUT); // Declara pinoSS como saída
 
  if (SD.begin()) { // Inicializa o SD Card
    Serial.println("SD Card initialized."); // Imprime na tela
    Merliah.print("===== DATA REQUEST " + (String)loop_num + " =====");
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

  // DHT22
  dht.begin(); // Inicializando o DHT22

  // PB10
  pinMode(REED, INPUT_PULLUP);

  // SV10
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);

  // DS18B20
  ds18b20_sensor.begin(); // Inicia a biblioteca DallasTemperature

  // MPU6050
  Wire.begin();
  Serial.println("Inicializando dispositivos I2C...");
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);
  mpu.initialize();
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

  // ********** DHT22 ********** //

  // DHT22: Temperatura do ar
  float air_temp = dht.readTemperature();
  Merliah.print("AIR TEMPERATURE: " + (String)air_temp + "C");

  // DHT22: Umidade relativa do ar
  float humidity = dht.readHumidity();
  Merliah.print("HUMIDITY: " + (String)humidity + "%");
  delay(1000);

  // ********** PB10 ********** //

  pb10_val = digitalRead(REED); // Le o status do reed switch
  if ((pb10_val == LOW) && (pb10_old_val == HIGH)) { // Verifica se o status mudou
    delay(10);
    REEDCOUNT += 1; // Adiciona 1 a contagem de pulsos
    pb10_old_val = pb10_val; // igual o valor antigo com o atual

    //Serial.print("Pluviometric measure (counter): "); Serial.print(REEDCOUNT); Serial.println(" pulse(s)");
    Merliah.print("PLUVIOMETRIC HEIGHT: " + (String)(REEDCOUNT * 0.25) + " mm");
  }
  else {
    pb10_old_val = pb10_val; // Nao realizar nada, caso o status nao mude
  }

  // ********** SV10 ********** //

  windvelocity();

  // SV10: RPM
  RPMcalc();
  Merliah.print("SV10 ROTATIONS PER MINUTE: " + (String)sv10_RPM + " RPM"); // Calcular e imprimir RPM do anemometro
  
  // SV10: Imprimir m/s
  WindSpeed_ms(); // Calcular vel. do vento em m/s
  Merliah.print("WIND SPEED: " + (String)windspeed_ms + " m/s; ");
  
  // SV10: Imprimir km/s
  WindSpeed_kmh(); // Calcular vel. do vento em km/h
  Merliah.println((String)windspeed_kmh + " km/h");
  delay(2000);

  // ********** BMP280 ********** //

  //Serial.print(F("Temperature: ")); Serial.print(bmp.readTemperature()); Serial.println(" *C");
  //Serial.print(F("Aprox. altitude: ")); Serial.print(bmp.readAltitude(1013.25),0); Serial.println(" m");
  float barometric_pressure = bmp.readPressure(); // Ler valor de pressao barometrica
  Merliah.println("PRESSURE: " + (String)barometric_pressure + " Pa");
  delay(2000);

  // ********** DS18B20 ********** //

  // DS18B20: Temperatura da agua
  ds18b20_sensor.requestTemperatures();
  float water_temp = ds18b20_sensor.getTempCByIndex(0);
  Merliah.println("WATER TEMPERATURE: " + (String)water_temp + " C");

  // ********** MPU6050 ********** //

  unsigned long tempo_inicio = millis(); // Adquire o tempo em milisegundos o tempo de início
  barometric_pressure = bmp.readPressure(); // Le a pressao barometrica
  altitude = bmp.readAltitude(barometric_pressure); // Le a altitude
  amplitude_maxima = altitude;
  amplitude_minima = altitude;

  while(millis() - tempo_inicio < 15000){ // Por 15 segundos 
    barometric_pressure = bmp.readPressure();
    altitude = bmp.readAltitude(barometric_pressure);
    if (altitude < amplitude_minima) {
      amplitude_minima = altitude;
    } 
    if (altitude > amplitude_maxima) {
      amplitude_maxima = altitude;
    }
  }
  amplitude_onda = (amplitude_maxima - amplitude_minima)/2.0; // Calcula e registra a altitude da onda segunda dados adquiridos em 15 segundos
  mean_point = (amplitude_maxima + amplitude_minima)/2.0; // Calcula e registra a altitude do ponto medio entre amplitudes max e min
  smudge_factor = (amplitude_maxima - mean_point)*0.15; // Calcula e registra o fator smudge da onda
  
  tempo_inicio = millis(); // Registra tempo de operacao atual (ms)
  
  while(millis() - tempo_inicio < 15000) { // Por 15 segundos 
    barometric_pressure = bmp.readPressure();
    altitude = bmp.readAltitude(barometric_pressure);
    // Se dentro de uma margem de 30% da amplitude de onda a partier do ponto medio
    // Inicia o temporizador de outra forma o para
    if (altitude < mean_point + smudge_factor && altitude > mean_point - smudge_factor) { // True se a altitude estiver dentro do fator smudge
      if (!mpu_started){
        mpu_startTime = millis();
        mpu_started = true;
      }
      else {
        if (escaped_smudge) { // Caso a altitude tenha escapado a delimitacao max e/ou min do fator smudge
          mpu_endTime = millis(); // mpu_endTime eh atribuido quando escapa o fator smudge
          mpu_started = false;
          escaped_smudge = false;
          if (mean_period != -1) { // Executado apos a primeira vez
            mean_period = (mean_period + (mpu_endTime - mpu_startTime)*2)/2.0; // Periodo medio
          }
          else { // Executado apenas a primeira vez
            mean_period = (mpu_endTime - mpu_startTime)*2; // Periodo medio inicial: 
          }
        }
      }
    }
    else {
      escaped_smudge = true; // A altitude ultrapassou as delimitacoes max e/ou min do fator smudge
    } 
  }
  frequencia = 1/(mean_period/1000);
  
  Merliah.println("FREQUENCIA DA ONDA: " + (String)frequencia + " Hz");
  Merliah.println("AMPLITUDE DA ONDA: " + (String)amplitude_onda + " m");
  
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
  long sv10_startTime = millis();
  while(millis() < sv10_startTime + sv10_period) { // Fica dentro do loop ate o contador millis alcancar sv10_period de 5s
  }
}

void RPMcalc(){
  sv10_RPM=((sv10_counter)*60)/(sv10_period/1000);  // Calculo de RPM
}
void WindSpeed_ms(){
  windspeed_ms = ((2*pi*sv10_radius*sv10_RPM)/60)/1000;  // Calculo da velocidade do vento em m/s
 
}
void WindSpeed_kmh(){
  windspeed_kmh = (((2*pi*sv10_radius*sv10_RPM)/60)/1000)*3.6;  // Calculo da velocidade do vento em km/h
 
}
void addcount(){
  sv10_counter++;
}
