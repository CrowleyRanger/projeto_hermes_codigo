#include <Wire.h>

// Sensor SHT2: Temperatura do ar e umidade relativa
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

// ******************** VOID SETUP ******************** //
void setup() {
  Serial.begin(9600);

  // SHT20
  Serial.println("Exemplo SHT20");
  sht20.initSHT20();

  // PB10
  pinMode(REED, INPUT_PULLUP);

  // SV10
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);

  // DS18B20
  ds18b20_sensor.begin(); // Inicia a biblioteca DallasTemperature
  
}
// ******************** // ******************** //

// ******************** VOID LOOP ******************** //
void loop() {

  // ***** SHT20 ***** //
  float umd = sht20.readHumidity();
  float temp = sht20.readTemperature();
  Serial.print("TEMPERATURE: ");
  Serial.print(temp, 1); // Mostra 1 numero apos o ponto
  Serial.print("C");
  Serial.print("HUMIDITY: ");
  Serial.print(umd, 1); // Mostra 1 numero apos o ponto
  Serial.print("%");
  Serial.println();
  delay(1000);

  // ***** PB10 ***** //
  pb10_val = digitalRead(REED); // Le o status do reed switch
  if ((pb10_val == LOW) && (pb10_old_val == HIGH)) { // Verifica se o status mudou
    delay(10);
    REEDCOUNT += 1; // Adiciona 1 a contagem de pulsos
    pb10_old_val = pb10_val; // igual o valor antigo com o atual

    // Imprime o resultado no monitor serial
    Serial.print("Pluviometric measure (counter): ");
    Serial.print(REEDCOUNT);
    Serial.println(" pulse(s)");
    Serial.print("Pluviometric measure (calculating): ");
    Serial.print(REEDCOUNT * 0.25);
    Serial.println(" mm");
  }
  else {
    pb10_old_val = pb10_val; // Nao realizar nada, caso o status nao mude
  }

  // ***** SV10 ***** //
  sv10_sample++;
  Serial.print("Measuring wind speed...");
  windvelocity();
  Serial.println(" Finished.");
  Serial.print("SV10 ROTATIONS PER MINUTE: ");
  Serial.print(sv10_counter);
  Serial.println(" RPM");
  RPMcalc();
  Serial.print(sv10_RPM);
  Serial.print("WIND SPEED: ");
  // SV10: Imprimir m/s
  WindSpeed_ms();
  Serial.print(windspeed_ms);
  Serial.println(" m/s; "); 
  // SV10: Imprimir km/s
  WindSpeed_kmh();
  Serial.print(windspeed_kmh);
  Serial.println(" km/h");  
  delay(2000);

  // ***** BMP280 ***** //
    //Serial.print(F("Temperature: "));
    //Serial.print(bmp.readTemperature()); // A temperatura está sendo medida pelo SHT22
    //Serial.println(" *C");
    
    Serial.print("PRESSURE: ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
 
    //Serial.print(F("Aprox. altitude: "));
    //Serial.print(bmp.readAltitude(1013.25),0);
    //Serial.println(" m");
    delay(2000);

    // ***** DS18B20 ***** //
    Serial.print("WATER TEMPERATURE: ");
    ds18b20_sensor.requestTemperatures();
    Serial.println(ds18b20_sensor.getTempCByIndex(0));

}
// ******************** // ******************** //

// ******************** FUNCTIONS ******************** //

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
// ******************** // ******************** //
