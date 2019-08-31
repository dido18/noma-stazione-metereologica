#include <Wire.h>            // ethernet board
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DHT.h"
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include "Arduino.h"
#include "SI114X.h"        //sensore luminosità

#define DHTPIN 8          // pin sensore umidità
#define DHTTYPE DHT22     // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE); //inizializza sensore umidità temperatura
#define SEALEVELPRESSURE_HPA (1013.25) // pressione livello del mare

Adafruit_BME280 bme;


SI114X SI1145 = SI114X();
char msgBuffer[20];

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 11, 165);
IPAddress server(192, 168, 11, 7);
EthernetClient ethClient;
PubSubClient client(ethClient);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("umidita",dtostrf(bme.readHumidity(), 6, 2, msgBuffer));
      // ... and resubscribe
      client.subscribe("prova1");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


unsigned long previousMillis = 0; //
unsigned long interval = 5000; //intervallo invio dati
int pin = 7; // anemometro
double duration = 0; // anemometro
double periodo = 0; // anemometro
double velocita = 0; // anemometro
double raggio = 0.32; // anemometroc
char temperatura = 0;
double periodomedia = 0;
double sommavelocita = 0;
int contatoremedia = 0;
double mediavelocita = 0;


void setup()
{
  Serial.begin(9600);

  client.setServer(server, 1883);
  client.setCallback(callback);

  Ethernet.begin(mac, ip);
  // Allow the hardware to sort itself out
  //delay(1500);

  pinMode (pin, INPUT_PULLUP);// anemometro
  // initialize the LCD
  //	lcd.begin();
  // Turn on the blacklight and print a message.
  //lcd.backlight();
  //lcd.print("Hello, world!");

  //sensore umidità
  dht.begin();

  //sensore pressione
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  //sensore luce
  Serial.println("Beginning Si1145!");
  while (!SI1145.Begin()) {
    Serial.println("Si1145 is not ready!");
    //delay(1000);
  }
  Serial.println("Si1145 is ready!");
}

void loop()
{ // lettura sensore BME
  // lcd.setCursor(0,0);
  //lcd.print("Temperatura =");
  //lcd.print(bme.readTemperature());

  // lcd.setCursor(0,1);
  //lcd.print("Pressure =");
  //lcd.print(bme.readPressure() / 100.0F);
  //lcd.println("hPa");

  //lcd.setCursor(0,2);
  //lcd.print("Ap Altitude = ");
  // lcd.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  //lcd.println("m");

  // lcd.setCursor(0,3);
  //lcd.print("Humidity = ");
  //lcd.print(bme.readHumidity());
  //lcd.println("%");


  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  //Serial.println(sensorValue);

  //Serial.println();
  // delay(1000);

  //Serial.println("sensorValue");
  //anemometro


  duration = pulseIn(pin, HIGH, 5000000); //latenza 2 sec

  periodo = duration / 1000000 + duration / 10000000; //aggiunta di tempo passaggio calamita circa 30 gradi

  //Serial.print("periodo");
  //Serial.println(periodo);

  if (periodo > 0.01)

  {
    velocita = 3.6 * 2 * PI * raggio / periodo;
    periodomedia = periodomedia + periodo;
    sommavelocita = sommavelocita + velocita;
    contatoremedia = contatoremedia + 1;

    //lcd.println(velocita);
    Serial.print("velocita");

    Serial.println(velocita);

  }
  else

  {
    contatoremedia = 0;
    velocita = 0;
    sommavelocita = 0;
    mediavelocita = 0;

  }
  //Sensore luminosità

  float Vis = SI1145.ReadVisible();
  float IR = SI1145.ReadIR();
  float UV = SI1145.ReadUV() / 100;
  //Serial.println(Vis);

  //sensore BME pressione

  float pressione = bme.readPressure() / 100.0F;
  //Serial.println(pressione);

  //sensore umidità

  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    // Serial.println(F("Failed to read from DHT sensor!"));
    return;

  }


  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  //delay(2000);
  // Serial.print(F("Humidity: "));
  // Serial.println(h);
  // Serial.print(bme.readHumidity());
  temperatura = bme.readHumidity();
  // Serial.print(F("Temperature: "));
  //Serial.print(t);
  //Serial.println(F("°C "));
  //Serial.print(f);
  //Serial.print(F("Heat index: "));
  //Serial.print(hic);
  //Serial.println(F("°C "));
  //Serial.print(hif);
  //Serial.println(F("°F"));


  //invio dati ethernet
  if (!client.connected()) {
    reconnect();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {

    if (velocita == 0 & contatoremedia == 0 )
    {
      //Serial.println("ciao");

      client.publish("umidita", dtostrf(dht.readHumidity(), 6, 2, msgBuffer)); //trasforma in stringa e invia. Stringa: 6 cifr, 2 virgole
      client.publish("temperatura", dtostrf(dht.readTemperature(), 6, 2, msgBuffer));

      client.publish("anemometro", dtostrf(mediavelocita, 6, 2, msgBuffer));
      client.publish("pressione", dtostrf(bme.readPressure(), 6, 2, msgBuffer)); //pressione

      client.publish("visibile", dtostrf(SI1145.ReadVisible(), 6, 2, msgBuffer)); // sensore luminosità
      client.publish("IR", dtostrf(SI1145.ReadIR(), 6, 2, msgBuffer));
      client.publish("UV", dtostrf(SI1145.ReadUV() / 100, 6, 2, msgBuffer));
      client.loop();
    }
    else
    {

      //if the LED is off turn it on and vice-versa:
      mediavelocita = sommavelocita / contatoremedia;
      Serial.print("mediavelocita");
      Serial.println(mediavelocita);
      sommavelocita = 0;
      contatoremedia = 0;

      client.publish("umidita", dtostrf(dht.readHumidity(), 6, 2, msgBuffer)); //trasforma in stringa e invia. Stringa: 6 cifr, 2 virgole
      client.publish("temperatura", dtostrf(dht.readTemperature(), 6, 2, msgBuffer));

      client.publish("anemometro", dtostrf(mediavelocita, 6, 2, msgBuffer));
      client.publish("pressione", dtostrf(bme.readPressure(), 6, 2, msgBuffer)); //pressione

      client.publish("visibile", dtostrf(SI1145.ReadVisible(), 6, 2, msgBuffer)); // sensore luminosità
      client.publish("IR", dtostrf(SI1145.ReadIR(), 6, 2, msgBuffer));
      client.publish("UV", dtostrf(SI1145.ReadUV() / 100, 6, 2, msgBuffer));
      client.loop();

    }
    previousMillis = currentMillis; //save the last time you blinked the LED
  }


  //   client.publish("temperatura",dtostrf(dht.readTemperature(), 6, 2, msgBuffer));
  //
  //   client.loop();

  //delay(1000);
}


