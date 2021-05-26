/* * Door Module --- Home Security  * *
--05/26/2021
? Features: Detect Knock (interrupt)
?            Environment

  TODO: Detect Door open/closed
        Detect Noise
        Trigger RPi/Controller
        Add MQTT
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <time.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <NTPClient.h>

// #define BMP_SCK 5
// #define BMP_MISO 12
// #define BMP_MOSI 11 
// #define BMP_CS 10

// * WiFi + MQTT * //
// Replace the next variables with your SSID/Password combination
char* ssid = "+++SSID+++";
const char* password = "++PASSWORD+++";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.1.81";
//const char* mqtt_server = "localhost";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
long localadd;
char msg[50];
int value = 0;
//////////////////////////

// * Time Setup * //
//Setup Time object
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//Week Days
String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Month names
String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

#define timeSeconds 10


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

const int knockPin = 13;
const int ledonPin = 12;
const int ledoffPin = 14;


int knockVal = HIGH;
boolean knockAlarm = false;
unsigned long prevKnockTime;
unsigned long delayTime;

int knockAlarmTime = 100;

void bmpData(){
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();
}

// Checks if motion was detected, sets LED HIGH and starts a timer
ICACHE_RAM_ATTR void detectsKnock() {
  startTimer = true;
  lastTrigger = millis();
  Serial.print("[TRIGGER]Knock DETECTED at:");
  Serial.println(millis());
  //client.publish(motionTopic, oN);
  //motionState = oN;
  digitalWrite(ledoffPin,LOW);
  digitalWrite(ledonPin,HIGH);
  Serial.print("[TRIGGER]Turning guide lights: ON");
  //client.publish(glightTopic, oN);
}

void setup ()
{
  Serial.begin(115200);  
  pinMode (ledoffPin, OUTPUT) ;
  pinMode (ledonPin, OUTPUT) ;

  //pinMode (knockPin, INPUT) ;
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(knockPin, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(knockPin), detectsKnock, RISING);

  Serial.println(F("BMP280 Sensor loading...."));
  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

}
void loop ()
{

    // Current time
  now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    Serial.println("[FORCE]Motion stopped...");
    digitalWrite(ledonPin,LOW);
    digitalWrite(ledoffPin,HIGH);
    Serial.println("[FORCE]Turning guide lights off.");
    startTimer = false;
  }

  bmpData();
  delay(1000);
}
