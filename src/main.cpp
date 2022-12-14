/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 * This sketch provides an example of how to implement a humidity/temperature sensor using a DHT11/DHT21/DHT22. 
 * It inlcudes Heat Index *sensor*
 * 
 */
#include <Arduino.h>

#define SN "TempHumRessenti"
#define SV "2.2"

// Enable debug prints to serial monitor
//#define MY_DEBUG

// Enable and select radio type attached
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95
#define MY_RADIO_RF24

#define MY_REPEATER_FEATURE

//Options: RF24_PA_MIN, RF24_PA_LOW, (RF24_PA_HIGH), RF24_PA_MAX
#define MY_RF24_PA_LEVEL RF24_PA_MAX

// Rien pour automatique
//#define MY_NODE_ID AUTO
#define MY_NODE_ID 1

//MY_RF24_CHANNEL par defaut 76
#define MY_RF24_CHANNEL 81 //test
//#define MY_RF24_CHANNEL 83 //Production

// Uncomment the type of sensor in use:
//#define DHTTYPE DHT11 // DHT 11 
//#define DHTTYPE DHT12
//#define DHTTYPE DHT21 // DHT 21 (AM2301)
#define DHTTYPE DHT22 // DHT 22 (AM2302)

// Set this to the pin you connected the DHT's data and power pins to; connect wires in coherent pins
#define DHTDATAPIN        3         
//#define DHTPOWERPIN       8

// Sleep time between sensor updates (in milliseconds) to add to sensor delay (read from sensor data; typically: 1s)
static const uint64_t UPDATE_INTERVAL = 60000; 

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM 1
#define CHILD_ID_RESSENT 2

// Set this offset if the sensors have permanent small offsets to the real temperatures/humidity.
// In Celsius degrees or moisture percent
#define SENSOR_TEMP_OFFSET 0      // used for temperature data and heat index computation
#define SENSOR_HUM_OFFSET 0       // used for humidity data
#define SENSOR_RESSENTI_OFFSET 0   // used for heat index data

// Wait times
#define SHORT_WAIT 50
#define LONG_WAIT 500
#define LONG_WAIT2 2000

// used libraries: they have to be installed by Arduino IDE (menu path: tools - manage libraries)
#include <MySensors.h>  // *MySensors* by The MySensors Team (tested on version 2.3.2)
//#include <Adafruit_Sensor.h> // Official "Adafruit Unified Sensor" by Adafruit (tested on version 1.1.1)
#include <DHT_U.h> // Official *DHT Sensor library* by Adafruit (tested on version 1.3.8) 

DHT_Unified dhtu(DHTDATAPIN, DHTTYPE);
// See guide for details on Adafruit sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

uint32_t delayMS;
float newTemp;
float newHum;
uint8_t nNoUpdates = FORCE_UPDATE_N_READS; // send data on start-up 
bool metric = true;
float temperature;
float humidity;
float T_ressentie;

float lastTemp;
float lastHum;
float lastT_ressentie;

bool first_message_sent = false;

MyMessage msgTEMP(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHUM(CHILD_ID_HUM, V_HUM);
MyMessage msgRESSENTI(CHILD_ID_RESSENT, V_TEMP);

float calculRessenti(float temperature, float percentHumidity) {
  // Based on Adafruit DHT official library (https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp)
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml

  float hi;

  temperature = temperature + SENSOR_TEMP_OFFSET; //include TEMP_OFFSET in HeatIndex computation too
  temperature = 1.8*temperature+32; //convertion to *F

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  hi = (hi-32)/1.8;
  return hi; //return Heat Index, in *C

}

/*void before()
{
  // Optional method - for initialisations that needs to take place before MySensors transport has been setup (eg: SPI devices).

}*/

void presentation()  
{ 
  Serial.print("===> Envoyer présentation pour noeud : "); Serial.println(MY_NODE_ID);

  char sNoeud[] = STR(MY_NODE_ID);

  // Send the sketch version information to the gateway
  Serial.println("Envoyer SketchInfo");
  Serial.print(SN); Serial.print(" "); Serial.println(SV);
  sendSketchInfo(SN, SV);
  wait(LONG_WAIT2);

  // Register all sensors to gw (they will be created as child devices)
  Serial.print("Envoyer présentation pour du noeud : ");
  Serial.println("Présenter les capteurs");
  // Temperature
  char sChild0[25];
  strcpy(sChild0, "myS ");
  strcat(sChild0, sNoeud);
  strcat(sChild0, " Temperature");
  Serial.println(sChild0);
  present(CHILD_ID_TEMP, S_TEMP, sChild0);
  wait(LONG_WAIT2); //to check: is it needed

  // Humidite
  char sChild1[25];
  strcpy(sChild1, "myS ");
  strcat(sChild1, sNoeud);
  strcat(sChild1, " Humidite");
  Serial.println(sChild1);
  present(CHILD_ID_HUM, S_HUM, sChild1);
  wait(LONG_WAIT2); //to check: is it needed

  // Ressenti
  char sChild2[25];
  strcpy(sChild2, "myS ");
  strcat(sChild2, sNoeud);
  strcat(sChild2, " T Ressentie");
  Serial.println(sChild2);
  present(CHILD_ID_RESSENT, S_TEMP, sChild2);
  wait(LONG_WAIT2); //to check: is it needed

  metric = getControllerConfig().isMetric;

}

void setup()
{
  ////pinMode(DHTPOWERPIN, OUTPUT);
  ////digitalWrite(DHTPOWERPIN, HIGH);
  //Serial.begin(9600); 
  // Initialize device.
  dhtu.begin();
  
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dhtu.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.print  ("Min Delay:   "); Serial.print(sensor.min_delay/1000); Serial.println(" ms");  
  Serial.println("------------------------------------");
  
  // Print humidity sensor details.
  dhtu.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.print  ("Min Delay:   "); Serial.print(sensor.min_delay/1000); Serial.println(" ms");  
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000; 

}

void loop()      
{  

  //Pour Home assistant
  if (!first_message_sent) {
    //send(msgPrefix.set("custom_lux"));  // Set custom unit.
    Serial.println("Sending initial value");
    send(msgTEMP.set(temperature + SENSOR_TEMP_OFFSET, 2));
    wait(LONG_WAIT);                                      //to check: is it needed
    send(msgHUM.set(humidity + SENSOR_HUM_OFFSET, 2));
    wait(LONG_WAIT);                                      //to check: is it needed
    send(msgRESSENTI.set(T_ressentie + SENSOR_RESSENTI_OFFSET, 2));
    wait(LONG_WAIT);                                      //to check: is it needed
    first_message_sent = true;
  }

  ////digitalWrite(DHTPOWERPIN, HIGH);   
  delay(delayMS); //delai entre chaque mesure

  sensors_event_t event;  
  // Get temperature event and use its value.
  dhtu.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    temperature = event.temperature;
    #ifdef MY_DEBUG
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" *C");
    #endif
  }

  // Get humidity event and use its value.
  dhtu.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    humidity = event.relative_humidity;
    #ifdef MY_DEBUG
      Serial.print("Humidite: ");
      Serial.print(humidity);
      Serial.println("%");
    #endif
  }

if (fabs(humidity - newHum)>=0.05 || fabs(temperature - newTemp)>=0.05 || nNoUpdates >= FORCE_UPDATE_N_READS) {
    newTemp = temperature;
    newHum = humidity;
    T_ressentie = calculRessenti(temperature,humidity); //computes Heat Index, in *C
    nNoUpdates = 0; // Reset no updates counter
    #ifdef MY_DEBUG
      Serial.print("T Ressentie: ");
      Serial.print(T_ressentie);
      Serial.println(" *C");    
    #endif    
    
    if (!metric) {
      temperature = 1.8*temperature+32; //convertion to *F
      T_ressentie = 1.8*T_ressentie+32; //convertion to *F
    }
    
    #ifdef MY_DEBUG
      wait(SHORT_WAIT);
      Serial.print("Sending temperature: ");
      Serial.print(temperature);
    #endif    
    if (lastTemp != temperature) {
      lastTemp = temperature;
      send(msgTEMP.set(temperature + SENSOR_TEMP_OFFSET, 2));
    }

    #ifdef MY_DEBUG
      wait(SHORT_WAIT);
      Serial.print("Sending humidity: ");
      Serial.print(humidity);
    #endif    
    if (lastHum != humidity) {
      lastHum = humidity;
      send(msgHUM.set(humidity + SENSOR_HUM_OFFSET, 2));
    }

    #ifdef MY_DEBUG
      wait(SHORT_WAIT);
      Serial.print("Sending T ressentie: ");
      Serial.print(T_ressentie);
    #endif    
    if (lastT_ressentie != T_ressentie) {
      lastT_ressentie = T_ressentie;
      send(msgRESSENTI.set(T_ressentie + SENSOR_RESSENTI_OFFSET, 2));
    }

  }

  nNoUpdates++;

  // Sleep for a while to save energy
  ////digitalWrite(DHTPOWERPIN, LOW); 
  wait(300); // waiting for potential presentation requests
  sleep(UPDATE_INTERVAL); 

}