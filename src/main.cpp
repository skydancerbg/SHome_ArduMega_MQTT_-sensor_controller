// https://forum.arduino.cc/index.php?topic=462937.0
// I have two installation with 20+ sensors each, on a Mega for memory size reason.
// Pay attention to the cable you use (a CAT6 id fine and a cable like those used for HI FI systems too)
// and to the topoly (avoid star network) and you'll get your result with no issue.
// what resistances did you use? i saw something about using 1.9k instead of 4.7k
// This was probably for use when things get critical with very long cables. The standard 4.7k is good for at least 5m.
// you should check the DHT22 if you want to have humidity and temperature.
// The temp is not as precise as the DS18B20,
// if you want very good temp sensor + humidity you should check for the DHT33
// only seen in a shop in Switzerland.

// https://www.instructables.com/id/Remote-Temperature-Monitoring-Using-MQTT-and-ESP82/
// Once this is all connected up you can use the Simple DallasTemperature example to get the temperature from the sensor which is sent to the serial output.
// I added sensors.setResolution(12) which sets the resolution of the device to 12 bits so that I get a more precise temperature reading. You can see from the values below what you can expect from each of the bit resolutions:
// Mode Resol Conversion time
// 9 bits 0.5°C 93.75 ms
// 10 bits 0.25°C 187.5 ms
// 11 bits 0.125°C 375 ms
// 12 bits 0.0625°C 750 ms

// #include <OneWire.h>

// #include <DallasTemperature.h>

// // Data wire is plugged into pin 2 on the Arduino
// #define ONE_WIRE_BUS 5

// // Setup a oneWire instance to communicate with any OneWire devices
// // (not just Maxim/Dallas temperature ICs)
// OneWire oneWire(ONE_WIRE_BUS);

// DallasTemperature sensors(&oneWire);

// .................................

// void setup()
// {
// ...........
//   pinMode(inPin, INPUT);
//   sensors.begin();
// }

// void loop()
// {
// ...................................
//   long now = millis();
//   if (now - lastMsg > 60000) {
//     lastMsg = now;
//     sensors.setResolution(12);
//     sensors.requestTemperatures(); // Send the command to get temperatures
//     temp = sensors.getTempCByIndex(0);
//     Sprintln(temp);
//     if((temp > -20) && (temp <60))
//       {
//       client.publish("ha/_temperature1", String(temp).c_str(),TRUE);
//       }
//   }
// }

// Connecting multiple sensors
// If the sensors are located relatively close to the Arduino/emonTx, then satisfactory operation 
// should be achieved by making a parallel connection of the sensors at the connection to the Arduino
//  or emonTx . This is called a radial or 'star' arrangement.
// If long cable runs are required, consideration should be given to connecting in 'daisy-chain' fashion
//  where one cable runs from the furthest sensor, connecting to each sensor in turn, before ending at the Arduino or emonTx.
// There is more about this in the note below.
// For short cable runs, unscreened two or three-core cable, or single-core (parasite mode) or twin-core (normal mode) 
// screened audio cable should be suitable. For longer cable runs, low capacitance cable such as RF aerial downlead 
//! (parasite mode) has been successfully used over a distance of 10 m. CAT 5 network cable has also been used with success over 
//! a distance of 30m, with data & ground using one twisted pair and power & ground using a second twisted pair.

 
// Cable Length
//! Up to 20m cable length has been successfully reported with a lower pull-up resistor value of 2K. 
// Adding multiple sensors will reduce the practical length, as will non consistent / nonlinear cable runs, see app note
//Spec says the ESP8266 can supply 12mA to the GPIO pins, 
//! and max draw on the DS18B20 is 5mA, 
//! so pull-up is needed with 2 or more, 
//but I don't know if that can change it from 9 to 12 bit...
// If you are experiencing a random temperature of 185°F, 85°C, or x.x, please follow these suggestions:

// Disconnect and reconnect the connections to the temperature sensor in question. Solder the connection wires if a faulty connection is suspected. Make sure there is plenty of metal to metal contact.
// Verify that the sensor is receiving +5V across the voltage source wire and the ground wire at the sensor connection. This will help insure that the sensor is not functioning in parasitic mode.
// The 1-Wire bus is "single-ended" and has no intrinsic noise protection. It is susceptible to interference if the cable is routed near power lines, fluorescent fixtures, motors or other noise sources. Keep the cable wiring short and avoid routing it near other electrical equipment.
// If you are still experiencing issues after the above steps, connect the sensor directly into the module itself, leaving out any long lengths of wire. Typical installations can support extending the temperature sensor up to 600 feet total combined cable length in a daisy-chain (linear) topology. It may vary depending on many factors such as type of cable used, electrical noise, etc. Connecting the sensor directly into the unit will eliminate any issues with extending the cable sensor length.
// If you are still experiencing temperature issues after these steps, please contact us for advanced technical support
// For more information on these sensors, please refer to the manufacturer documentation: http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

//! You have 100nF or similar decoupling capacitor between 5V/Gnd at the sensor end of the cable?
//! You have a strong pullup (2k2 perhaps rather than 4k7) between 5V and the data line at the Arduino end of the cable?
//! You run the data and ground on the _same_ twisted pair of the Cat5 cable?
//! All of these precautions should reduce chance of interference/noise corrupting the signal on the OneWire bus.

//! A DS18B20 returns 127 C when it is disconnected. ?????????????
// https://www.hacktronics.com/Tutorials/arduino-1-wire-tutorial.html
// {
//   float tempC = sensors.getTempC(deviceAddress);
//   if (tempC == -127.00) {
//     Sprint("Error getting temperature");
//   } else {

//! If the measurement sequence fails due to noise or insufficient power, 
//! the subsequent measurement data will be +85°C (185°F) which is the power-on reset value of the temperature register. 
//! Be aware that a defective sensor can corrupt the measurement sequence and spoil the measurement for all of the sensors on the bus.

//! Can you give it a try with 3K3 or even 2K2 PU resistors? (e.g. use 2x 4K7 in parallel)
//! That would pull up the (3.3V) voltage faster, giving a better square wave.

//! In case of temperature conversion problems (result is 85),
//! strong pull-up setup may be necessary. See section Powering the DS18B20
// in DS18B20 datasheet (page 7) and use DallasTemperature(OneWire*, uint8_t) constructor.
// https://github.com/milesburton/Arduino-Temperature-Control-Library
//? https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/ExternalPullup/ExternalPullup.ino

//? assync:
// https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/WaitForConversion/WaitForConversion.pde
// ? https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/WaitForConversion2/WaitForConversion2.pde
/////////////////////////////////////////

#include <Arduino.h>

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h> // https://pubsubclient.knolleary.net/api.html
//! I've changed the mah message lenght from 128 to 1024 in the Global library (PubSubClient.h)
//! Works on Arduino Mega and is enough for reporting over MQTT up to 28 DS18B20 sensors and 4 DHT22 sensors!!!
// #include <stdlib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
// #include <ArduinoJson.h>
#include "Private.h"
#include "Definitions.h"

// MAC address for the controller
byte mac[] = {
    // 0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x02 //izmislen MAC
};
//////////*************
//!PLACE YOUR CREDENTIALS IN Private.h as follows:
//////////*************
// MQTT connection parameters:
//const char *mqtt_server = "IP of the server";
//const char *mqttClientId = "your client ID";
//const char *mqtt_user = "mqtt broker user";
//const char *mqtt_pass = "mqtt broker pass";
//////////*************

//////////*************
const long mqtt_reconect_interval = 5000;    // 5 seconds ds18x20
unsigned long ds18x20_Read_delay = 800;  // 750 ms are required for the sensors to prepare the reading
//reading 5 ds18x20 ad one DGT_22 takes 2036 ms plus 800 ms for requesting readout for ds18x20
// unsigned long mqtt_msgs_interval_millis = 30000-800-2036;  //! DEFAULT REPORTING INTERVAL I SET TO ARROUND 30 SECONDS (recirculation pump control reqires short reporting intrval)
unsigned long mqtt_msgs_interval_millis = 60000;  //! DEFAULT REPORTING INTERVAL I SET TO ARROUND 30 SECONDS (recirculation pump control reqires short reporting intrval)
// unsigned long ds18x20_perform_reading_millis=0;
bool ds18x20_perform_reading = false;
bool loop_first_pass = true;
//TODO ???Create another topic to report only the hot water recirculation and sollar water heating related temperatures frequently???
//TODO ???DO WE NEED THE REST OF THE SENSORS TO REPORT SO OFTEN?? 
unsigned long lastMsg_millis = 0; // will store last time STATUS was send to MQTT
unsigned long elapsed_millis = 0; // will store last time STATUS was send to MQTT

// Ethernet DHCP maintenace
void dhcp_maintain ();

// MQTT maintenance (with reconect if nessesary)
void mqtt_maintain();

// MQTT reconect in loop - non blocking!!!
boolean mqtt_reconnect();

// call sensors.requestTemperatures() to issue a global temperature request
// Request all devices on all busses
void ds18x20_Request_All_Temperatures();

// Read the temperatures for all ds18x20 sensors and
// place the result in the stat_msg buffer
void ds18x20_Read_All_Temperatures(char *stat_msg, bool &ds18xb20_found);

// Read the temperatures for all DHT sensors and
// place the result in the stat_msg buffer
// Reads the sensors by the DHT sensor bus order up to num_DHT_attached 
//! num_DHT_attached - CHANGE THIS NUMBER WHEN ATTACHING DHT SENSORS TO THE CONTROLER!
void dht_Read_All_Temperatures(char *stat_msg, bool &ds18xb20_found);

// Handle incomming MQTT messages
void mqtt_callback(char *topic, byte *payload, unsigned int length);

// MQTT publish STAT message
void mqtt_publish_stat_message(char *topic, char *payload);

// char* string manipulation
int stringToInteger(const char *inputstr);

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);
unsigned long lastReconnectAttempt = 0;

// Execute reconect attempt
boolean reconnect();


//  MQTT topics:
// const char willTopic[] = "arduino/will";
const char *mqtt_LWT_topic = "tele/ardu_m_heating_temp_hum/LWT";
const char *mqtt_sensor_topic = "tele/ardu_m_heating_temp_hum/SENSOR";
const char *mqtt_power_detection_sensor_topic = "cmnd/ardu_m_heating_temp_hum/POWERDETECT";
const char *mqtt_set_reporting_iterval_topic = "cmnd/ardu_m_heating_temp_hum/INTERVAL";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////
// Arduino Mega pinout http://www.circuitstoday.com/wp-content/uploads/2018/02/Arduino-Mega-Pin-Configuration.jpg
//! DO NOT USE: Pins 0 and 1 Rx1,TX1 - used by serial
//! DO NOT USE: Pin 4 - SS pin for the SD card
//! DO NOT USE: Pin 10 - SS for the Ethernet module (10 is default)
//! DO NOT USE: Pin 13 - LED_BUILT_IN pin
//! DO NOT USE: 50, 51, 52, 53 - SPI pins for the Ethernet shield /on the raiser header or on he pin headers/
//! DO NOT USE: 54 do not use, needs to be set to Output HIGH
// Available pins:

// OneWire ds18x20[] = { 22,23,24,25,26,27,28,29,30,31 };
OneWire ds18x20[] = {22, 23, 24, 25, 26};
const int oneWireCount = sizeof(ds18x20) / sizeof(OneWire);//! ??? не трябва ли за всяка шиnа по отделно??????
DallasTemperature ds18x20_sensor_bus[oneWireCount];
int num_DS18X20_devices_per_bus[oneWireCount][1]; //???? NOT USED ????
// deviceCount = sensors.getDeviceCount();


// https://forum.arduino.cc/index.php?topic=547129.0
//
#define DHTTYPE DHT22 // DHT 22  (AM2302)

//! WARNING ATTACH DHT SENSORS TO THE PINS IN THIS ORDER:
//! MAX NUMBER OF DHT SENSORS IS 4 - MORE REQUIRES CHANGE BELOW!
#define DHTPIN1 2   
#define DHTPIN2 3
#define DHTPIN3 5
#define DHTPIN4 6
#define MAX_NUMBER_OF_DHT_SENSORS 4
const int num_DHT_attached = 2; //!   CHANGE THIS NUMBER WHEN ATTACHING DHT SENSORS TO THE CONTROLER!

DHT dht[] = {
    {DHTPIN1, DHTTYPE},
    {DHTPIN2, DHTTYPE},
    {DHTPIN3, DHTTYPE},
    {DHTPIN4, DHTTYPE},
};

float dht_humidity[num_DHT_attached];
float dht_temperature[num_DHT_attached];

// POWER DETECTION (220V) sensors
const boolean PowerDetected = true; // button connects to GND
const int Num_pwr_detect_sensors =4;
int pwr_detect_pins[] = {7,8,9,11};
bool pwr_detect_last_reading[Num_pwr_detect_sensors]={false,false,false,false};

void read_Power_Detection_Sensors();
void mqtt_publish_message(char *out_topic, char *payload);
void printAddress(DeviceAddress deviceAddress);

//********************************
// *********** SETUP *************
void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // INIT POWER DETECTION (220V) SENSOR PINS
  for (size_t i = 0; i < Num_pwr_detect_sensors; i++)
  {
    pinMode(i,INPUT);
  }
  

  // start serial port
  // Open serial communications and wait for port to open:
  Sbegin(115200);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////
  // Start the Ethernet connection:
  Sprintln(F("Initialize Ethernet with DHCP:"));
  if (Ethernet.begin(mac) == 0)
  {
    Sprintln(F("Failed to configure Ethernet using DHCP"));
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      Sprintln(F("Ethernet shield was not found.  Sorry, can't run without hardware. :("));
    }
    else if (Ethernet.linkStatus() == LinkOFF)
    {
      Sprintln(F("Ethernet cable is not connected."));
    }
    // no point in carrying on, so do nothing forevermore:
    while (true)
    {
      delay(1);
    }
  } // END Start the Ethernet connection

  // print local IP address:
  Sprintln(F("IP address: "));
  Sprintln(Ethernet.localIP());

  // Set MQTT server/port and callback funcion
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqtt_callback);
  mqtt_reconnect();

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////

  Sprintln("Dallas Temperature Multiple Bus Control");
  Sprint("============Ready with ");
  Sprint(oneWireCount);
  Sprintln(" Sensor busses ================");

  // Start up the Dallas library on all defined bus-wires
  DeviceAddress deviceAddress;
  for (int i = 0; i < oneWireCount; i++)
  {
    ds18x20_sensor_bus[i].setOneWire(&ds18x20[i]);
    ds18x20_sensor_bus[i].begin();
    // int deviceCount = ds18x20_sensor_bus[i].getDeviceCount();
    // Sprint("Bus");
    // Sprint(i+1);
    // Sprint(" Device Count: ");
    // Sprintln(deviceCount);
    // num_devices_per_bus[i][1]=deviceCount;

    // deviceCount = sensors.getDeviceCount();

    for (size_t j = 0; j < 10; j++)//!dhfgjhfhjfhjfhgdfgdfg
    {
      if (ds18x20_sensor_bus[i].getAddress(deviceAddress, j))
        ds18x20_sensor_bus[i].setResolution(deviceAddress, 12); /* code */
    }
    // Initial request temperature to the bus in order to warm up the sensors and avoid 85C reading ???????
    // не не необходимо !!! ds18x20_sensor_bus[i].requestTemperatures(); //! test!!!!!!!!!!??????????????????
  }
  //Initialize DHT22 sensors
  for (auto &dht22_sensor : dht)
  {
    dht22_sensor.begin();
  }
}


//********************************
// *********** LOOP *************
void loop(void)
{
  ////////////////////////////////////////////////////
  // DHCP maintenance:
  dhcp_maintain ();
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MQTT connection maintenance:
  mqtt_maintain();
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // READ POWER DETECTION (220V) SENSOR PINS
  read_Power_Detection_Sensors();

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  unsigned long now = millis();

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
if (loop_first_pass || (now - lastMsg_millis > mqtt_msgs_interval_millis))
{
 elapsed_millis= now;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
// call sensors.requestTemperatures() to issue a global temperature request
// Request all devices on all busses
  ds18x20_Request_All_Temperatures();
  // // call sensors.requestTemperatures() to issue a global temperature
  loop_first_pass=false;
  ds18x20_perform_reading = true;
  digitalWrite(LED_BUILTIN, HIGH);
}

  now = millis();

  //delay(1000);//! WTF ????? 1second (a bit more than 750ms required to prepare the ds18x20 sensor readings)
if (ds18x20_perform_reading && (now - lastMsg_millis > ds18x20_Read_delay))
{
  // 800 ms passed, so we let Ehernet and MQTT to do maintenace
  ////////////////////////////////////////////////////
  // DHCP maintenance:
  dhcp_maintain ();
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // MQTT connection maintenance:
  mqtt_maintain();
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // START THE SENSOR READING PROCEDURE:
//! I've changed the mah message lenght from 128 to 1024 in the Global library (PubSubClient.h)
//! Works on Arduino Mega and is enough for reporting over MQTT up to 28 DS18B20 sensors and 4 DHT22 sensors!!!
//? stat_msg[1024] corresponds to the max size of the MQTT PubSubClient buffer as set
  char stat_msg[1024] = {"{"};  // Open the mqtt STAT message curly brackets 
  bool ds18xb20_found = false;// Flag for not printing the leading comma in the DHT section, if there are no ds18xb20 sensors attached
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ds18x20_Read_All_Temperatures(stat_msg, ds18xb20_found);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //   //TODO Add humidity readings to the mqtt message!
  dht_Read_All_Temperatures(stat_msg, ds18xb20_found);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  strcat(stat_msg, "}"); // close the mqtt STAT message curly bracket
  Sprintln("This is the MQTT message payload:");
  Sprintln(stat_msg);

  //! ВНИМАНИЕ вдигнах MQTT_MAX_PACKET_SIZE от 128 на 1024 в глобалната библиотека !!!!!!!!!!!!!!!!!!!!!!!!!!
  //TODO Да я направя локална и там да увелича MQTT_MAX_PACKET_SIZE  , а не в глобалната!!!!!!!!!!!!!!!!!!!!!!

  mqttClient.publish((char *)mqtt_sensor_topic, stat_msg);
  lastMsg_millis = millis();  // Set current millis in order to schedule next STAT publish in mqtt_stat_msgs_interval
  ds18x20_perform_reading = false;
  digitalWrite(LED_BUILTIN, LOW);

  Sprint("Elapsed time reading the DHT sensors: ");
  Sprint (millis()-elapsed_millis);
  Sprintln(" ms !!! USE THIS VALUE TO FINE TUNE THE MQTT MESSAGE PERIOD IN mqtt_msgs_interval_millis !!!!");
}
  // delay(5000);//!5 sec delay in order to publish approximatelly every 7 seconds
  // mqttClient.loop();
  // delay(5000);//!5 sec delay in order to publish approximatelly every 7 seconds
  // mqttClient.loop();
  // delay(5000);//!5 sec delay in order to publish approximatelly every 7 seconds
  // mqttClient.loop();
  // delay(5000);//!5 sec delay in order to publish approximatelly every 7 seconds
  // mqttClient.loop();
  // delay(5000);//!5 sec delay in order to publish approximatelly every 7 seconds
  // mqttClient.loop();
  // delay(3000);//!5 sec delay in order to publish approximatelly every 7 seconds
  // mqttClient.loop();


  // tele/.../SENSOR payload examples:
  // {"Time":"2020-02-09T19:30:03","SI7021":{"Temperature":2.5,"Humidity":73.9},"TempUnit":"C"}
  // {"Time":"2020-02-10T16:43:14","DS18B20":{"Id":"0300A2790D8B","Temperature":24.2},"TempUnit":"C"}

} //END Loop
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PubSub MQTT reconnect
boolean mqtt_reconnect()
{
  // Attempt to connect
  if (mqttClient.connect(mqttClientId, mqtt_user, mqtt_pass, mqtt_LWT_topic, 1, true, "Offline"))
  {
    Sprintln("MQTT server  connected");
    // Once connected, publish an "Online" announcement...
    mqttClient.publish(mqtt_LWT_topic, "Online");
    Sprintln("Online message published on LWT topic");

    //Subscribe to MQTT topics:
    //
  }
  return mqttClient.connected();
} //END mqtt_reconnect()

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Publish STAT MQTT message
void mqtt_publish_message(char *out_topic, char *payload)
{
  mqttClient.publish(out_topic, payload);
  Sprint("Message published on ");
  Sprint(out_topic);
  Sprint(" topic with payload: ");
  Sprintln(payload);
}
//! Periodic stat example:
//! {"Time":"2020-02-09T19:25:44","Uptime":"3T06:15:04","UptimeSec":281704,"Heap":28,"SleepMode":"Dynamic","Sleep":50,"LoadAvg":19,"MqttCount":8,"POWER1":"OFF","POWER2":"OFF","POWER3":"OFF","POWER4":"OFF","Wifi":{"AP":1,"SSId":"code","BSSId":"F8:D1:11:3B:4B:16","Channel":9,"RSSI":100,"LinkCount":1,"Downtime":"0T00:00:37"}}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Handle incomming MQTT messages
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  Sprintln("Message arrived");
  Sprint("on topic - ");
  Sprintln(topic);
// "cmnd/ardu_m_heating_temp_hum/INTERVAL";
//! WE EXPECT PAYLOAD  - NUMBER OF MILLISECONDS HERE!
  if (strcmp(topic, "cmnd/ardu_m_heating_temp_hum/INTERVAL") == 1) {
    unsigned long incomming_millis = (unsigned long)payload;
    //! if it is less than 5 sec (5000 millis) discard- it is not a valid reporting period
    if (incomming_millis>=5000)
    {
      mqtt_msgs_interval_millis=incomming_millis;
    }
    
  }
} //Handle incomming MQTT messages

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// function to print a device address to serial
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Sprint("0");
    SprintWtype(deviceAddress[i], HEX);
  }
}
////////////////////////////////////////////////////

void dhcp_maintain (){
    switch (Ethernet.maintain())
  {
  case 1:
    //renewed fail
    Sprintln(F("Error: renewed fail"));
    break;

  case 2:
    //renewed success
    Sprintln(F("Renewed success"));
    //print your local IP address:
    Sprintln("My IP address: ");
    Sprintln(Ethernet.localIP());
    break;

  case 3:
    //rebind fail
    Sprintln(F("Error: rebind fail"));
    break;

  case 4:
    //rebind success
    Sprintln(F("Rebind success"));
    //print your local IP address:
    Sprint(F("My IP address: "));
    Sprintln(Ethernet.localIP());
    break;

  default:
    //nothing happened
    break;
  } // End DHCP maintenance
}
void mqtt_maintain(){
  // PubSub MQTT client non blocking reconect based on  https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
  // Actually is blocking while the connect attempt - https://github.com/knolleary/pubsubclient/issues/147
  //???? Solution??? https://github.com/knolleary/pubsubclient/issues/583 ???

  unsigned long now = millis();
    if (!mqttClient.connected())
  {
    Sprint("failed, rc=");
    Sprint(mqttClient.state());
    Sprintln(" try connecting MQTT server again in " );
    Sprint(mqtt_reconect_interval - (now - lastReconnectAttempt));
    Sprintln(" miliseconds " );

    if (now - lastReconnectAttempt > mqtt_reconect_interval)
    {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      Sprintln("");
      Sprintln("New MQTT connection attempt...");
      if (mqtt_reconnect())
      {
        lastReconnectAttempt = 0;
        Sprintln(" Connection to MQTT server successful! ");
        //! ??????????????????????????????????????????????
        lastMsg_millis = millis();  // Schedule next STAT publish in mqtt_stat_msgs_interval
      }
    }
  }
  else
  {
    // Client connected
    mqttClient.loop();
  } //END MQTT connection non blocking maintenance

}

// call sensors.requestTemperatures() to issue a global temperature request
// Request all devices on all busses 
void ds18x20_Request_All_Temperatures(){
  Sprintln();
  Sprint("Requesting temperatures...");
  for (int i = 0; i < oneWireCount; i++)
  {
    ds18x20_sensor_bus[i].requestTemperatures();
  }
  Sprintln("DONE");
}

void ds18x20_Read_All_Temperatures(char *stat_msg, bool &ds18xb20_found){

  DeviceAddress deviceAddress;
  bool include_comma=false;

  for (int i = 0; i < oneWireCount; i++)
  {
    int deviceCount = ds18x20_sensor_bus[i].getDeviceCount();
    Sprint("Bus");
    Sprint(i+1);
    Sprint(" Device Count: ");
    Sprintln(deviceCount);

    for (size_t j = 0; j < ds18x20_sensor_bus[i].getDeviceCount(); j++) //! WTF 3?????
    {
      if (ds18x20_sensor_bus[i].getAddress(deviceAddress, j))
      { 
        float temperature = ds18x20_sensor_bus[i].getTempCByIndex(j);

        if (temperature == -127.00)
        {
          Sprint("Error sensor disconnected");
        }
        // else if (temperature == 85.00)
        // {
        //   Sprint("Error sensor temperature conversion problems");
        // }
        else
        {
          ds18xb20_found = true; // enables printing the leading comma in DHT section of the message
          if (include_comma==false)
          {
            include_comma= true;
          }else{
          strcat(stat_msg, ",");
          }
          
          strcat(stat_msg, "\"DS18B20_");
          // Zero padded human readable HEX representation of the device address:
          for (uint8_t k = 0; k < 8; k++)
          {
            char hexbuffer[4];

            if (deviceAddress[k] < 16)
              strcat(stat_msg, "0"); // add leading zero if is only one character long....
                                     // https://arduinobasics.blogspot.com/2019/05/sprintf-function.html
            sprintf(hexbuffer, "%01X", deviceAddress[k]);
            // sprintf(hexbuffer, "%0#X", deviceAddress[k]); // in format 0xFF
            strcat(stat_msg, hexbuffer);
          }

          strcat(stat_msg, "\":");
          // strcat(stat_msg, "\":\"");
          // https://arduino.stackexchange.com/questions/16933/read-sensor-and-convert-reading-to-const-char
          char tempBuffer[20]; // make sure this is big enough to hold your string
          int IntegerPart = (int)(temperature);
          // https://forum.arduino.cc/index.php?topic=42403.0
          char str[20];
          sprintf(str,"%d",IntegerPart);
          int numberOfDigits = strlen(str)+3; //strlen(str) includes the minus sign if any, the 3 is dot plus 2 the for the decimal part 
          // Sprintln(numberOfDigits);
          strcat(stat_msg, dtostrf(temperature, numberOfDigits, 2, tempBuffer)); // da se izprobva s tricifrena temperatura
          Sprint("Temperature for sensor ");
          Sprint(j + 1); // make it 1 based
          Sprint(" device address: ");
          printAddress(deviceAddress);
          Sprint(" on bus ");
          Sprint(i + 1); // make it 1 based
          Sprint(" is ");
          Sprintln(temperature);
          // Sprintln();
        }
      }
    }
  }
}

  void dht_Read_All_Temperatures(char *stat_msg, bool &ds18xb20_found){

    for (int i = 0; i < num_DHT_attached; i++)
    {
      dht_temperature[i] = dht[i].readTemperature();
      dht_humidity[i] = dht[i].readHumidity();

      //!It takes about 300ms to read each DHT sensor, that's why we give ethernet and mqtt a chance between the readings
      ////////////////////////////////////////////////////
      // DHCP maintenance:
      dhcp_maintain ();
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // MQTT connection maintenance:
      mqtt_maintain();
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    for (int i = 0; i < MAX_NUMBER_OF_DHT_SENSORS; i++)
    {
      if (isnan(dht_temperature[i]) || isnan(dht_humidity[i]))
      {
        //TODO What to do if there is false reading????????????????????
        //? JUST SKIP IT? THIS WILL POP A MESSAGE IN OPENHAB LOG...
      }
      else
      {
        if (ds18xb20_found) strcat(stat_msg, ","); //Add comma if there are previous sensor readings in the stat_message

        strcat(stat_msg, "\"DHT_");
        char buf[4];   // "-32\0"
        int l = i + 1; // make the DHT naming 1 based (not zero based)
        itoa(l, buf, 10);
        strcat((char *)stat_msg, (char *)buf);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        strcat(stat_msg, "\":{\"Temperature\":");
        // strcat(stat_msg, "\":");
              // strcat(stat_msg, "\":\"");
        // https://arduino.stackexchange.com/questions/16933/read-sensor-and-convert-reading-to-const-char
        char temBuffer[20]; // make sure this is big enough to hold your string
        int IntegerPart = (int)(dht_temperature[i]);
        // https://forum.arduino.cc/index.php?topic=42403.0
        char str_t[20];
        sprintf(str_t,"%d",IntegerPart);
        int numberOfDigits = strlen(str_t)+3; //?? 2 ili 3 ????? strlen(str) includes the minus sign if any, the 3 is dot plus 2 the for the decimal part 
        // Sprintln(numberOfDigits);
        strcat(stat_msg, dtostrf(dht_temperature[i], numberOfDigits, 2, temBuffer)); 
        // strcat(stat_msg, "\", ");
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        strcat(stat_msg, ",\"Humidity\":");
        // strcat(stat_msg, "\":");
              // strcat(stat_msg, "\":\"");
        // https://arduino.stackexchange.com/questions/16933/read-sensor-and-convert-reading-to-const-char
        char humBuffer[20]; // make sure this is big enough to hold your string
        IntegerPart = (int)(dht_humidity[i]);
        // https://forum.arduino.cc/index.php?topic=42403.0
        char str_h[20];
        sprintf(str_h,"%d",IntegerPart);
        numberOfDigits = strlen(str_h)+3; //strlen(str) includes the minus sign if any, the 3 is dot plus 2 the for the decimal part 
        // Sprintln(numberOfDigits);
        strcat(stat_msg, dtostrf(dht_humidity[i], numberOfDigits, 2, humBuffer));
        strcat(stat_msg, "}"); 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Sprint(F("Temperature "));
        // Sprint(F("Temperature for sensor 3 on bus 2 is "));
        Sprint(F("DHT sensor "));
        Sprintln(i+1);
        Sprint(F("Temperature: "));
        Sprintln(dht_temperature[i]);
        Sprint(F("Humidity: "));
        Sprint(i+1);
        Sprint(" ");
        Sprintln(dht_humidity[i]);
      }
    }
  }

  void read_Power_Detection_Sensors(){
  bool publish_now=false;
  for (size_t i = 0; i < Num_pwr_detect_sensors; i++)
  {
    boolean sensor_reading = (digitalRead(i)==HIGH);
    if (sensor_reading != pwr_detect_last_reading[i])
    {
     pwr_detect_last_reading[i] = sensor_reading;
     publish_now=true;
    }
  }
    if (publish_now || loop_first_pass)
    {
      char pwr_msg[128] = {"{"};  // Open the mqtt STAT message curly brackets 
      
      for (size_t i = 0; i < Num_pwr_detect_sensors; i++){
        strcat(pwr_msg, "\"PWR_");
        char buf[4];   // "-32\0"
        int l = i + 1; // make the POWER sensor naming 1 based (not zero based)
        itoa(l, buf, 10);
        strcat((char *)pwr_msg, (char *)buf);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        strcat(pwr_msg, "\":{\"PWRdetected\":"); 
        char reading_buf[4];
        itoa(pwr_detect_last_reading[i]?1:0,reading_buf,10); 
        strcat((char *)pwr_msg, (char *)reading_buf);
        strcat(pwr_msg, "}");/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //strcat(pwr_msg, "\",");
        if (i != Num_pwr_detect_sensors-1)
        {
        strcat(pwr_msg, ",");
        }

      }
        strcat(pwr_msg, "}");
      mqtt_publish_message((char *)mqtt_power_detection_sensor_topic, (char *)pwr_msg); 
      //Sprintln(pwr_msg);

    }
    
  }
   

