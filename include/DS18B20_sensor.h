// #pragma once

// #if defined(ARDUINO) && ARDUINO >= 100
// #include "arduino.h"
// #else
// #include "WProgram.h"
// #endif

// // #include "Definitions.h"
// #include <OneWire.h>
// #include <DallasTemperature.h>

// class DS18B20sensor
// {
// protected:
// 	int id_;
//     OneWire sensorOneWireBus_;
//     int one_wire_bus_number_;
//     DeviceAddress sensorDeviceAddress_;
//     float temperature = 0.0;
// public:
// 	DS18B20sensor();
// 	void init( int pinNumber);
// 	void readTemperature();
// 	void getTemperature();
//     void getDeviceAddress();
// 	bool isOn();
// };
// //extern DS18B20sensor ds18b20sensor;
