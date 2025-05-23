// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

void setup() {
  Serial.begin(9600);
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void loop() {
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
}

/////////
#include <DHT.h>
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  Serial.print("Temp: "); Serial.print(temp); Serial.print(" °C, ");
  Serial.print("Humidity: "); Serial.println(humidity);
  delay(2000);
}
////////////////////////
#define TRIG 3
#define ECHO 4

void setup() {
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10); digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  int distance = duration * 0.034 / 2;
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
  delay(1000);
}
////////////////
#define MQ2 A0

void setup() {
  Serial.begin(9600);
}

void loop() {
  int gasLevel = analogRead(MQ2);
  Serial.print("Gas Sensor Value: "); Serial.println(gasLevel);
  delay(1000);
}
//////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void setup() {
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println("BMP180 not detected.");
  }
}

void loop() {
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    Serial.print("Pressure: "); Serial.print(event.pressure); Serial.println(" hPa");
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
  }
  delay(2000);
}
/////////////////////

#define SOUND_PIN A2

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sound = analogRead(SOUND_PIN);
  Serial.print("Sound Level: "); Serial.println(sound);
  delay(1000);
}
//////////////////////////////// explanation
Got it! Here's the detailed explanation of each sensor with pin connections and working principles for each. This should help you understand both the sensor functionality and how to connect them to your Arduino board.

✅ 1. DHT11 – Temperature & Humidity Sensor
Purpose:
The DHT11 sensor measures temperature and humidity in the air.

Pinout & Connections:
VCC → 5V

GND → GND

DATA → Digital Pin 2

Working Principle:
The DHT11 uses a thermistor to measure temperature and a capacitive sensor to measure humidity. It sends the data digitally to the Arduino via a single-wire interface.

The sensor reads temperature and humidity values and sends them to the Arduino. These values can be used to monitor environmental conditions.

✅ 2. HC-SR04 – Ultrasonic Distance Sensor
Purpose:
The HC-SR04 measures the distance between the sensor and an object using ultrasonic sound waves.

Pinout & Connections:
VCC → 5V

GND → GND

TRIG → Digital Pin 3 (Trigger pin, sends a pulse)

ECHO → Digital Pin 4 (Receives the echo pulse)

Working Principle:
The sensor sends out an ultrasonic pulse from the TRIG pin.

It then waits for the pulse to reflect back from an object and be received on the ECHO pin.

The duration of the echo pulse is used to calculate the distance using the formula:
Distance = (Duration * Speed of Sound) / 2.

This is useful for measuring distances in applications like robotics or object detection.

✅ 3. MQ-2 – Gas Sensor
Purpose:
The MQ-2 sensor is used to detect gases such as LPG, methane, smoke, and other flammable gases.

Pinout & Connections:
VCC → 5V

GND → GND

AOUT → Analog Pin A0 (for analog output)

DOUT → Digital Pin (optional, if you want a threshold-based digital output)

Working Principle:
The MQ-2 uses a heating element with a gas-sensitive layer that changes resistance when exposed to gases.

The sensor measures the resistance of the sensor material in response to gases in the air. The analog signal from the AOUT pin is proportional to the concentration of the gas detected.

This sensor can be used in applications like gas leak detection or air quality monitoring.

✅ 4. LDR – Light Dependent Resistor (Photoresistor)
Purpose:
The LDR measures the intensity of light.

Pinout & Connections:
One leg → 5V

Other leg → Analog Pin A1 + 10kΩ resistor to GND

Working Principle:
The resistance of the LDR decreases as the light intensity increases. The more light it receives, the less resistance it has.

By measuring the voltage drop across the LDR (using a voltage divider with a resistor), the Arduino can determine the light level.

This is commonly used in light sensors for automatic lighting or light-sensitive devices.

✅ 5. BMP180 – Barometric Pressure & Temperature Sensor
Purpose:
The BMP180 sensor measures barometric pressure and temperature.

Pinout & Connections:
VCC → 3.3V (or 5V depending on the module)

GND → GND

SDA → A4 (on Arduino Uno)

SCL → A5 (on Arduino Uno)

Working Principle:
The BMP180 uses a piezo-resistive sensor to measure air pressure. The sensor outputs pressure data, which can be used to calculate altitude.

It also has a temperature sensor to measure the ambient temperature. The data from both sensors is processed and sent to the Arduino via I2C communication.

✅ 6. PIR Motion Sensor
Purpose:
The PIR sensor detects motion by sensing infrared radiation (IR) changes in its environment.

Pinout & Connections:
VCC → 5V

GND → GND

OUT → Digital Pin 5

Working Principle:
The PIR sensor detects changes in the infrared radiation emitted by humans or animals. When motion is detected, the sensor sends a HIGH signal to the Arduino.

It is commonly used in security systems, automatic lighting, or human detection applications.

✅ 7. IR Obstacle Avoidance Sensor
Purpose:
The IR sensor detects obstacles or objects in its path using infrared light.

Pinout & Connections:
VCC → 5V

GND → GND

OUT → Digital Pin 6 (or any other digital pin)

Working Principle:
The sensor sends out infrared light from an LED, and if an object is in its path, the light is reflected back and detected by a photodiode.

The sensor outputs a LOW signal when an object is detected and a HIGH signal when no object is detected. This is useful for obstacle detection in robots.

✅ 8. Sound Sensor
Purpose:
The sound sensor detects sound intensity in the environment, usually for detecting loud noises or monitoring sound levels.

Pinout & Connections:
VCC → 5V

GND → GND

AOUT → Analog Pin A2 (for analog output)

DOUT → Digital Pin (optional for threshold-based output)

Working Principle:
The sound sensor typically uses a microphone to detect sound waves. When sound waves hit the microphone, the electrical output changes, and this change is measured.

The Arduino can then read the output voltage, which is proportional to the sound intensity.

✅ 9. Flame Sensor
Purpose:
The flame sensor detects the presence of fire or flames by sensing specific infrared (IR) wavelengths emitted by flames.

Pinout & Connections:
VCC → 5V

GND → GND

DO → Digital Pin 8 (for detecting flame presence)

Working Principle:
The sensor uses a photodiode to detect the infrared radiation emitted by flames.

When a flame is detected, the sensor sends a LOW signal to the Arduino, which can then trigger an alarm or safety mechanism.

✅ 10. Joystick Module
Purpose:
The joystick module provides a simple input device that can detect movement along two axes (X and Y) and a button press.

Pinout & Connections:
VRx → Analog Pin A4

VRy → Analog Pin A5

SW → Digital Pin 7

VCC → 5V

GND → GND

Working Principle:
The joystick has two potentiometers that vary resistance as you move the joystick. These are used to measure the X and Y axes.

The SW button on the joystick detects whether the button is pressed.

✅ 11. Rain Sensor (YL-83 / FC-37)
Purpose:
The rain sensor detects the presence of rain by measuring the conductivity between the sensor’s traces. The more rain, the lower the resistance between the traces.

Pinout & Connections:
VCC → 5V

GND → GND

A0 (Analog Output) → Analog Pin A6 (or any analog input)

Working Principle:
The rain sensor has two conductive traces. When rainwater falls on the sensor, it lowers the resistance between these traces, increasing the conductivity.

The analog output can be read by the Arduino to determine if it is raining or not, based on the resistance level.

✅ 12. LCD 16x2 with I2C
Purpose:
The LCD (Liquid Crystal Display) is used to display information such as sensor readings on a screen.

Pinout & Connections (I2C):
SDA → A4 (on Arduino Uno)

SCL → A5 (on Arduino Uno)

VCC → 5V

GND → GND

Working Principle:
The I2C communication allows you to control the LCD using just two pins (SDA and SCL), reducing the number of connections needed.

You can send data to the display to show text, numbers, or even custom characters. This is great for providing user feedback or displaying sensor values in real-time.

Summary of Connections:
Sensor	Power Pin (VCC)	Ground Pin (GND)	Data Pin/Other Connections
DHT11	5V	GND	Digital Pin 2 (DATA)
HC-SR04	5V	GND	Trigger Pin 3,





