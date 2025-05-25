# DHT Sensor Library by Nexon Shimwa

Welcome to my customized DHT sensor library for Arduino. This library supports popular temperature and humidity sensors such as **DHT11**, **DHT22**, and **DHT21**.

🔗 **GitHub Pages Demo/Docs**:  
[https://nexonshimwa.github.io/DHT-sensor-library-master-by-nexon/](https://nexonshimwa.github.io/DHT-sensor-library-master-by-nexon/)

---

## 📦 Features

- Supports DHT11, DHT22, and DHT21 sensors
- Works with Arduino Uno, Mega, Nano, ESP32, ESP8266, etc.
- Clean code and well-commented
- Example sketches included for easy usage

---

## 📥 Installation

### Option 1: Install via Arduino IDE (ZIP)
1. Go to [Releases](https://github.com/nexonshimwa/DHT-sensor-library-master-by-nexon/releases).
2. Download the latest `.zip` release.
3. In Arduino IDE, go to **Sketch > Include Library > Add .ZIP Library...**
4. Select the downloaded file and click **Open**.

### Option 2: Manual Install
1. Download this repository as `.zip`.
2. Extract it.
3. Rename the folder to `DHTSensorByNexon`.
4. Move it to your `Documents/Arduino/libraries` folder.
5. Restart Arduino IDE.

---

## 🧪 How to Use

```cpp
#include "DHT.h"

#define DHTPIN 2        // Pin where your sensor is connected
#define DHTTYPE DHT11   // DHT11, DHT22, or DHT21

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");

  delay(2000);
}
