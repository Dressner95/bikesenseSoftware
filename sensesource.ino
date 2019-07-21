// This #include statement was automatically added by the Particle IDE.
#include "bme680.h"
#include "Adafruit_BME680.h"
#include "Adafruit_Sensor.h"
#include "bme680_defs.h"

// This #include statement was automatically added by the Particle IDE.
#include "Adafruit_CAP1188.h"

// This #include statement was automatically added by the Particle IDE.
#include "TCA9534.h"

// This #include statement was automatically added by the Particle IDE.
#include "AssetTracker.h"

#include "Wire.h"

SYSTEM_THREAD(ENABLED);

//Four buttons on the top
Adafruit_CAP1188 buttons = Adafruit_CAP1188();

int currentButtons[4] = {0, 0, 0, 0};
int previousButtons[4] = {0, 0, 0, 0};
int buttonPressed[4] = {0, 0, 0, 0};

bool softOff = false;

// How many minutes between publishes? 10+ recommended for long-time continuous publishing!
int delayMinutes = 0.5;

//How many feet before sending GPS
int distanceInterval = 20;
float currentLat;
float currentLon;
float lastLat;
float lastLon;

// Used to keep track of the last time we published data
long lastPublish = 0;

//IO extender for LEDs
TCA9534 leds(0x38);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;

double temperatureInC = 0;
double relativeHumidity = 0;
double pressureHpa = 0;
double gasResistanceKOhms = 0;
double approxAltitudeInM = 0;

//Rest of the Asset Tracker board functions
AssetTracker box = AssetTracker();

FuelGauge fuel;

//Query and store device name on start up
String deviceName;

void handler(const char *topic, const char *data) {
  char dmy[strlen(data) + 1];
  strcpy(dmy, data);
  deviceName = dmy;
}
void setup() {

  waitUntil(Particle.connected);

  //Get device name
  Particle.subscribe("particle/device/name", handler);
  Particle.publish("particle/device/name");

  //Remote functions
  Particle.function("batt", batteryStatus);
  Particle.function("gps", gpsPublish);
  Particle.function("readings", readings);
  Particle.function("PowerOn", powerOn);
  Particle.function("PowerOff", powerOff);

  //Set LED control
  RGB.control(true);
  RGB.brightness(128);

  //Turn on all functions of the board
  box.begin();
  box.gpsOn();
  box.antennaExternal();

  //Initialize communication with the CAP1188
  if (!buttons.begin()) {
    Serial.println("CAP1188 Error");
  }


  //Initialize communication with the TCA9534
  if (!leds.Begin()) {
    Serial.println("TCA Error");
  }
  initLeds();




  //Initialize communication with the BME680
  if (!bme.begin()) {
    Serial.println("BME680 Error");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  // 320*C for 150 ms
  bme.setGasHeater(320, 150);



  Serial.begin(9600);

}

void loop() {


  checkTouch();

  for (int i = 0; i < 4; i++) {
    if (previousButtons[i] == 0 & currentButtons[i] == 1) {
      buttonPressed[i] = 1;

      switch (i) {

        //J2
        case 0:
          powerOn("1");

          Cellular.on();
          Cellular.connect();
          box.gpsOn();

          softOff = false;

          break;

        //J3
        case 1:
          displayBattery();
          break;

        //J4
        case 2:
          powerOff("1");

          Cellular.off();
          box.gpsOff();

          RGB.color(0, 0, 0);

          softOff = true;

          break;

        //J5
        case 3:
          emergency("1");

          readings("E");
          break;
      }
    }
    if (previousButtons[i] == 1 & currentButtons[i] == 0) {
      buttonPressed[i] = 0;
    }
  }

  //Update at end of loop
  for (int i = 0; i < 4; i++) {
    previousButtons[i] = currentButtons[i];
  }

  if (!softOff) {

    //Update the GPS
    box.updateGPS();

    //Wait for cell signal and cloud connection
    if (Particle.connected()) {
      Particle.connect();
      //Now check for GPS
      if (box.gpsFix()) {

  initReadings();

        //When ready = true then the sense box is connected and should begin transmitting
        RGB.color(0, 0, 0);
        RGB.color(0, 255, 0);

        currentLat = box.readLat();
        currentLon = box.readLon();


        /*  if (millis()-lastPublish > delayMinutes*60*1000) {
            readings("Interval");
            lastPublish = millis();
          }
          */

        if ( distance() > distanceInterval) {
          readings("Interval");

            //all ON

        }

      } else {
        RGB.color(0, 0, 0);
        RGB.color(255, 255, 0);

      }


    } else {
      RGB.color(0, 0, 0);
      RGB.color(255, 0, 0);
    }
  }
  else {
    // System.sleep(SLEEP_MODE_DEEP,5);
  }
}

void checkTouch() {

  uint8_t touched = buttons.touched();

  if (touched == 0) {
    // No touch detected
    // return;

    memset(currentButtons, 0, sizeof(currentButtons));

  }

  for (uint8_t i = 0; i < 4; i++) {
    if (touched & (1 << i)) {
      //Serial.print(i+1);

      Serial.print("C"); Serial.print(i + 1); Serial.print("\t");

      currentButtons[i] = 1;

    }
  }
}

void initLeds() {
  for (int i = 0; i < 8; i++) {
    leds.PinMode(i, true);
  }

  for (int i = 0; i < 8; i++) {
    leds.DigitalWrite(i, OUTPUT);
  }
}

int readings(String command) {
  if (! bme.performReading()) {
    return 0;
  } else {
    temperatureInC = bme.temperature;
    relativeHumidity = bme.humidity;
    pressureHpa = bme.pressure / 100.0;
    gasResistanceKOhms = bme.gas_resistance / 1000.0;
    approxAltitudeInM = bme.readAltitude(SEALEVELPRESSURE_HPA);

    String data = String::format(
                    "{"
                    "\"temperatureInC\":%.2f,"
                    "\"humidityPercentage\":%.2f,"
                    "\"pressureHpa\":%.2f,"
                    "\"gasResistanceKOhms\":%.2f"
                    "\"approxAltitudeInM\":%.2f"
                    "}",
                    temperatureInC,
                    relativeHumidity,
                    pressureHpa,
                    gasResistanceKOhms,
                    approxAltitudeInM);

    char dataPayload[120];
    sprintf(dataPayload, "{\"lat\":%f,\"long\":%f,\"temp\":%2.2f,\"voc\":%4f,\"name\":\"" + deviceName + "\"}", box.readLatDeg(), box.readLonDeg(), temperatureInC * 1.8 + 32, gasResistanceKOhms);
    Particle.publish("uploadData", dataPayload, 60, PRIVATE);

    lastLat = box.readLat();
    lastLon = box.readLon();

  for (int i = 0; i < 8; i++) {
    leds.DigitalWrite(i, INPUT);
  }

  delay(500);
    //all ON
  for (int i = 0; i < 8; i++) {
    leds.DigitalWrite(i, OUTPUT);
  }

    //Particle.publish("Sensor", data, 60, PRIVATE);
    return 1;
  }
}

void initReadings() {
  if (! bme.performReading()) {
  } else {
    temperatureInC = bme.temperature;
    relativeHumidity = bme.humidity;
    pressureHpa = bme.pressure / 100.0;
    gasResistanceKOhms = bme.gas_resistance / 1000.0;
    approxAltitudeInM = bme.readAltitude(SEALEVELPRESSURE_HPA);

    String data = String::format(
                    "{"
                    "\"temperatureInC\":%.2f,"
                    "\"humidityPercentage\":%.2f,"
                    "\"pressureHpa\":%.2f,"
                    "\"gasResistanceKOhms\":%.2f"
                    "\"approxAltitudeInM\":%.2f"
                    "}",
                    temperatureInC,
                    relativeHumidity,
                    pressureHpa,
                    gasResistanceKOhms,
                    approxAltitudeInM);
  }

  currentLat = box.readLat();
  currentLon = box.readLon();

  lastLat = box.readLat();
  lastLon = box.readLon();
}

int powerOn(String command) {

  //all OFF
  for (int i = 0; i < 8; i++) {
    leds.DigitalWrite(i, OUTPUT);
  }

  leds.DigitalWrite(4, INPUT);
  delay(500);
  leds.DigitalWrite(3, INPUT);
  leds.DigitalWrite(5, INPUT);
  delay(500);
  leds.DigitalWrite(2, INPUT);
  leds.DigitalWrite(6, INPUT);
  delay(500);
  leds.DigitalWrite(1, INPUT);
  leds.DigitalWrite(7, INPUT);
  delay(500);
  leds.DigitalWrite(0, INPUT);

  delay(1000);

  //all off
  for (int i = 0; i < 8; i++) {
    leds.DigitalWrite(i, OUTPUT);
  }

  return 1;
}

int powerOff(String command) {

  //all ON
  for (int i = 0; i < 8; i++) {
    leds.DigitalWrite(i, INPUT);
  }

  delay(1000);
  leds.DigitalWrite(0, OUTPUT);
  delay(500);
  leds.DigitalWrite(1, OUTPUT);
  leds.DigitalWrite(7, OUTPUT);
  delay(500);
  leds.DigitalWrite(2, OUTPUT);
  leds.DigitalWrite(6, OUTPUT);
  delay(500);
  leds.DigitalWrite(3, OUTPUT);
  leds.DigitalWrite(5, OUTPUT);
  delay(500);
  leds.DigitalWrite(4, OUTPUT);

  return 1;
}

int emergency(String command){

      //all OFF
  for (int i = 0; i < 8; i++) {
    leds.DigitalWrite(i, OUTPUT);
  }

  for (int i = 0; i < 5; i++) {
    leds.DigitalWrite(1, INPUT);
    leds.DigitalWrite(7, INPUT);
    leds.DigitalWrite(3, INPUT);
    leds.DigitalWrite(5, INPUT);

    delay(500);
    //all off
    for (int i = 0; i < 8; i++) {
      leds.DigitalWrite(i, OUTPUT);
    }
    leds.DigitalWrite(2, INPUT);
    leds.DigitalWrite(6, INPUT);
    leds.DigitalWrite(4, INPUT);
    leds.DigitalWrite(0, INPUT);

    delay(500);
    //all off
    for (int i = 0; i < 8; i++) {
      leds.DigitalWrite(i, OUTPUT);
    }
  }
  //all off
  for (int i = 0; i < 8; i++) {
    leds.DigitalWrite(i, OUTPUT);
  }

    return 1;
}

void displayBattery() {
  int ledsToShow;
  ledsToShow = map((int)fuel.getSoC(), 0, 100, 1 , 8);

  for (int i = 0; i < ledsToShow; i++) {
    //On
    leds.DigitalWrite(i, INPUT);
    delay(200);
  }

  delay(1000);

  for (int i = 0; i < 8; i++) {
    //Off
    leds.DigitalWrite(i, OUTPUT);

  }

}

int batteryStatus(String command) {
  Particle.publish("B",
                   "v:" + String::format("%.2f", fuel.getVCell()) +
                   ",c:" + String::format("%.2f", fuel.getSoC()),
                   60, PRIVATE
                  );
  // if there's more than 10% of the battery left, then return 1
  if (fuel.getSoC() > 10) {
    return 1;
  }
  // if you're running out of battery, return 0
  else {
    return 0;
  }
}

int gpsPublish(String command) {
  if (box.gpsFix()) {
    Particle.publish("G", box.readLatLon(), 60, PRIVATE);
    return 1;
  } else {
    return 0;
  }
}
/*
//Calculates distance between current and last reported gps cord
float distance() {
  float deltaLat = currentLat - lastLat;
  float deltaLon = currentLon - lastLon;
  float distanceBetween = pow(deltaLat, 2) + pow(deltaLon, 2);
  distanceBetween = sqrt(distanceBetween) * 100000;
  return distanceBetween;
}
*/

float distance(){
float R = 20902000;
float deltaLat = (currentLat - lastLat) * (PI/180);
float deltaLon = (currentLon - lastLon) * (PI/180);

float a = sin(deltaLat/2) * sin(deltaLat/2) + cos((lastLat) * (PI/180)) * cos((lastLon)* (PI/180)) * sin(deltaLon/2) * sin(deltaLon/2) ;
float c = 2 * atan2(sqrt(a),sqrt(1-a));

float d = R * c;
return d;
}
