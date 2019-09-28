SYSTEM_MODE(SEMI_AUTOMATIC);

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

PRODUCT_ID(9894);
PRODUCT_VERSION(7);

enum states {
    STARTUP,
    BUTTONCHECK,
    SLEEP,
    SEARCHING,
    CONNECTED,
    FULLPOWER,
    DATACOLLECTION
};

states currentState = STARTUP;

unsigned long previousMillis = 0;

//Four buttons on the top
Adafruit_CAP1188 buttons = Adafruit_CAP1188();

int currentButtons[4] = {0, 0, 0, 0};
int previousButtons[4] = {0, 0, 0, 0};
int buttonPressed[4] = {0, 0, 0, 0};

bool softOff = false;

// How many minutes between publishes? 10+ recommended for long-time continuous publishing!
int delayMinutes = 0.5;

//How many feet before sending GPS
double distanceInterval = 20;
double currentLat;
double currentLon;
double lastLat;
double lastLon;

// Used to keep track of the last time we published data
long lastPublish = 0;
long buttonTime = 0;
long serialTimer = 3000;
float holdTime = 1000;
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

void setup(){

  Serial.begin(9600);
  //Set LED control
  RGB.control(true);
  RGB.brightness(64);

  //Connect to the asset tracker and make sure it's off
    box.begin();
    Cellular.off();
    box.gpsOff();

    //Initialize communication with the CAP1188
    if (!buttons.begin()) {
      Serial.println("CAP1188 Error");

      RGB.color(0, 0, 0);
      RGB.color(255, 0, 0);

      delay(555);
    }


    //Initialize communication with the BME680
    //It sleeps after start up
    if (!bme.begin()) {
      Serial.println("BME680 Error");
      RGB.color(0, 0, 0);
      RGB.color(0, 0, 255);
        delay(555);
    }


}

void loop(){
  switch (currentState){
    case STARTUP:
    Serial.println(currentState);
    //quick delay to sort out the CAP1188
    delay(200);
    previousMillis = millis();
    RGB.color(0, 0, 0);
    RGB.color(0, 128, 128);
    currentState = BUTTONCHECK;
    break;//end of start up

    case BUTTONCHECK:
    Serial.println(currentState);


    //run a check on cap pads
    checkTouch();

    for (int i = 0; i < 4; i++) {
      if (previousButtons[i] == 0 && currentButtons[i] == 1) {
        buttonPressed[i] = 1;
        buttonTime = millis();
      } else if (previousButtons[i] == 1 && currentButtons[i] == 0){
        buttonPressed[i] = 0;
      }
    }

    if(currentButtons[2]){
      RGB.color(0, 0, 0);
      RGB.color(0, 0, 255);
    } else {
      RGB.color(0, 0, 0);
      RGB.color(0, 128, 128);
    }

    if (buttonPressed[2] && millis() - buttonTime > 500){


      //Initialize communication with the TCA9534
      if (!leds.Begin()) {
        Serial.println("TCA Error");
        RGB.color(0, 0, 0);
        RGB.color(255, 0, 255);
          delay(555);
      }
      initLeds();
      currentState = FULLPOWER;
    }

    if (millis() - previousMillis > 1500 && !buttonPressed[2] || millis() - buttonTime > 3000){
      currentState = SLEEP;
    }

    //reset button arrays
    for (int i = 0; i < 4; i++) {
      previousButtons[i] = currentButtons[i];
    }

    for (int i = 0; i < 4; i++) {
      if (previousButtons[i] == 0 && currentButtons[i] == 0) {
          buttonPressed[i] = 0;
    }
  }
    break; //end of SLEEP

    case FULLPOWER:
    Serial.println(currentState);


    powerOn("1");

    Cellular.on();

    Cellular.connect();
    box.gpsOn();
    box.antennaExternal();

    currentState = SEARCHING;
    break;//end of FULLPOWER

    case SEARCHING:
    Serial.println(currentState);

    //Cellular Check
    if(!Cellular.ready() && !Cellular.connecting()){
      Cellular.connect();
    }
    if(Cellular.connecting()){
      //Do nothing just wait
      RGB.color(0, 0, 0);
      RGB.color(255, 0, 0);
    }

    if(Cellular.ready()){
        //Particle Check
        if (Particle.connected()){
          currentState = CONNECTED;
        } else {
          Particle.connect();
        }
    }


    //Update the GPS
    box.updateGPS();

    checkTouch();

    for (int i = 0; i < 4; i++) {
      if (previousButtons[i] == 0 && currentButtons[i] == 1) {
        buttonPressed[i] = 1;
        buttonTime = millis();
      } else if (previousButtons[i] == 1 && currentButtons[i] == 0){
        buttonPressed[i] = 0;
      }
    }

    if(buttonPressed[0] && millis() - buttonTime > 1000){
    powerOff("0");

    Cellular.off();
    box.gpsOff();

    currentState = SLEEP;
    }

    //Show battery
    if(buttonPressed[1] && !previousButtons[1]){
      displayBattery();
    }

    //reset button arrays
    for (int i = 0; i < 4; i++) {
      previousButtons[i] = currentButtons[i];
    }

    for (int i = 0; i < 4; i++) {
      if (previousButtons[i] == 0 && currentButtons[i] == 0) {
          buttonPressed[i] = 0;
    }
  }


    break; //END OF SEARHCING

    case SLEEP:
    Serial.println(currentState);
    RGB.color(0, 0, 0);
    Cellular.off();
    box.gpsOff();
    System.sleep(SLEEP_MODE_SOFTPOWEROFF,10,SLEEP_DISABLE_WKP_PIN);
    currentState = STARTUP;
    break; //end of SLEEP

    case CONNECTED:
     Serial.println(currentState);
      //Get device name
     Particle.subscribe("particle/device/name", handler);
     Particle.publish("particle/device/name");

     //Remote functions
     Particle.function("batt", batteryStatus);
     Particle.function("gps", gpsPublish);
     Particle.function("readings", readings);
     Particle.function("PowerOn", powerOn);
     Particle.function("PowerOff", powerOff);

    RGB.color(0, 0, 0);
    RGB.color(128, 128, 0);

    currentState = DATACOLLECTION;
    break; //end of CONNECTED

    case DATACOLLECTION:
    Serial.println(currentState);
    //Update the GPS
    box.updateGPS();

    checkTouch();

    for (int i = 0; i < 4; i++) {
      if (previousButtons[i] == 0 && currentButtons[i] == 1) {
        buttonPressed[i] = 1;
        buttonTime = millis();
      } else if (previousButtons[i] == 1 && currentButtons[i] == 0){
        buttonPressed[i] = 0;
      }
    }

    //Show battery
    if(buttonPressed[1] && !previousButtons[1]){
      displayBattery();
    }

    //Emergency
    if(buttonPressed[3] && (millis() - buttonTime > holdTime)){
      emergency("1");

      readings("E");

      buttonPressed[3] = 0;
    }

    //Turn "off" unit
    if(buttonPressed[0] && (millis() - buttonTime > holdTime)){
      powerOff("1");

      Cellular.off();
      box.gpsOff();

      currentState = SLEEP;
    }

    //Now check for GPS
    if (box.gpsFix()) {

      RGB.color(0, 255, 0);

      currentLat = box.readLatDeg();
      currentLon = box.readLonDeg();

      if ( distance() > distanceInterval) {
        readings("I");

          //all ON

      }
    } else {
      RGB.color(0, 0, 0);
      RGB.color(128, 128, 0);
    }


    //DO NOT WRITE AFTER THIS BLOCK
    //Update at end of loop
    for (int i = 0; i < 4; i++) {
      previousButtons[i] = currentButtons[i];
    }

    for (int i = 0; i < 4; i++) {
      if (previousButtons[i] == 0 && currentButtons[i] == 0) {
          buttonPressed[i] = 0;
      }
    }

    break; //end of DATACOLLECTION
  } //end of switch
} //end of loop

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

      //Serial.print("C"); Serial.print(i + 1); Serial.print("\t");

      currentButtons[i] = 1;

    }
    else{
      currentButtons[i] = 0;
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
    sprintf(dataPayload, "{\"lat\":%f,\"long\":%f,\"temp\":%2.2f,\"voc\":%4f,\"type\":\"" + command + "\",\"name\":\"" + deviceName + "\"}", box.readLatDeg(), box.readLonDeg(), temperatureInC * 1.8 + 32, gasResistanceKOhms);
    Particle.publish("maker", dataPayload, 60, PRIVATE);

    lastLat = box.readLatDeg();
    lastLon = box.readLonDeg();

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
/*
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
*/

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
  float deltaLon = currentLon - ;
  float distanceBetween = pow(deltaLat, 2) + pow(deltaLon, 2);
  distanceBetween = sqrt(distanceBetween) * 100000;
  return distanceBetween;
}
*/

double distance(){
double R = 20902000;
double deltaLat = (currentLat - lastLat) * (PI/180);
double deltaLon = (currentLon - lastLon) * (PI/180);

double a = sin(deltaLat/2) * sin(deltaLat/2) + cos((lastLat) * (PI/180)) * cos((lastLon)* (PI/180)) * sin(deltaLon/2) * sin(deltaLon/2) ;
double c = 2 * atan2(sqrt(a),sqrt(1-a));

double d = R * c;
return d;
}
