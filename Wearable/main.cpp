#include <MAX30105.h>
#include <heartRate.h>
#include <Wire.h>
#include <spo2_algorithm.h>
#include "BLEDevice.h"

/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.

  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/
// Temperature Setup Constants
const int thermistorPin = 34; // Pin connected to the thermistor
const float referenceVoltage = 3.3;
const float referenceResistor = 10000; // the 'other' resistor
const float beta = 3950; // Beta value (Typical Value)
const float nominalTemperature = 25; // Nominal temperature for calculating the temperature coefficient
const float nominalResistance = 10000; // Resistance value at nominal temperature
float average_temp = 1;
int count = 0;

// Heartrate Setup Constants
MAX30105 particleSensor;

const byte RATE_SIZE = 5;  //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE] = {75, 90, 80, 90, 60};     //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  //Time at which the last beat occurred

float beatsPerMinute = 1;
int beatAvg = 90;


// BLE Setup Constants
// The remote service we wish to connect to.
static BLEUUID serviceUUID("19b10000-e8f2-537e-4f6c-d104768a1214");
// The characteristic of the remote service we are interested in.
//static BLEUUID ATUUID("19b10003-e8f2-537e-4f6c-d104768a1214");
static BLEUUID HRUUID("19b10002-e8f2-537e-4f6c-d104768a1214");
static BLEUUID RTUUID("19b10004-e8f2-537e-4f6c-d104768a1214");
static BLEUUID configUUID("19b10001-e8f2-537e-4f6c-d104768a1214");
//static BLEUUID tsensUUID("19b10005-e8f2-537e-4f6c-d104768a1214");
static BLEUUID timeUUID("19b10006-e8f2-537e-4f6c-d104768a1214");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
//static BLERemoteCharacteristic* pATCharacteristic;
static BLERemoteCharacteristic* pHRCharacteristic;
static BLERemoteCharacteristic* pRTCharacteristic;
static BLERemoteCharacteristic* pConfigCharacteristic;
//static BLERemoteCharacteristic* pTsensCharacteristic;
static BLERemoteCharacteristic* pTimeCharacteristic;
static BLEAdvertisedDevice* myDevice;

BLERemoteService* pRemoteService;

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  pRemoteService = pClient->getService(serviceUUID);
  /*if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }*/
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  //pATCharacteristic = pRemoteService->getCharacteristic(ATUUID);
  pHRCharacteristic = pRemoteService->getCharacteristic(HRUUID);
  pRTCharacteristic = pRemoteService->getCharacteristic(RTUUID);
  pConfigCharacteristic = pRemoteService->getCharacteristic(configUUID);
  //pTsensCharacteristic = pRemoteService->getCharacteristic(tsensUUID);
  //Serial.println("test");
  pTimeCharacteristic = pRemoteService->getCharacteristic(timeUUID);
  /*if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }*/
  //Serial.println(" - Found our characteristics");

  // Read the value of the characteristic.
  /*if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }*/

  /*if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);
    */
  connected = true;
  return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Aircon control module found");
    //Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    }  // Found our server
  }    // onResult
};     // MyAdvertisedDeviceCallbacks

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  Serial.println("MAX30105 found. ");

  // Initialize BLE
  Serial.println("Starting wearable application...");
  BLEDevice::init("");
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  //Init heartRate
  byte ledBrightness = 0x6F;  //Options: 0=Off to 255=50mA
  byte sampleAverage = 2;     //Options: 1, 2, 4, 8, 16, 32, 2 is good value
  byte ledMode = 2;           //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 1000;      //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;       //Options: 69, 118, 215, 411
  int adcRange = 8192;        //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  //Configure sensor with these settings
  // particleSensor.setup();
  // particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  // particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  Serial.println("LETSGO");
  pinMode(thermistorPin, INPUT); // Set pin as input
}

void loop() {
  long irValue = particleSensor.getIR();
  // Serial.println(irValue);
  if (checkForBeat(irValue) == true) {
    // We sensed a beat!
    // Serial.println("i am here");
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);
    // Serial.print(beatsPerMinute);
    // Serial.print(" ");
    
    if ((beatsPerMinute * 2.5) < 120 && (beatsPerMinute * 2.5) > 60) {
      rates[rateSpot++] = (byte)(beatsPerMinute * 2.5);  //Store this reading in the array
      rateSpot %= RATE_SIZE;                     //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      // Serial.print(beatsPerMinute);
      // Serial.print(" ");
    }
    
    pinMode(thermistorPin, INPUT); // Set pin as input
    for (int i = 0; i < 50; i ++) {
      // Serial.println(irValue);
      int adcValue = analogRead(thermistorPin); // Read ADC value
      float voltage = (adcValue * referenceVoltage) / 4095.0; // Calculate voltage
      float resistance = (voltage * referenceResistor) / (referenceVoltage - voltage); // Calculate thermistor resistance with updated configuration

      // Calculate temperature using the Beta parameter equation
      float tempK = 1 / (((log(resistance / nominalResistance)) / beta) + (1 / (nominalTemperature + 273.15)));
      
      float tempC = tempK - 273.15; // Get temperature in Celsius
      float tempF = 1.8 * tempC + 32.0; // Get temperature in Fahrenheit
      average_temp += tempC;
    }
    average_temp /= 50;

    // Serial.print("Detecting Avg BPM=");
    // Serial.println(beatAvg);
    // Serial.print(irValue);

    // if (irValue < 50000)
    //   Serial.print(" No finger?");
    if (doConnect == true) {
      if (connectToServer()) {
        Serial.println("We are now connected to the Aircon control module.");
      } else {
        Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      }
      doConnect = false;
    }
    while (pConfigCharacteristic->readValue() == "")
      ;
    
    pHRCharacteristic->writeValue(std::to_string(beatAvg));
    Serial.print("Sending Average Heartrate Measured: ");
    Serial.println(beatAvg);
    // Serial.print("IR=");
    // Serial.print(irValue);
    // Serial.print(", BPM=");
    // Serial.println(" ");
    pRTCharacteristic->writeValue(std::to_string(average_temp));
    Serial.print("Sending Wrist Temperature Measured: ");
    Serial.println(average_temp);
    while (pTimeCharacteristic->readValue() == "")
      ;
    Serial.print("Time till sending next value: ");
    double time = std::stod(pTimeCharacteristic->readValue());
    Serial.print(time, 2);
    Serial.println(" mins");
    pTimeCharacteristic->writeValue("");

    // If we are connected to a peer BLE Server, update the characteristic each time we are reached
    // with the current time since boot.
    /*if (connected) {
      String newValue = "Time since boot: " + String(millis()/1000);
      Serial.println("Setting new characteristic value to \"" + newValue + "\"");
      
      // Set the characteristic's value to be the array of bytes that is actually a string.
      pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
    }else if(doScan){
      BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
    }*/
    // Serial.println();
  }
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  
}