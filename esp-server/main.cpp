#include <Arduino.h>
#include "BluetoothSerial.h"
// #include <IRremote.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <math.h>
#include <string.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLE2902.h>
//#include <LiquidCrystal_I2C.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

int recv = 0;
static double heartrate = 0;
static double rawTemp = 0;
static double airTemp = 0;

// as server
BLEServer *pServer = NULL;
BLECharacteristic *pConfigCharacteristic = NULL;
BLECharacteristic *pATCharacteristic = NULL;
BLECharacteristic *pHRCharacteristic = NULL;
BLECharacteristic *pRTCharacteristic = NULL;
//BLECharacteristic *pTsensCharacteristic = NULL;
BLECharacteristic *pTimeCharacteristic = NULL;
bool deviceConnected = false;
// bool ringConnected = false;
static std::string airconType = "";

#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CONFIG_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define HR_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"
#define AT_CHARACTERISTIC_UUID "19b10003-e8f2-537e-4f6c-d104768a1214"
#define RT_CHARACTERISTIC_UUID "19b10004-e8f2-537e-4f6c-d104768a1214"
//#define TSENS_CHARACTERISTIC_UUID "19b10005-e8f2-537e-4f6c-d104768a1214"
#define TIME_CHARACTERISTIC_UUID "19b10006-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("connected");
    if (pServer->getConnectedCount() <= 1)
    {
      BLEDevice::startAdvertising();
    }
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pConfigCharacteristic)
  {
    airconType = static_cast<std::string>(pConfigCharacteristic->getValue());
  }
};

class MyATCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    airTemp = std::stod(pCharacteristic->getValue());
    Serial.print("Air Temperature received: ");
    Serial.println(airTemp);
  }
};

class MyHRCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    heartrate = std::stod(pCharacteristic->getValue());
    Serial.print("Heartrate received: ");
    Serial.println(heartrate);
  }
};

class MyRTCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    rawTemp = std::stod(pCharacteristic->getValue());
    Serial.print("Wrist Temperature received: ");
    Serial.println(rawTemp);
  }
};

class MyTimeCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
  }
};

BluetoothSerial SerialBT;
uint8_t address[6] = {0x96, 0xC5, 0xDB, 0xF8, 0xEC, 0x07};
String message = "";
const int IR_EMIT = 4; // yellow
// const int IR_REC = 2; //red

const double BASAL_HEART_RATE = 65.0;
const double NEUTRAL_CORE_TEMP = 36.8;
const double NEUTRAL_SKIN_TEMP = 33.7;

IRsend irsend(IR_EMIT);
// IRrecv irrecv(IR_REC);
// decode_results results;

double getTime(double tsens) {
  if (abs(tsens) < 0.5) {
    return 15.0;
  }
  if (abs(tsens) > 3) {
    return 1.0;
  }
  return -7.688 * log(abs(tsens)) + 9.8616;
}

// metabolic rate in watts
double metabolic(double heartrate, double airTemp)
{
  double hrr = heartrate / BASAL_HEART_RATE;
  return 58.15 * (0.68 + 4.69 * (hrr - 1.0) - 0.052 * (hrr - 1.0) * (airTemp - 20.0));
}

double meanSkinTemp(double rawTemp)
{
  return -0.0844 * rawTemp * rawTemp + 6.5457 * rawTemp - 90.539;
}

double coreTemp(double heartrate)
{
  return 36.8 + (1.0 / 33.0) * (heartrate - BASAL_HEART_RATE);
}

double bloodPurfusionRate(double meanSkinTemp, double coreTemp)
{
  // warm thermal signals
  double wsig = max(0.0, (coreTemp - NEUTRAL_CORE_TEMP));
  // cold thermal signals
  double csig = max(0.0, (NEUTRAL_SKIN_TEMP - meanSkinTemp));
  return (6.3 + 200.0 * wsig) / (3600.0 * (1.0 + 0.5 * csig));
}

double tsens(double rawTemp, double heartrate, double airTemp)
{
  double M = metabolic((double)heartrate, (double)airTemp);
  // lower limit of evaporation regulation
  double tbc = ((0.19 / 58.15) * M) + 36.601;
  // upper limit of evaportion regulation
  double tbh = ((0.347 / 58.15) * M) + 36.669;
  // sweating evaportation efficiency
  // double ne = 1.0 / (1.0 + ((8.63 + 0.046 * airTemp) * (1.73 - 0.04 * airTemp) / 6.45));
  double ne = 1.0 / (1.0 + ((7.7 + 0.046 * airTemp) * (2.08 - 0.05 * airTemp) / 6.45));
  double mst = meanSkinTemp((double)rawTemp);
  double tcr = coreTemp((double)heartrate);
  // Serial.print("ne");
  // Serial.println(ne);
  // Serial.print("mst");
  // Serial.println(mst, 10);
  // Serial.print("core");
  // Serial.println(tcr, 10);
  double mbl = bloodPurfusionRate(mst, tcr);
  // transient value of skin to body mass fraction ratio
  double alpha = 0.0418 + (0.745 / (3600.0 * mbl + 0.585));
  double overallBodyTemp = alpha * mst + (1 - alpha) * tcr;
  // Serial.print("overallbt");
  // Serial.println(overallBodyTemp);
  // Serial.print("tbc");
  // Serial.println(tbc);
  // Serial.print("tbh");
  // Serial.println(tbh);
  if (overallBodyTemp < tbc)
  {
    return 0.4685 * (overallBodyTemp - tbc);
  }
  else if (overallBodyTemp <= tbh)
  {
    return 4.7 * ne * ((overallBodyTemp - tbc) / (tbh - tbc));
  }
  return 4.7 * ne + 0.4685 * (overallBodyTemp - tbh);
}

/*int airconTemp(double rawTemp, double heartrate) {
  double minimumTsens = 10.0;
  int minTemp = 0;
  for (double i = 0; i < 100; i++) {
    if (abs(tsens(rawTemp, heartrate, i)) < minimumTsens) {
      minimumTsens = abs(tsens(rawTemp, heartrate, i));
      minTemp = i;
    }
    Serial.print(i);
    Serial.print(": ");
    Serial.println(tsens(rawTemp, heartrate, i));
  }
  Serial.print("tsens: ");
  Serial.println(minimumTsens, 10);
  Serial.print("minTemp: ");
  Serial.println(minTemp);
  return minTemp;
}*/

void setup()
{
  Serial.begin(9600);
  BLEDevice::init("ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pConfigCharacteristic = pService->createCharacteristic(
      CONFIG_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  pConfigCharacteristic->setValue("");
  pRTCharacteristic = pService->createCharacteristic(
      RT_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  pHRCharacteristic = pService->createCharacteristic(
      HR_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  /*pATCharacteristic = pService->createCharacteristic(
      AT_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);*/
  /*pTsensCharacteristic = pService->createCharacteristic(
      TSENS_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  pTsensCharacteristic->setValue("");*/
  pTimeCharacteristic = pService->createCharacteristic(
      TIME_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  pTimeCharacteristic->setValue("");

  pConfigCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  //pATCharacteristic->setCallbacks(new MyATCallbacks());
  pHRCharacteristic->setCallbacks(new MyHRCallbacks());
  pRTCharacteristic->setCallbacks(new MyRTCallbacks());
  pTimeCharacteristic->setCallbacks(new MyTimeCallbacks());
  pConfigCharacteristic->addDescriptor(new BLE2902());
  //pATCharacteristic->addDescriptor(new BLE2902());
  pHRCharacteristic->addDescriptor(new BLE2902());
  pRTCharacteristic->addDescriptor(new BLE2902());
  //pTsensCharacteristic->addDescriptor(new BLE2902());
  pTimeCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter

  // pAdvertising->setScanFilter(true, true);
  BLEDevice::startAdvertising();

  irsend.begin();
  // irrecv.enableIRIn();
  pinMode(IR_EMIT, OUTPUT);
  // pinMode(IR_REC, INPUT);
  //SerialBT.begin("ESP32 Aircon");
}

void loop()
{
  while (deviceConnected)
  {
    while (airconType == "")
      ;
    Serial.println(airconType.c_str());
    deviceConnected = false;
    // BLEDevice::deinit(false);
    // delay(1000);
    // SerialBT.begin("Aircon ESP32"); //Bluetooth device name
    // BLEDevice::init("");
    /*BLEClient* pClient = BLEDevice::createClient();
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);*/
    // connect = SerialBT.connect(address);
    // while (!connect) {
    //   Serial.println("tryiing");
    //   connect = SerialBT.connect(address);
    // }
    // Serial.println("connect to hc-05");
    // SerialBT.write(100);
  }
  /*if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }*/

  // writing to the other person
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  // receiving from the other person
  //if (SerialBT.available()) {
  //  Serial.write(SerialBT.read());
  //}
  if (SerialBT.available()) {
    char incoming = SerialBT.read();
    if (incoming != '\n') {
      message += String(incoming);
    } else {
      message = "";
    }
  }

  if (message[message.length() - 1] == 'T') {
    String rT = message.substring(0, message.length() - 1);
    Serial.println("Temperature Measured: " + rT);
    rawTemp = rT.toDouble();
    Serial.printf("rawTemp");
    Serial.println(rawTemp);
  }

  if (message[message.length() - 1] == 'H') {
    String hr = message.substring(0, message.length() - 1);
    heartrate = hr.toDouble();
    Serial.printf("heartRate");
    Serial.println(heartrate);
  }
  if (message[message.length() - 1] == 'A') {
    String at = message.substring(0, message.length() - 1);
    airTemp = at.toDouble();
    Serial.printf("airTemp");
    Serial.println(airTemp);
  }


  if (rawTemp > 0 &&  heartrate > 0 && airTemp > 0)
  {
    // compute tsens with forumlas
    double thermalSensation = tsens(rawTemp, heartrate, airTemp);
    // int tempToSet = airconTemp(rawTemp, heartrate);
    // determine what to do with the ir sensor
    rawTemp = 0.0;
    heartrate = 0.0;
    airTemp = 0.0;
    Serial.print("TSENS Measured: ");
    Serial.println(thermalSensation, 10);
    double time = getTime(thermalSensation);
    if (thermalSensation > 0.5)
    {
      irsend.sendNEC(0x12000000, 32, 10);
    }
    else if (thermalSensation < -0.5)
    {
      irsend.sendNEC(0x34000000, 32, 10);
    }
    else
    {
      irsend.sendNEC(0x56000000, 32, 10);
    }
    //pTsensCharacteristic->setValue(thermalSensation);
    pTimeCharacteristic->setValue(std::to_string(time));
    Serial.print("Time till next data: ");
    Serial.print(time, 2);
    Serial.println(" mins");
    recv++;
  }

  if (rawTemp > 0 && heartrate > 0) {
    Serial.println("Measuring Air Temperature...");
    Serial.println("Air Temperature Measured: 27.8");
    airTemp = 27.8;
  }

  // Serial.println(airTemp + rawTemp + heartrate);
  delay(20);
  if (recv == 6) {
    SerialBT.begin("ESP32 Aircon");
  }
}