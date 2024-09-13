#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <DHTesp.h>
#include <examples/DHT_ESP32/DHT_ESP32.ino>

#define IRREMOTEPIN 3
#define DHTPIN 4
#define DHTTYPE DHTesp::DHT11

float setTemp(float body, String model);
float body, out, optimal;
String model;

void setup()
{
  Serial.begin(9600);
  dht.setup(DHTPIN, DHTTYPE);
  model = "Dyson";

  optimal = 25;
}

void loop()
{
  body = dht.getTemperature();
  out = setTemp(body, model);
  Serial.println("Body: " + String(body) + " Out: " + String(out));
  delay(5000);
}

float setTemp(float body, String model)
{
  float k = abs(body - optimal);
  return body < optimal ? optimal + k : optimal - k;
}