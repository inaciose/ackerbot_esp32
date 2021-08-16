#include <Arduino.h>

#define LED 2

void setup() {
  pinMode(LED,OUTPUT);
  Serial.begin(115200);
}

void loop() {
  delay(500);
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
  Serial.println("Hello world!");
}