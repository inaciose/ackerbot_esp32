#include <Arduino.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "wifi.h"

#define LED_BUILTIN 2
#define LED_TIMER 500

//#define ENABLE_MPU6050 //if want use oled,turn on this macro
#define ENABLE_OLED //if want use oled,turn on this macro

#if defined(ENABLE_OLED) || defined(ENABLE_MPU6050)
  #include <Wire.h>
#endif

#ifdef ENABLE_OLED
// oled
#include "SSD1306Wire.h"
#define OLED_ADDRESS 0x3c
#define I2C_SDA 14
#define I2C_SCL 15
SSD1306Wire display(OLED_ADDRESS, SDA, SCL, GEOMETRY_128_64);
bool hasDisplay = false; // we probe for the device at runtime
#endif

void lcdMessage(String msg) {
  #ifdef ENABLE_OLED
    if(hasDisplay) {
        display.clear();
        display.drawString(128 / 2, 32 / 2, msg);
        display.display();
    }
  #endif
}

void blinkLedBuiltin() {
	static boolean ledstate = 0;
	static long unsigned int ledTimer = 0;
	if(millis() >= ledTimer) {
		ledTimer = millis() + LED_TIMER;
		ledstate = !ledstate;
		digitalWrite(LED_BUILTIN, ledstate);
	}
}

void setup() {

  // WIFI mode
  int softap_mode = 0;
  // client mode ip
  IPAddress ip;
  // access point mode ip
  IPAddress apIP = IPAddress(192, 168, 4, 1);

  // led config
  pinMode(LED_BUILTIN, OUTPUT);

  // serial config
  Serial.begin(115200);
  Serial.println("Booting");

  #ifdef ENABLE_OLED
    hasDisplay = display.init();
    if(hasDisplay) {
        display.flipScreenVertically();
        display.setFont(ArialMT_Plain_16);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.setContrast(255);
    }  
    lcdMessage("booting");
  #endif

  // WIFI
  if(softap_mode) {
    const char *hostname = "akbesp";
    lcdMessage("starting softAP");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    bool result = WiFi.softAP(hostname, "12345678", 1, 0);
    if (!result) {
      Serial.println("AP Config failed.");
      return;
    } else {
      Serial.println("AP Config Success.");
      Serial.print("AP MAC: ");
      Serial.println(WiFi.softAPmacAddress());
      ip = WiFi.softAPIP();
    }
  } else {
    lcdMessage(String("join ") + ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(F("."));
    }
    ip = WiFi.localIP();
    Serial.println(F("WiFi connected"));
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);

      #ifdef ENABLE_OLED
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        display.drawString(display.getWidth() / 2, display.getHeight() / 2 - 10, "OTA Update");
        display.display();
      #endif

    })
    .onEnd([]() {
      Serial.println("\nEnd");
      #ifdef ENABLE_OLED
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
        display.drawString(display.getWidth() / 2, display.getHeight() / 2, "OTA done: Restart");
        display.display();
      #endif
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      #ifdef ENABLE_OLED
        display.drawProgressBar(4, 32, 120, 8, progress / (total / 100) );
        display.display();
      #endif
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      #ifdef ENABLE_OLED
        if (error == OTA_AUTH_ERROR) lcdMessage("OTA error: Auth Failed");
        else if (error == OTA_BEGIN_ERROR) lcdMessage("OTA error: Begin Failed");
        else if (error == OTA_CONNECT_ERROR) lcdMessage("OTA error: Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) lcdMessage("OTA error: Receive Failed");
        else if (error == OTA_END_ERROR) lcdMessage("OTA error: End Failed");
        else lcdMessage("OTA error: other");
      #endif

    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(ip);
  lcdMessage(ip.toString());

}

void loop() {

  blinkLedBuiltin();
  ArduinoOTA.handle();
}