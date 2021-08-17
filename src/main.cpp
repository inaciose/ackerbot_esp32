#include <Arduino.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "wifi.h"

#define LED_BUILTIN 2
#define LED_TIMER 500

#define BUTTON1_PIN 34
#define BUTTON2_PIN 35
#define BUTTON3_PIN 5
int button1 = false;
int button2 = false;
int button3 = false;

#define ENABLE_OLED       // if we want use OLED, turn on this macro
//#define ENABLE_MPU6050  // if want use MPU6050, turn on this macro
#define ENABLE_GPS        // if want use ENABLE_GPS

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

#ifdef ENABLE_GPS
  #include <TinyGPS++.h>
  #define RXD2 16
  #define TXD2 17
  TinyGPSPlus gps;
#endif

#include "pid.h"

// servo
#include <ESP32Servo.h>
Servo servo1;
int servo1Angle = 90;
int servo1Pin = 15;

// bridge-h
int ENA = 27;
int IN1 = 14;
int IN2 = 13;

#define WPWMRANGE 256
#define WPWMFREQ 1000
int motorState = 0;
//int motorSpeed = 0;
signed int leftMotorPwmOut = 0;
//signed int leftMotorPwmOutCmd = 0;

// quadrature encoder
#define ENCODER_LEFT_FUNCTIONA encoderLeftCounterA
#define ENCODER_LEFT_FUNCTIONB encoderLeftCounterB
#define ENCODER_LEFT_PINA 19
#define ENCODER_LEFT_PINB 18
#define ENCODER_LEFT_SIGNAL CHANGE
byte encoderLeftPinLast; // control
volatile signed int encoderLeftPulses; // the number of pulses
boolean encoderLeftDirection; // the rotation direction

long encoderLeftPulsesTarget = 0;
long encoderRightPulsesTarget = 0;
bool encoderLeftPulsesOnTarget = false;
bool encoderRightPulsesOnTarget = false;
bool encoderPulsesTargetEnabled = false;
int encoderLeftPulsesTargetStopOffset = 0;
int encoderRightPulsesTargetStopOffset = 0;

long encoderLeftPulsesTargetStart = 0;
long encoderRightPulsesTargetStart = 0;

// pid
volatile long encoderLeftPulsesSpeedPID; // the number of pulses for PID
volatile long encoderLeftPulsesSteeringPID; // the number of pulses for PID, must be reset at same time

float wheelbase = 0.21; // meters
float wheelradius = 0.033; // meters

// pulses per rotation
int ppr = 2000;
// pulses per meter
int ppm = 0;
// meters per pulse
float mpp = 0;
// meters per wheel rotation
float mpr = 0;

// instantaneous speed
float speed = 0.0;
// speed on second
float speedOs = 0.0;

int speedLastPulses = 0;
int speedOsLastPulses = 0;

// bof:ROS
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Int16.h>

// Set the rosserial socket server IP address
//IPAddress server(192,168,1,64);
int serverIp = 60; // 64
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;

// encoder pulses publisher
std_msgs::Int32 encoderLeftPulsesPub;
ros::Publisher encoderPublisher("wheel", &encoderLeftPulsesPub);
#define ENCODER_PUB_TIMER 100

// mcu ip publisher
//std_msgs::String ip_msg;
//ros::Publisher ipPublisher("mcu_ip", &ip_msg);
//#define IP_PUB_TIMER 10000

void encoderPub() {
	static long unsigned int encoderPubTimer = 0;
	if(millis() >= encoderPubTimer) {
		encoderPubTimer = millis() + ENCODER_PUB_TIMER;
		encoderLeftPulsesPub.data = encoderLeftPulses;
		encoderPublisher.publish( &encoderLeftPulsesPub );
	}
}

/*
void ipPub() {
	static long unsigned int ipPubTimer = 0;
	if(millis() >= ipPubTimer) {
		ipPubTimer = millis() + IP_PUB_TIMER;
		ip_msg.data = WiFi.localIP().toString().c_str();
		ipPublisher.publish( &ip_msg );
	}
}
*/

float steering_angle = 0;
float velocity_target = 0;

float twist_angular = 0;
float twist_linear = 0;

long unsigned int secondTimer = 0;
long unsigned int loopInfoTimer = 0;

void setPidTarget() {
  int encoderPulsesTarget = 0;

  leftSpeedPidSetPointTmp = -leftSpeedPidSetPointTmp;

	/*
	Serial.print(leftSpeedPidSetPointTmp);
	Serial.println("\n");
	*/

	// pid
	//leftSpeedPidSetPointTmp = velocity_target;
	// set left motors direction and speed
	if(leftSpeedPidSetPointTmp > 0) {
		leftSpeedPidSetPointDirection = 1;
		leftSpeedPidSetPoint = round(leftSpeedPidSetPointTmp);
	} else if(leftSpeedPidSetPointTmp < 0) {
		leftSpeedPidSetPointDirection = -1;
		leftSpeedPidSetPoint = round(abs(leftSpeedPidSetPointTmp));
	} else {
		leftSpeedPidSetPointDirection = 0;
		leftSpeedPidSetPoint = round(leftSpeedPidSetPointTmp);
	}

	leftSpeedPid.SetMode(AUTOMATIC);
	//useSteeringPid = false;
	//encoderLeftPulsesSteeringPID = encoderRightPulsesSteeringPID = 0;

	// set the target for encoders if any
	if(encoderPulsesTarget) {
		// set left encoder target
		if(leftSpeedPidSetPointDirection >= 0) {
			//encoderLeftPulsesTarget = encoderLeftPulses + (float)(abs(encoderLeftPulsesTarget) / 1000.0 * pulses_per_m);
			encoderLeftPulsesTarget = encoderLeftPulses + abs(encoderPulsesTarget);    
		} else {
			//encoderLeftPulsesTarget = encoderLeftPulses - (float)(abs(encoderLeftPulsesTarget) / 1000.0 * pulses_per_m);
			encoderLeftPulsesTarget = encoderLeftPulses - abs(encoderPulsesTarget);
		}

		encoderLeftPulsesOnTarget = false;
		encoderPulsesTargetEnabled = true;
		// debug only
		encoderLeftPulsesTargetStart = encoderLeftPulses;
	} else {
		encoderLeftPulsesTarget = 0;
		encoderLeftPulsesOnTarget = false;
		encoderPulsesTargetEnabled = false;
		// debug only
		encoderLeftPulsesTargetStart = 0;
	}
}

void twistMsgCb(const geometry_msgs::Twist& msg) {
	
	velocity_target = msg.linear.x;
	twist_linear = msg.linear.x;
	twist_angular = msg.angular.z;

	if(velocity_target == 0 || msg.angular.z == 0) {
		steering_angle = 0;
	} else {
		float radius = velocity_target / msg.angular.z;
		steering_angle = atan(wheelbase / radius);
	}

	steering_angle = -steering_angle;

	// dx = (l + r) / 2
	// dr = (r - l) / w
	//float speed_wish_right = (cmd_vel.angle*WHEEL_DIST)/2 + cmd_vel.speed;
	//float speed_wish_left = velocity_target * 2 - speed_wish_right;

	leftSpeedPidSetPointTmp = velocity_target * 2 - ((msg.angular.z * (wheelbase * 0.5)) / 2 + velocity_target);

	/*
	Serial.print(leftSpeedPidSetPointTmp);
	Serial.print("\t");
	*/

	leftSpeedPidSetPointTmp = (leftSpeedPidSetPointTmp * ppm) / SPEED_PID_SAMPLE_FREQ;

	setPidTarget();

}

ros::Subscriber<geometry_msgs::Twist> cmdVelSubscribe("cmd_vel", &twistMsgCb);
// eof:ROS

void servo_dir( const std_msgs::Int16 & cmd_msg){
  servo1.write(cmd_msg.data); // servo angle, range 0-180  
}

ros::Subscriber<std_msgs::Int16> sub_dir("pub_dir", servo_dir);

void motor_vel( const std_msgs::Int16 & cmd_msg){
  leftSpeedPidSetPointTmp = (cmd_msg.data * ppm) / SPEED_PID_SAMPLE_FREQ;
  setPidTarget(); 
}

ros::Subscriber<std_msgs::Int16> sub_vel("pub_vel", motor_vel);


IRAM_ATTR void encoderLeftCounterA() {
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_LEFT_PINA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_LEFT_PINB) == LOW) {
      encoderLeftPulses = encoderLeftPulses - 1;         // CW
      // pid
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    } else {
      encoderLeftPulses = encoderLeftPulses + 1;         // CCW
      // pid
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  } else {
    // its low-to-high-to-low on channel A
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_LEFT_PINB) == HIGH) {
      encoderLeftPulses = encoderLeftPulses - 1;          // CW
      // pid
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    } else {
      encoderLeftPulses = encoderLeftPulses + 1;          // CCW
      // pid
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  }
}

IRAM_ATTR void encoderLeftCounterB() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_LEFT_PINB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_LEFT_PINA) == HIGH) {
      encoderLeftPulses = encoderLeftPulses - 1;         // CW
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    }
    else {
      encoderLeftPulses = encoderLeftPulses + 1;         // CCW
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  }

  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_LEFT_PINA) == LOW) {
      encoderLeftPulses = encoderLeftPulses - 1;          // CW
      encoderLeftPulsesSpeedPID--;
      encoderLeftPulsesSteeringPID--;
    }
    else {
      encoderLeftPulses = encoderLeftPulses + 1;          // CCW
      encoderLeftPulsesSpeedPID++;
      encoderLeftPulsesSteeringPID++;
    }
  }
}

void motorGoFront(int speed) {
	if(motorState != 1) {
		digitalWrite(IN1, LOW);
		digitalWrite(IN2, HIGH);
		motorState = 1;
	}
	analogWrite(ENA, speed);
}

void motorGoBack(int speed) {
	if(motorState != -1) {
		digitalWrite(IN1, HIGH);
		digitalWrite(IN2, LOW);
		motorState = -1;
	}
	analogWrite(ENA, speed);
}

void motorStop() {
	if(motorState != 0) {
		analogWrite(ENA, 0);
		digitalWrite(IN1, LOW);
		digitalWrite(IN2, LOW);
		motorState = 0;
	}
}

void bodyMotorsControl() {
  // set motor left direction & speed
  if(leftMotorPwmOut == 0) {
	motorStop();
  } else if(leftMotorPwmOut > 0) {
    motorGoFront(leftMotorPwmOut);
  } else {
    motorGoBack(abs(leftMotorPwmOut));  
  }
}

boolean leftSpeedPidCompute() {
  boolean pidResult;
  leftSpeedPidInput = abs(encoderLeftPulsesSpeedPID);
  pidResult = leftSpeedPid.Compute(); 
  if(pidResult) {
    encoderLeftPulsesSpeedPID = 0;
    leftSpeedPidInputLast = leftSpeedPidInput;
	speed = (encoderLeftPulses - speedLastPulses) * mpp * SPEED_PID_SAMPLE_FREQ;
	speedLastPulses = encoderLeftPulses;
  }
  return pidResult;
}

void update_PID() {
  // calculate speedPid
  leftSpeedPidResult = leftSpeedPidCompute();

  if(leftSpeedPidSetPoint) {
	// show something to debug
	
	/*
	Serial.print(leftSpeedPidSetPoint);
	Serial.print("\t");
	Serial.print(leftSpeedPidInputLast);
	Serial.println("\n");
	*/
/*
	Serial.print(steering_angle);
	Serial.print("\t");
	Serial.print(velocity_target);
	Serial.print("\t");
	Serial.print(speedOs);
	Serial.print("\t");
	Serial.print(speed);
	Serial.print("\t");
	
	Serial.print(leftSpeedPidSetPoint);
	Serial.print("\t");
	Serial.print(leftSpeedPidInputLast);
	Serial.print("\t");
	
	Serial.print(encoderLeftPulses);
	Serial.print("\t");
	Serial.print(leftMotorPwmOut);
	Serial.print("\t");
	Serial.print(motorState);
	Serial.print("\t");
	Serial.println(servo1Angle);
*/	
  }
}

void lcdMessage(int cls, int line, String msg) {
  #ifdef ENABLE_OLED
    if(hasDisplay) {
        if(cls) display.clear();
        //display.drawString(128 / 2, 32 / 2, msg);
        display.drawString(0, line * 16, msg);
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

  // button config
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);

  // serial config
  Serial.begin(115200);
  Serial.println("Booting");

  #ifdef ENABLE_OLED
    hasDisplay = display.init();
    if(hasDisplay) {
        display.flipScreenVertically();
        display.setFont(ArialMT_Plain_16);
        //display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setContrast(255);
    }  
    lcdMessage(1, 0, "booting");
  #endif

  #ifdef ENABLE_GPS
    // serial link to GPS
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  #endif

	// motor settings	
	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);

  //ledcSetup(pwmChannel, freq, resolution);
	ledcSetup(0, 1000, 8);

	// stop motors
	analogWrite(ENA, 0);
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	motorState = 0;

	// encoder & odometry settings
	// meters per wheel rotation
	mpr = (2 * PI * wheelradius);
	// pulses per meter
	ppm = ppr / mpr;
	// meters per pulse
	mpp = 1.0 / ppm;

	Serial.print("wheelbase: ");
	Serial.println(wheelbase, 3);
	Serial.print("wheelradius: ");
	Serial.println(wheelradius, 3);

	Serial.print("pulses per rotation: ");
	Serial.println(ppr, 6);
	Serial.print("meters per rotation: ");
	Serial.println(mpr, 6);
	Serial.print("meters per pulse: ");
	Serial.println(mpp, 6);
	Serial.print("pulses per meter: ");
	Serial.println(ppm);

  // WIFI
  if(softap_mode) {
    const char *hostname = "akbesp";
    lcdMessage(0, 1, "starting softAP");
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
    lcdMessage(0, 1, String("join ") + ssid);
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
        if (error == OTA_AUTH_ERROR) lcdMessage(1, 0, "OTA error: Auth Failed");
        else if (error == OTA_BEGIN_ERROR) lcdMessage(1, 0, "OTA error: Begin Failed");
        else if (error == OTA_CONNECT_ERROR) lcdMessage(1, 0, "OTA error: Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) lcdMessage(1, 0, "OTA error: Receive Failed");
        else if (error == OTA_END_ERROR) lcdMessage(1, 0, "OTA error: End Failed");
        else lcdMessage(1, 0, "OTA error: other");
      #endif

    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(ip);

  String slocalip = ip.toString();
  lcdMessage(1, 0, slocalip);

	// try to build the apropriate
	// ros master server ip
	size_t found = slocalip.indexOf(".");
	slocalip = slocalip.substring(found+1);
	found = slocalip.indexOf(".");
	slocalip = slocalip.substring(found+1);
	found = slocalip.indexOf(".");
	slocalip = slocalip.substring(0, found);
	IPAddress server(192,168,slocalip.toInt(),serverIp);
	Serial.print("ROS master IP: ");
	Serial.println(server);
  lcdMessage(0, 1, server.toString());

	// servo
	servo1.attach(servo1Pin);
	servo1.write(90 - 0);
	delay(50);

	// encoders
	encoderLeftDirection = false; 
	pinMode(ENCODER_LEFT_PINA,INPUT);
	pinMode(ENCODER_LEFT_PINB,INPUT);
	attachInterrupt(ENCODER_LEFT_PINA, ENCODER_LEFT_FUNCTIONA, ENCODER_LEFT_SIGNAL);
	attachInterrupt(ENCODER_LEFT_PINB, ENCODER_LEFT_FUNCTIONB, ENCODER_LEFT_SIGNAL);

	// bof:ROS
	// Set the connection to rosserial socket server
	nh.getHardware()->setConnection(server, serverPort);
	nh.initNode();

	// Another way to get IP
	Serial.print("ROS IP = ");
	Serial.println(nh.getHardware()->getLocalIP());

	nh.subscribe(cmdVelSubscribe);
	nh.advertise(encoderPublisher);
	//nh.advertise(ipPublisher);
  nh.subscribe(sub_dir);
  nh.subscribe(sub_vel);

	//ipPub();
	// eof:ROS

	// left speed PID
	//leftSpeedPidSetPoint = SPEED_PID_TARGET;
	leftSpeedPidSetPoint = 0;
	//leftSpeedPid.SetMode(AUTOMATIC);
	leftSpeedPid.SetMode(MANUAL);
	leftSpeedPid.SetSampleTime(SPEED_PID_SAMPLE_TIME); 
	leftSpeedPid.SetOutputLimits(LEFT_SPEED_PID_MIN_OUTPUT, LEFT_SPEED_PID_MAX_OUTPUT);
 
}

void loop() {
  ArduinoOTA.handle();

	servo1Angle = map(steering_angle*100, -156, 156, 50, 130);  // scale it to use it with the servo (value between 0 and 180)
	
	//servo1Angle -= 0; // go straight compensation

	if(servo1Angle < 60) servo1Angle = 60;
	if(servo1Angle > 120) servo1Angle = 120;

	if(twist_linear == 0 || twist_linear > 0.02) {
		servo1.write(servo1Angle);
	}

	static double encoderLeftPulsesLast = 999;
	
	if(encoderLeftPulses != encoderLeftPulsesLast) {
		if(encoderLeftPulses != encoderLeftPulsesLast) encoderLeftPulsesLast = encoderLeftPulses;
	}

	// check encoder targets
	if(encoderPulsesTargetEnabled) {
		//
		// check left encoder target
		if(!encoderLeftPulsesOnTarget) {
		if(leftSpeedPidSetPointDirection >= 0) {
			//Serial.print("LP>: "); Serial.println(encoderLeftPulses);
			if(encoderLeftPulses >= encoderLeftPulsesTarget - encoderLeftPulsesTargetStopOffset) {
			encoderLeftPulsesOnTarget = true;  
			}
		} else {
			//Serial.print("LP<: "); Serial.println(encoderLeftPulses);
			if(encoderLeftPulses < encoderLeftPulsesTarget + encoderLeftPulsesTargetStopOffset) {
			encoderLeftPulsesOnTarget = true;
			}      
		}
		// left stop on encoder target
		if(encoderLeftPulsesOnTarget) {
			//Serial.println("L ON TARGET");
			leftMotorPwmOut = 0;
			leftSpeedPidSetPoint = 0;
			leftSpeedPidSetPointDirection = 0; 
		}
		}
		
		//
		// encoders on target
		if(encoderLeftPulsesOnTarget) {
		//Serial.println("BOTH ON TARGET");

		encoderLeftPulsesTargetStart = 0;
		encoderLeftPulsesTarget = 0;
		encoderLeftPulsesOnTarget = false;
		encoderPulsesTargetEnabled = false;
		}
	}
	
  	// set speed  
	// if(setPwmStatus) {    
	// 	leftMotorPwmOut = leftMotorPwmOutCmd;
	//  setPwmStatus = false;
	// } else {
    leftMotorPwmOut = 0;
    if(leftSpeedPidSetPoint != 0) {
      leftMotorPwmOut = leftSpeedPidOutput * leftSpeedPidSetPointDirection; // - steeringPidOutput;
    }
	//
	//  }

  	update_PID();
  	bodyMotorsControl();

	// speed calculation m/s, on second
	if(millis() >= secondTimer) {
		secondTimer = millis() + 1000;
		speedOs = (encoderLeftPulses - speedOsLastPulses) * mpp;
		speedOsLastPulses = encoderLeftPulses;
	}

	if(millis() >= loopInfoTimer) {
		loopInfoTimer = millis() + 100;
		
		Serial.print(twist_angular);
		Serial.print("\t");
		Serial.print(steering_angle);
		Serial.print("\t");
		Serial.print(servo1Angle);
		Serial.print(" | \t");
		Serial.print(twist_linear);
		Serial.print("\t");
		Serial.print(velocity_target);
		Serial.print(" | \t");
		Serial.print(speedOs);
		Serial.print("\t");
		Serial.print(speed);
		Serial.print("\t");
		Serial.print(leftSpeedPidSetPoint, 0);
		Serial.print("\t");
		Serial.print(leftSpeedPidInputLast);
		Serial.print("\t");
		Serial.print(encoderLeftPulses);
		Serial.print("\t");
		Serial.print(leftMotorPwmOut);
		Serial.print("\t");
		Serial.println(motorState);
		
	}

  #ifdef ENABLE_GPS
    while (Serial2.available() > 0)
      if (gps.encode(Serial2.read())) {
        Serial.print(F("Location: ")); 
        if (gps.location.isValid()) {
          Serial.print(gps.location.lat(), 6);
          Serial.print(F(","));
          Serial.print(gps.location.lng(), 6);
        } else {
          Serial.print(F("INVALID"));
        }        
      }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
  #endif

	// publish
	encoderPub();
	//ipPub();

  if ( digitalRead(BUTTON1_PIN) == HIGH ) { 
    Serial.println("button1_state = HIGH");
  }
  if ( digitalRead(BUTTON2_PIN) == HIGH ) { 
    Serial.println("button2_state = HIGH");
  }
  if ( digitalRead(BUTTON3_PIN) == HIGH ) { 
    Serial.println("button3_state = HIGH");
  }


  blinkLedBuiltin();

  nh.spinOnce();
}