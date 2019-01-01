/*
 * Title: WiFi Reflowduino
 * Author: Terence Ang
 * Website: www.terenceang.com
 * Last modified: 30/12/2018
 *
 * -----------------------------------------------------------------------------------------------
 * Credits: Original Code from Timothy Woo
 * Website: www.botletics.com
 * Special thanks to all those who have been an invaluable part of the DIY community,
 * like the author of the Arduino PID library and the developers at Adafruit!
 * Thanks to Timothy Woo
 *
 * -----------------------------------------------------------------------------------------------
 * License: This code is released under the GNU General Public License v3.0
 * https://choosealicense.com/licenses/gpl-3.0/ and appropriate attribution must be
 * included in all redistributions of this code.
 *
 * -----------------------------------------------------------------------------------------------
 * Disclaimer: Dealing with mains voltages is dangerous and potentially life-threatening!
 * If you do not have adequate experience working with high voltages, please consult someone
 * with experience or avoid this project altogether. We shall not be liable for any damage that
 * might occur involving the use of the Reflowduino and all actions are taken at your own risk.
 */

#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <FS.h>
#include <Adafruit_MAX31865.h> // PT100 RTD Temprature Sensor
#include <PID_v1.h> // Library for PID control
#include "pitches.h" // Includes the different notes for the buzzer
#include "states.h"

#define DEBUG
 // Define pins
#define buzzer D1
#define relay D2
#define LED D3 // This LED is used to indicate if the reflow process is underway
#define MAX_CS D4 // MAX31855 chip select pin
#define START_SW D8


// Define reflow temperature profile parameters (in *C)
// If needed, define a subtraction constant to compensate for overshoot:
#define T_const 5 // From testing, overshoot was about 5-6*C

// SN63/Pb37 Profile
#define T_preheat 125
#define T_soak 183
#define T_reflow 215 - T_const

// Standard lead-free solder paste (melting point around 215*C)
//#define T_preheat 150
//#define T_soak 217
//#define T_reflow 249 - T_const

// "Low-temp" lead-free solder paste (melting point around 138*C)
//#define T_preheat 90
//#define T_soak 138
//#define T_reflow 165 - T_const

// Test values to make sure your Reflowduino is actually working
//#define T_preheat 50
//#define T_soak 80
//#define T_reflow 100 - T_const

#define T_cool 100 // Safe temperature at which the board is "ready" (dinner bell sounds!)
#define preheat_rate 2 // Increase of 1-3 *C/s
#define soak_rate 0.7 // Increase of 0.5-1 *C/s
#define reflow_rate 2 // Increase of 1-3 *C/s
#define cool_rate -4 // Decrease of < 6 *C/s max to prevent thermal shock. Negative sign indicates decrease

// Define PID parameters. The gains depend on your particular setup
// but these values should be good enough to get you started
#define PID_sampleTime 1000 // 1000ms = 1s
// Preheat phase
#define Kp_preheat 150
#define Ki_preheat 0
#define Kd_preheat 100
// Soak phase
#define Kp_soak 200
#define Ki_soak 0.05
#define Kd_soak 300
// Reflow phase
#define Kp_reflow 300
#define Ki_reflow 0.05
#define Kd_reflow 350

double temperature, output, setPoint; // Input, output, set point
PID myPID(&temperature, &output, &setPoint, Kp_preheat, Ki_preheat, Kd_preheat, DIRECT);
double max_temp = 0;

reflow_state status = idle;
reflow_state previous_status;

bool cleararray = true;
bool key_beep = false;

// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 max = Adafruit_MAX31865(D4, D7, D6, D5);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 PT_Sensor = Adafruit_MAX31865(D4);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


double T_start; // Starting temperature before reflow process
int windowSize = 2000;
unsigned long sendRate = 2000; // Send data to app every 2s
unsigned long t_start = 0; // For keeping time during reflow process
unsigned long previousMillis = 0;
unsigned long duration, t_final, windowStartTime, timer;
unsigned long preheat_duration, soak_duration, reflow_duration, cool_duration;

#ifdef DEBUG
#define Report_delay 2000
unsigned long debug_previous = 0;

#endif // DEBUG

//Wifi setup
const char* ssid = "DHIOTS";
const char* password = "IOTConnect456";
ESP8266WebServer server(80);   //instantiate server at port 80 (http port)

String page = "";
String text = "";
double data;

StaticJsonBuffer<200> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
char JSONmessageBuffer[300];

StaticJsonBuffer<10000> JsonchartBuffer;
JsonObject& chart = JsonchartBuffer.createObject();
char chartMessageBuffer[3000];

JsonArray& chartTime = chart.createNestedArray("time");
JsonArray& chartTemp = chart.createNestedArray("temp");


void setup() {
	Serial.begin(115200); // This should be different from the Bluetooth baud rate
	while (!Serial) continue;

	WiFi.begin(ssid, password); //begin WiFi connection
	Serial.println("");

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.print("Connected to ");
	Serial.println(ssid);
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	// Port defaults to 8266
	// ArduinoOTA.setPort(8266);

	// Hostname defaults to esp8266-[ChipID]
	// ArduinoOTA.setHostname("myesp8266");

	// No authentication by default
	// ArduinoOTA.setPassword((const char *)"123");

	ArduinoOTA.onStart([]() {
		Serial.println("Start");
	});

	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});

	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});

	ArduinoOTA.begin();

	if (!SPIFFS.begin())
	{
		// Serious problem
		Serial.println("SPIFFS Mount failed");
	}
	else {
		Serial.println("SPIFFS Mount succesfull");
	}

	server.serveStatic("/", SPIFFS, "/index.html");

	server.on("/chart.json", []() {
		chart.printTo(chartMessageBuffer, sizeof(chartMessageBuffer));
		Serial.println(chartMessageBuffer);
		server.send(200, "application/json", chartMessageBuffer);
	});

	server.on("/data.json", []() {
		root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
		//Serial.println(JSONmessageBuffer);
		server.send(200, "application/json", JSONmessageBuffer);
	});

	server.begin();
	Serial.println("Web server started!");

#ifdef buzzer
	pinMode(buzzer, OUTPUT);
	tone(buzzer, NOTE_G3, 100);
	delay(100);
	noTone(buzzer);
#endif // buzzer

	//playTune(doneDealMelody); // Play the buzzer melody

	pinMode(LED, OUTPUT);
	pinMode(relay, OUTPUT);

	digitalWrite(LED, LOW);
	digitalWrite(relay, LOW); // Set default relay state to OFF

	attachInterrupt(START_SW, toggle_sw, FALLING);

	PT_Sensor.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

	myPID.SetOutputLimits(0, windowSize);
	myPID.SetSampleTime(PID_sampleTime);
	myPID.SetMode(AUTOMATIC); // Turn on PID control

  //  while (!Serial) delay(1); // OPTIONAL: Wait for serial to connect
	Serial.println("*****Wifi Reflowduino*****");
}

void loop() {
	ArduinoOTA.handle();

	/***************************** MEASURE TEMPERATURE *****************************/
	temperature = PT_Sensor.temperature(RNOMINAL, RREF);
	if (temperature > max_temp) max_temp = temperature;

	root["temp"] = temperature;
	root["max_temp"] = max_temp;
	root["phase"] = states_name[status];
	root["time"] = duration / 60;



	// collect chgart data only when inoperation
	if (status > idle &&  status < ended) {
		if (cleararray) {
			cleararray = false;
			JsonchartBuffer.clear();
			//Serial.println("clear array");
			JsonObject& chart = JsonchartBuffer.createObject();
			JsonArray& chartTime = chart.createNestedArray("time");
			JsonArray& chartTemp = chart.createNestedArray("temp");
		}
		else {
			chartTime.add(duration);
			chartTemp.add(temperature);
			chart.printTo(Serial);
		}
	}




	uint8_t fault = PT_Sensor.readFault();

	if (fault) status = rtd_fault;

	// Process
	switch (status)
	{
	case idle:
		digitalWrite(LED, LOW); // Red LED indicates reflow is underway
		setPoint = 0;
		break;

	case preheat:
		digitalWrite(LED, HIGH); // Red LED indicates reflow is underway

		if (temperature >= T_preheat) { // Check if the current phase was just completed
			status = soak;
			duration = millis() - t_start; // reset duration for the next phase
			preheat_duration = duration; // record preheat duration
			t_start = millis(); // Reset timer for next phase
			myPID.SetTunings(Kp_soak, Ki_soak, Kd_soak); // set tuning for next phase
			Serial.print("Preheat phase complete : ");
			Serial.println(preheat_duration);
			phase_beep();
		}
		else {
			// Calculate the projected final time based on temperature points and temperature rates
			t_final = (T_preheat - T_start) / preheat_rate + t_start;
			// Calculate desired temperature at that instant in time using linear interpolation
			setPoint = duration * (T_preheat - T_start) / (t_final - t_start);
		}

		break;

	case soak:
		digitalWrite(LED, HIGH); // Red LED indicates reflow is underway
		if (temperature >= T_soak) {
			status = reflow;
			duration = millis() - t_start; // reset duration for the next phase
			soak_duration = duration; // record preheat duration
			t_start = millis(); // Reset timer for next phase
			myPID.SetTunings(Kp_reflow, Ki_reflow, Kd_reflow); // set tuning for next phase
			Serial.println("Soaking phase complete : ");
			Serial.println(preheat_duration);
			phase_beep();
		}
		else {
			t_final = (T_soak - T_start) / soak_rate + t_start;
			setPoint = duration * (T_soak - T_start) / (t_final - t_start);
		}
		break;

	case reflow:
		digitalWrite(LED, HIGH); // Red LED indicates reflow is underway
		if (temperature >= T_reflow) {
			status = cool;
			duration = millis() - t_start; // reset duration for the next phase
			reflow_duration = duration; // record preheat duration
			t_start = millis(); // Reset timer for next phase
			myPID.SetTunings(Kp_preheat, Ki_preheat, Kd_preheat); // set tuning for next phase
			Serial.print("Reflow phase complete : ");
			Serial.println(reflow_duration);
			Serial.println("Open the door! ");
			Door_Beep();
		}
		else {
			t_final = (T_reflow - T_start) / reflow_rate + t_start;
			setPoint = duration * (T_reflow - T_start) / (t_final - t_start);
		}

		break;

	case cool:
		digitalWrite(LED, HIGH); // Red LED indicates reflow is underway

		if (temperature <= T_cool) {
			status = ended;
			cool_duration = millis() - t_start; // reset duration for the next phase
			t_start = millis(); // Reset timer for next phase
			Serial.println("Cooling complete : ");
			Serial.println(cool_duration);
			Done_Beep(); // Play the buzzer melody
		}
		else {
			digitalWrite(relay, LOW); //cooling - just turn off the heaters.
			setPoint = 0;
		}
		break;

	case ended:
		if ((millis() - t_start) > 5000) {
			status = idle;
		}
		break;

	case rtd_fault:
		Serial.println("Invalid reading, check RTD!");
		Serial.print("Fault 0x"); Serial.println(fault, HEX);
		if (fault & MAX31865_FAULT_HIGHTHRESH) {
			Serial.println("RTD High Threshold");
		}
		if (fault & MAX31865_FAULT_LOWTHRESH) {
			Serial.println("RTD Low Threshold");
		}
		if (fault & MAX31865_FAULT_REFINLOW) {
			Serial.println("REFIN- > 0.85 x Bias");
		}
		if (fault & MAX31865_FAULT_REFINHIGH) {
			Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
		}
		if (fault & MAX31865_FAULT_RTDINLOW) {
			Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
		}
		if (fault & MAX31865_FAULT_OVUV) {
			Serial.println("Under/Over voltage");
		}

		tone(buzzer, NOTE_C3, 2000);
		delay(2000);
		noTone(buzzer);

		PT_Sensor.clearFault();
		Serial.println();
		break;
	}

	duration = millis() - t_start; // reset duration for the next phase

	// Compute PID output (from 0 to windowSize) and control relay accordingly
	myPID.Compute(); // This will only be evaluated at the PID sampling rate

	if (millis() - windowStartTime >= windowSize) windowStartTime += windowSize; // Shift the time window

	if (output > millis() - windowStartTime) digitalWrite(relay, HIGH); // If HIGH turns on the relay
	else digitalWrite(relay, LOW);

#ifdef DEBUG
	if (millis() > debug_previous + Report_delay) {
		debug_previous = millis();
		//uint16_t rtd = PT_Sensor.readRTD();
		//float ratio = rtd;
		//ratio /= 32768;

		//Serial.print("RTD value: "); Serial.println(rtd);
		//Serial.print("Ratio = "); Serial.println(ratio, 8);
		//Serial.print("Resistance = "); Serial.println(RREF*ratio, 8);

		//Serial.println();
		Serial.print("Temperature = "); Serial.println(temperature);
		Serial.print("Max Temperature = "); Serial.println(max_temp);
		Serial.print("Setpoint = "); Serial.println(setPoint);
		Serial.print("Duration = "); Serial.println(duration / 1000);
		Serial.print("PID Output = "); Serial.println(output);

		Serial.print("Reflow Status is "); Serial.println(states_name[status]);
		Serial.println(states_name[status]);
		Serial.println();
	}

	server.handleClient();

#endif // DEBUG

	if (key_beep) {
		tone(buzzer, NOTE_C3, 100);
		delay(100);
		noTone(buzzer);
		key_beep = false;
	}
}

///Beeps

void Door_Beep() {

	// This melody plays at the very end when it's safe to take your PCB's!
	int Melody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };

	// Note durations: 4 = quarter note, 8 = eighth note, etc.
	int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

	// Iterate over the notes of the melody:
	for (int thisNote = 0; thisNote < 8; thisNote++) {
		// To calculate the note duration, take one second divided by the note type
		// e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(buzzer, Melody[thisNote], noteDuration);

		// To distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		// stop the tone playing:
		noTone(buzzer);
	}
}

void Done_Beep() {

	// This melody plays at the very end when it's safe to take your PCB's!
	int Melody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };

	// Note durations: 4 = quarter note, 8 = eighth note, etc.
	int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

	// Iterate over the notes of the melody:
	for (int thisNote = 0; thisNote < 8; thisNote++) {
		// To calculate the note duration, take one second divided by the note type
		// e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(buzzer, Melody[thisNote], noteDuration);

		// To distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		// stop the tone playing:
		noTone(buzzer);
	}
}

void phase_beep() {
	tone(buzzer, NOTE_C3, 100);
	delay(100);
	noTone(buzzer);
}


void toggle_sw() {
	key_beep = true;

	if (status == idle) {

		//Serial.print("Starting temperature: ");
		//Serial.print(T_start);
		//Serial.println(" *C");
		//status = ended;
		//t_start = millis(); // Reset timer for next phase
		cleararray = true;
		status = preheat;
		t_start = millis(); // Begin timers
		windowStartTime = millis();
		T_start = temperature;
		//duration = millis() - t_start; //set duration for preheat
		Serial.print("Starting temperature: ");
		Serial.print(T_start);
		Serial.println(" *C");
	}
	else status = idle;
}