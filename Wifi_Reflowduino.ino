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

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <Adafruit_MAX31865.h> // PT100 RTD Temprature Sensor
#include <PID_v1.h> // Library for PID control
#include "pitches.h" // Includes the different notes for the buzzer

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

// Test Profile 100%
#define T_preheat 50
#define T_soak 80
#define T_reflow 100 - T_const

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

#define T_cool 40 // Safe temperature at which the board is "ready" (dinner bell sounds!)
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

ESP8266WiFiMulti wifiMulti;     // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'

ESP8266WebServer server(80);    // Create a webserver object that listens for HTTP request on port 80
File fsUploadFile;                                    // a File variable to temporarily store the received file

const char* mdnsName = "reflowduino";        // Domain name for the mDNS responder

void handleRoot();              // function prototypes for HTTP handlers
void handleNotFound();

// Buzzer settings
// This melody plays when the reflow temperature is reached,
// at which point you should open the door (for toaster ovens)
//int openDoorTune[] = {
//  NOTE_G6 // I found that NOTE_G6 catches my attention pretty well
//};

// This melody plays at the very end when it's safe to take your PCB's!
int doneDealMelody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };

// Note durations: 4 = quarter note, 8 = eighth note, etc.
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

const char* status = "idle";

// Logic flags
bool justStarted = true;
bool reflow = false; // Baking process is underway!
bool preheatComplete = false;
bool soakComplete = false;
bool reflowComplete = false;
bool coolComplete = false;
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

void toggle_sw() {
	key_beep = true;
	reflow = !reflow;
	if (!reflow) {
		status = "idle";
		// Logic flags reset
		justStarted = true;
		reflow = false; // Baking process is underway!
		preheatComplete = false;
		soakComplete = false;
		reflowComplete = false;
		coolComplete = false;
	}
}

double T_start; // Starting temperature before reflow process
int windowSize = 2000;
unsigned long sendRate = 2000; // Send data to app every 2s
unsigned long t_start = 0; // For keeping time during reflow process
unsigned long previousMillis = 0;
unsigned long duration, t_final, windowStartTime, timer;

#ifdef DEBUG
#define Report_delay 2000
unsigned long debug_previous = 0;

#endif // DEBUG


void setup() {
	Serial.begin(115200); // This should be different from the Bluetooth baud rate

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
	
	wifiMulti.addAP("DHIOTS", "IOTConnect456");   // add Wi-Fi networks you want to connect to
	//wifiMulti.addAP("ssid_from_AP_2", "your_password_for_AP_2");
	//wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");

	Serial.println("Connecting ...");
	int i = 0;
	while (wifiMulti.run() != WL_CONNECTED) { // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
		delay(250);
		Serial.print('.');
	}
	Serial.println('\n');
	Serial.print("Connected to ");
	Serial.println(WiFi.SSID());              // Tell us what network we're connected to
	Serial.print("IP address:\t");
	Serial.println(WiFi.localIP());           // Send the IP address of the ESP8266 to the computer

	if (MDNS.begin("esp8266")) {              // Start the mDNS responder for esp8266.local
		Serial.println("mDNS responder started");
	}
	else {
		Serial.println("Error setting up MDNS responder!");
	}

	server.on("/", handleRoot);               // Call the 'handleRoot' function when a client requests URI "/"
	server.onNotFound(handleNotFound);        // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"

	server.begin();                           // Actually start the server
	Serial.println("HTTP server started");
}

void loop() {
	/***************************** MEASURE TEMPERATURE *****************************/
	temperature = PT_Sensor.temperature(RNOMINAL, RREF);
	uint8_t fault = PT_Sensor.readFault();

	/***************************** REFLOW PROCESS CODE *****************************/
	if (reflow) {
		digitalWrite(LED, HIGH); // Red LED indicates reflow is underway

		// This only runs when you first start the reflow process
		if (justStarted) {
			justStarted = false;

			t_start = millis(); // Begin timers
			windowStartTime = millis();
			T_start = temperature;

			if (fault) {
				Serial.println("Invalid reading, check RTD!");
				status = "fault";
			}
			else {
				status = "Preheat";
				Serial.print("Starting temperature: ");
				Serial.print(T_start);
				Serial.println(" *C");
			}
		}

		// Determine the amount of time that elapsed in any particular phase (preheat, soak, etc)
		duration = millis() - t_start;

		// Determine the desired set point according to where are in the reflow process
		// Perform a linear extrapolation of what desired temperature we want to be at.
		/********************* PREHEAT *********************/
		if (!preheatComplete) {
			if (temperature >= T_preheat) { // Check if the current phase was just completed
				status = "Soaking";
				preheatComplete = true;
				t_start = millis(); // Reset timer for next phase
				Serial.println("Preheat phase complete!");
				tone(buzzer, NOTE_C3, 100);
				delay(100);
				noTone(buzzer);
			}
			else {
				// Calculate the projected final time based on temperature points and temperature rates
				t_final = (T_preheat - T_start) / preheat_rate + t_start;
				// Calculate desired temperature at that instant in time using linear interpolation
				setPoint = duration * (T_preheat - T_start) / (t_final - t_start);
			}
		}
		/********************* SOAK *********************/
		else if (!soakComplete) {
			if (temperature >= T_soak) {
				status = "reflow";
				soakComplete = true;
				t_start = millis();
				Serial.println("Soaking phase complete!");
				tone(buzzer, NOTE_C3, 100);
				delay(100);
				noTone(buzzer);
			}
			else {
				t_final = (T_soak - T_start) / soak_rate + t_start;
				setPoint = duration * (T_soak - T_start) / (t_final - t_start);
			}
		}
		/********************* REFLOW *********************/
		else if (!reflowComplete) {
			if (temperature >= T_reflow) {
				status = "Cooling";
				reflowComplete = true;
#ifdef buzzer
				tone(buzzer, NOTE_G3, 2000); // Alert the user to open the door!
				delay(2000);
				noTone(buzzer);
#endif
				t_start = millis();
				Serial.println("Reflow phase complete!");
			}
			else {
				t_final = (T_reflow - T_start) / reflow_rate + t_start;
				setPoint = duration * (T_reflow - T_start) / (t_final - t_start);
			}
		}
		/********************* COOLDOWN *********************/
		else if (!coolComplete) {
			if (temperature <= T_cool) {
				status = "idle";
				coolComplete = true;
				reflow = false;
				Serial.println("PCB reflow complete!");
#ifdef buzzer
				playTune(doneDealMelody); // Play the buzzer melody
#endif
			}
			else {
				t_final = (T_cool - T_start) / cool_rate + t_start;
				setPoint = duration * (T_cool - T_start) / (t_final - t_start);
			}
		}

		// Use the appropriate PID parameters based on the current phase
		if (!soakComplete) myPID.SetTunings(Kp_soak, Ki_soak, Kd_soak);
		else if (!reflowComplete) myPID.SetTunings(Kp_reflow, Ki_reflow, Kd_reflow);

		// Compute PID output (from 0 to windowSize) and control relay accordingly
		myPID.Compute(); // This will only be evaluated at the PID sampling rate
		if (millis() - windowStartTime >= windowSize) windowStartTime += windowSize; // Shift the time window
		if (output > millis() - windowStartTime) digitalWrite(relay, HIGH); // If HIGH turns on the relay
	//    if (output < millis() - windowStartTime) digitalWrite(relay, HIGH); // If LOW turns on the relay
		else digitalWrite(relay, LOW);
	}
	else {
		digitalWrite(LED, LOW);
		digitalWrite(relay, LOW);
	}

	server.handleClient();                    // Listen for HTTP requests from clients

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
		Serial.print("PID Output = "); Serial.println(output);

		Serial.print("Reflow switch is ");

		if (reflow) Serial.println("ON");
		else Serial.println("OFF");

		Serial.print("Reflow Status is ");
		Serial.println(status);
		Serial.println();
	}


	// Check and print any faults
	if (fault) {
		reflow = 0; //shutdon on fault.
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
	}

#endif // DEBUG

	if (key_beep) {
		tone(buzzer, NOTE_C3, 100);
		delay(100);
		noTone(buzzer);
		key_beep = false;
	}
}

// This function plays the melody for the buzzer.
// Make this as simple or as elaborate as you wish!
#ifdef buzzer

void playTune(int *melody) {
	// Iterate over the notes of the melody:
	for (int thisNote = 0; thisNote < 8; thisNote++) {
		// To calculate the note duration, take one second divided by the note type
		// e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000 / noteDurations[thisNote];
		tone(buzzer, melody[thisNote], noteDuration);

		// To distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		// stop the tone playing:
		noTone(buzzer);
	}
}
#endif // buzzer

void handleRoot() {
	server.send(200, "text/plain", "Hello world!");   // Send HTTP status 200 (Ok) and send some text to the browser/client
}

void handleNotFound() {
	server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}