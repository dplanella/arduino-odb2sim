#include <SoftwareSerial.h>

// Arduino ODBII scan tool simulator. Transmits ODBII PIDs with vehicle sensor
// information upon request, using the ELM327 protocol.
//
// The common setup will be a set of automotive sensors connected to the
// Arduino microcontroller (MCU) running as the simulator. Equipped with a
// Bluetooth module, sensor information will be sent to a client when
// requested. The client will generally be a smart phone running an OBDII app,
// such as Torque.
//
// This program implements the aqcuisition and calculation of sensor values,
// and transmission of those to the client using OBDII PID structures¹ over the
// ELM327 protocol.
//
//         +-----+---+     +--------+        Xx     +----+
//         |     |   |     |  +--+  |     Xx  XX    |----|
//         |    +++  |     | -+  +- |  Xx  XX  XX   ||  ||
//         | -> | |  +---> | -+  +- |   XX  X   X   ||  ||
//         |    +++  |     |  +--+  |  Xx  XX  XX   ||  ||
//         |     |   |     |        |     Xx  XX    |----|
//         +-----+---+     +--------+        Xx     +----+
//
//           Sensor      MCU + Bluetooth          Smartphone
//                      (OBDII scan tool           or tablet
//                         simulator)
//
// ¹ https://en.wikipedia.org/wiki/OBD-II_PIDs
//
//
// Copyright (C) 2017 David Planella <david.planella@ubuntu.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// Pin assignments
const int rpmPin = 2;
const int rxPin = 4;
const int txPin = 5;

// We use the software serial class so that we can use the
// hardware UART for debugging purposes and to print the output
// to a serial monitor
SoftwareSerial BtSerial(rxPin, txPin);

// RPM calculation variables
volatile byte rpm_pulse_count;
unsigned int rpm;
unsigned long time_old;

// Variables to track Mode 1 PIDs
int eng_rpm = 5400;
int ambient_air_temp;
int eng_oil_temp;
float eng_coolant_temp;
float vehicle_speed;
float in_manifld_press_abs;
float fuel_percentage;

// Variables to track Mode 21 (custom) PIDs
float vehicle_speed_2;
float analog_in_5;
int eng_oil_press;

// Responses
//
// Generally, ELM327 clones return the following as firmware version:
// "ELM327/ELM-USB v1.0 (c) SECONS Ltd."
// Valid ELM327 firmware versions are: 1.0, 1.3a, 1.4b, 2.1 and 2.2
// We will be using our own firmware version and device description
const String DEVICE_DESCRIPTION = "OBDII Arduino Simulator";
const String VERSION = DEVICE_DESCRIPTION + " " + "v1.0";
const String OK = "OK";
const String PROMPT = ">";

// Control characters, whitespace
const char CR = '\r';
const char LF = '\n';
const char SPACE = ' ';

// Request prefixes
const String AT_REQ = "AT";      // AT command request: handshake/control
const String MODE_01_REQ = "01"; // OBDII Mode 1 request: data (PID values)
const String MODE_21_REQ = "21"; // OBDII Mode 21 request: custom PID values

// Response prefixes
const String MODE_01_RSP = "41";
const String MODE_21_RSP = "61";

// OBDII Mode 1 PIDs
const String MODE_1_SUPPORTED_PIDS = "00";
const String MODE_1_ENG_COOLANT_TEMP = "05";
const String MODE_1_IN_MANIFLD_PRESS_ABS = "0B";
const String MODE_1_ENG_RPM = "0C";
const String MODE_1_VEHICLE_SPEED = "0D";
const String MODE_1_FUEL_TANK_LVL_IN = "2F";
const String MODE_1_AMBIENT_AIR_TEMP = "46";
const String MODE_1_ENG_OIL_TEMP = "5C";

// Serial debug switch. Set it to true to echo the serial communication to an
// external serial monitor.
const bool SERIAL_DEBUG = false;

void setup() {

  const int BAUDRATE = 9600;
  const int DELAY = 600;

  initSensors();

  // Set up the hardware and software serial objects
  if (SERIAL_DEBUG) {
    Serial.begin(BAUDRATE);
  }
  delay(DELAY);
  BtSerial.begin(BAUDRATE);
}

void loop() {

  // Buffer to hold the OBDII request from the serial port and further process
  // it
  static String OBDRequest = "";

  updateSensorValues();

  // Read characters from the serial port when available, one
  // per loop iteration.
  if (BtSerial.available()) {

    // Read one received character at a time
    char c = BtSerial.read();

    if ((c == LF || c == CR) && OBDRequest.length() > 0) {
      // Once a full command is received, process it and clear the command
      // buffer afterwards to start receiving new requests
      OBDRequest.toUpperCase();
      processRequest(OBDRequest);
      OBDRequest = "";
    } else if (c != SPACE && c != LF && c != CR) {
      // If the full command is not yet there, read a new character
      // Ignore whitespace and control characters
      OBDRequest += c;
    }
  }

}

// Process the requests (commands) sent from the client app
// The requests can either be:
// - AT commands to set up the simulator and perform handshaking
// - ODBII PID requests to transmit sensor information
void processRequest(String request) {

  String pid;
  byte reply_bytes;
  String ATCommand;
  double value;
  // Value after having applied the calculation formula defined by the ODBII
  // PID structure definition:
  // https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_01
  double after_formula;
  const byte TEMP_OFFSET = 40;
  const String mode_id_rsp = MODE_01_RSP;

  if (request.startsWith(MODE_01_REQ)) {
    // Mode 1 request: show current data

    pid = request.substring(2); // Get the PID request

    if (SERIAL_DEBUG) {
      Serial.println(MODE_01_REQ + pid);
    }

    if (pid == MODE_1_SUPPORTED_PIDS) {
      // List of PIDs supported (range 01 to 32)
      // Bit encoded [A7..D0] == [PID 0x01..PID 0x20]
      // https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00
      reply_bytes = 4;
      value = 142249984; // Hex: "087A9000"
      after_formula = value;

    } else if (pid == MODE_1_ENG_COOLANT_TEMP) {
      // Temperature of the engine coolant in °C
      reply_bytes = 1;
      value = eng_coolant_temp;
      after_formula = value + TEMP_OFFSET;

    } else if (pid == MODE_1_ENG_RPM) {
      // Engine speed in rpm
      reply_bytes = 2;
      value = eng_rpm;
      after_formula = value * 4;

    } else if (pid == MODE_1_VEHICLE_SPEED) {
      // Vehicle speed in km/h
      reply_bytes = 1;
      value = vehicle_speed;
      after_formula = value;

    } else if (pid == MODE_1_IN_MANIFLD_PRESS_ABS) {
      // Intake manifold absolute pressure in kPa
      reply_bytes = 1;
      value = in_manifld_press_abs;
      after_formula = value;

    } else if (pid == MODE_1_FUEL_TANK_LVL_IN) {
      // Fuel level in %
      reply_bytes = 1;
      value = fuel_percentage;
      after_formula = (int)(((float) value / 100.0) * 255.0);

    } else if (pid == MODE_1_AMBIENT_AIR_TEMP) {
      // Evaporation purge in %
      reply_bytes = 1;
      value = ambient_air_temp;
      after_formula = value + TEMP_OFFSET;

    } else if (pid == MODE_1_ENG_OIL_TEMP) {
      // Engine oil temperature in °C
      reply_bytes = 1;
      value = eng_oil_temp;
      after_formula = value + TEMP_OFFSET;
    } else {
      // Unhandled PID
      reply_bytes = 1;
      value = 0;
      after_formula = value;
    }

    mode_id_rsp = MODE_01_RSP;
    replyOBD2(mode_id_rsp, pid, after_formula, reply_bytes);

  } else if (request.startsWith(MODE_21_REQ)) {
    // Mode 21 request: used to display custom values

    pid = request.substring(2);

    if (SERIAL_DEBUG) {
      Serial.println(MODE_21_REQ + pid);
    }

    // These are just example custom PIDs. You can define your own instead
    if (pid == "13") {
      reply_bytes = 2;
      value = vehicle_speed_2;
      after_formula = value;

    } else if (pid == "14") {
      reply_bytes = 1;
      value = eng_oil_press;
      after_formula = value;

    } else if (pid == "15") {
      reply_bytes = 2;
      value = analog_in_5;
      after_formula = value;
    }

    mode_id_rsp = MODE_21_RSP;
    replyOBD2(mode_id_rsp, pid, after_formula, reply_bytes);

  } else if (request.startsWith(AT_REQ)) {
    // Process AT requests. They are used to set up the connection
    // between the microcontroller and the client app

    ATCommand = request.substring(2);

    if (SERIAL_DEBUG) {
      Serial.println(AT_REQ + ATCommand);
    }

    if (ATCommand == "Z") {
      // Reset all
      initSensors();
      replyVersion();
      replyOK();

    } else if (ATCommand == "E0") {
      replyNotImplemented();

    } else if (ATCommand == "M0") {
      replyNotImplemented();

    } else if (ATCommand == "L0") {
      replyNotImplemented();

    } else if (ATCommand == "ST62") {
      replyNotImplemented();

    } else if (ATCommand == "S0") {
      replyNotImplemented();

    } else if (ATCommand == "H0") {
      replyNotImplemented();

    } else if (ATCommand == "H1") {
      replyNotImplemented();

    } else if (ATCommand == "AT1") {
      replyNotImplemented();

    } else if (ATCommand == "@1") {
      // Device description (adapter manufacturer)
      replyDescription();

    } else if (ATCommand == "I") {
      // Adapter firmware version
      replyVersion();

    } else if (ATCommand == "SP0") {
      // Set protocol to auto
      replyNotImplemented();

    } else if (ATCommand == "DPN") {
      // Device Protocol Number
      replyValue("1"); //just say it is number 1.

    } else if (ATCommand == "RV") {
      // Read Voltage
      replyValue("12.5");

    } else if (ATCommand == "PC") {
      // Protocol Close: terminates current diagnostic session
      replyOK();
    }
  }

  replyPrompt();

  request = "";  // Clear the request buffer once processed
}

// Sends a reply to an OBD2 request
void replyOBD2(String mode_response, String pid, unsigned long value, byte reply_bytes) {
  String modeReply = mode_response + pid + toHexReply(value, reply_bytes);

  BtSerial.println(modeReply);
  if (SERIAL_DEBUG) {
    Serial.println(modeReply);
  }

}

// Sends an "OK" reply to an AT command request to acknowledge reception
void replyOK() {
  replyValue(OK);
}

// Sends an "OK" reply to an AT command request for an unimplemented command.
// The client app will then generally ignore the response, but it does expect
// an acknowledgement of reception
void replyNotImplemented() {
  replyOK();
}

// Sends the firmware version number upon AT command request
void replyVersion() {
  replyValue(VERSION);
}

// Sends the device description upon AT command request
void replyDescription() {
  replyValue(DEVICE_DESCRIPTION);
}

// Sends the prompt character to indicate that the simulator is idle and
// awaiting a command
void replyPrompt() {
  replyValue(PROMPT);
}

// Sends a value as a request to either an AT or OBD2 command
void replyValue(const String value) {
  BtSerial.println(value);
  if (SERIAL_DEBUG) {
    Serial.println(value);
  }
}

// Sends a response to an OBD2 command
void replyOBDResponse(const String response) {
  if (response.length() > 0) {
    replyValue(response);
  }
}

// Updates the sensor values
void updateSensorValues() {
  // Update RPM every n sample counts:
  // - increase n for better RPM resolution,
  // - decrease for faster update
  const int RPM_SAMPLE_COUNT = 30;

  // Sensor data sent as mode 1 PIDs
  if (rpm_pulse_count >= 30) {

    rpm = (30UL * 1000UL * 1000UL / (micros() - time_old)) * rpm_pulse_count;
    time_old = micros();

    rpm_pulse_count = 0;
    eng_rpm = rpm;
  }

  eng_coolant_temp = 93;
  vehicle_speed = 155;
  in_manifld_press_abs = 200;
  ambient_air_temp = 22; // Outside temperature 22 °C
  eng_oil_temp = 109; // Oil temp. 109 °C
  fuel_percentage = 89.0;

  // Sensor data sent as custom mode 21 PIDs
  eng_oil_press = 65;
  analog_in_5 = analogRead(5);
  vehicle_speed_2 = 312;
}

// Initialize the sensor definitions and pin states and function
void initSensors() {

  rpm_pulse_count = 0;
  rpm = 0;
  time_old = 0;

  attachInterrupt(digitalPinToInterrupt(rpmPin), onRpm, RISING);

}

// Increase pulse count upon an interrupt on the RPM monitoring pin
void onRpm() {
  rpm_pulse_count++;
}

// Return a hex string with padded zeroes up to the specified width
String toHexReply(unsigned long value, int width) {

  // Generally the sprintf() function would be used, but its Arduino
  // implementation lacks most of the advanced functionality from its
  // standard C++ counterpart. Thus we use a sort of a brute force
  // approach by prepending zeroes until the given width is achieved
  String value_hex = String(value, HEX);
  int value_hex_len = value_hex.length();
  const int MAX_REPLY_LEN = 4; // in bytes

  if (width <= MAX_REPLY_LEN) {
    // Width was specified in bytes:
    // 1 byte = 2 hex characters
    width *= 2;
    while ((width - value_hex_len) > 0) {
      value_hex = "0" + value_hex;
      value_hex_len++;
    }
  }

  return value_hex;
}
