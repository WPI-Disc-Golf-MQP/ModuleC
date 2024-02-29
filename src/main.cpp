#define NODE_NAME String("module_c")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include "Adafruit_VL6180X.h"
Adafruit_VL6180X vl = Adafruit_VL6180X();

// ----- OUTTAKE -----

MODULE* outtake_module;
int OUTTAKE_BEAM_BREAK_PIN = D10; //TODO: VERIFY PIN NUMBER
int OUTTAKE_SPEED_PIN = A0;  //TODO: VERIFY PIN NUMBER
int OUTTAKE_INVERT_PIN = D13;  //TODO: VERIFY PIN NUMBER

enum OUTTAKE_STATE {
    OUTTAKE_IDLE = 0, 
    };
OUTTAKE_STATE outtake_state = OUTTAKE_STATE::OUTTAKE_IDLE; 

void outtake_move_forward(int speed = 230) {
  digitalWrite(OUTTAKE_INVERT_PIN, LOW);
  analogWrite(OUTTAKE_SPEED_PIN, speed); // start
  loginfo("outtake moving forward");
}

void outtake_move_backward(int speed = 230) {
  digitalWrite(OUTTAKE_INVERT_PIN, HIGH);
  analogWrite(OUTTAKE_SPEED_PIN, speed); // start
  loginfo("outtake moving backward");
}

bool outtake_beam_broken() {
  return (digitalRead(OUTTAKE_BEAM_BREAK_PIN) == 0);
}

void start_outtake() {
  //outtake_state = OUTTAKE_STATE
  outtake_move_forward();
}

void stop_outtake() {
  analogWrite(OUTTAKE_SPEED_PIN, 0); // stop
  if (outtake_state != OUTTAKE_STATE::OUTTAKE_IDLE) {
    loginfo("stop");
    outtake_state = OUTTAKE_STATE::OUTTAKE_IDLE;
  }
}

void calibrate_outtake() {
  loginfo("calibrate outtake; TODO"); //TODO: Implement calibration
}

void check_outtake() {
  switch (outtake_state){
    case OUTTAKE_STATE::OUTTAKE_IDLE:
      stop_outtake();
      break;
    default:
      logwarn("Invalid outtake state");
      break;
  }
  outtake_module->publish_state((int) outtake_state);
}

bool verify_outtake_complete() {
  return outtake_state == OUTTAKE_STATE::OUTTAKE_IDLE;
}

// ----- LABEL TAMPER -----

MODULE* label_tamper_module;
int TAMPER_NEAR_SWITCH_PIN = D10; //TODO: VERIFY PIN NUMBER
int TAMPER_FAR_SWITCH_PIN = D10; //TODO: VERIFY PIN NUMBER
int TAMPER_SPEED_PIN = A0;  //TODO: VERIFY PIN NUMBER
int TAMPER_INVERT_PIN = D13;  //TODO: VERIFY PIN NUMBER

enum LABEL_TAMPER_STATE {
  LABEL_TAMPER_IDLE = 0, 
};
LABEL_TAMPER_STATE label_tamper_state = LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE; 

void tamper_move_forward(int speed = 230) {
  digitalWrite(TAMPER_INVERT_PIN, LOW);
  analogWrite(TAMPER_SPEED_PIN, speed); // start
  loginfo("tamper moving forward");
}

void tamper_move_backward(int speed = 230) {
  digitalWrite(TAMPER_INVERT_PIN, HIGH);
  analogWrite(TAMPER_SPEED_PIN, speed); // start
  loginfo("tamper moving backward");
}

void start_tamper() {
  //outtake_state = LABEL_TAMPER_STATE
  tamper_move_forward();
}

void stop_tamper() {
  analogWrite(TAMPER_SPEED_PIN, 0); // stop
  if (label_tamper_state != LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE) {
    loginfo("stop");
    label_tamper_state = LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE;
  }
}

void calibrate_tamper() {
  loginfo("calibrate label tamper; TODO"); //TODO: Implement calibration
}

void check_tamper() {
  switch (outtake_state){
    case LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE:
      stop_tamper();
      break;
    default:
      logwarn("Invalid label tamper state");
      break;
  }
  label_tamper_module->publish_state((int) label_tamper_state);
}

bool verify_tamper_complete() {
  return label_tamper_state == LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE;
}

// ----- BOX_CONVEYOR -----

MODULE* box_conveyor_module;
int BOX_CONVEYOR_BEAM_BREAK_PIN = D10; //TODO: VERIFY PIN NUMBER
int BOX_CONVEYOR_SPEED_PIN = A0;  //TODO: VERIFY PIN NUMBER
int BOX_CONVEYOR_INVERT_PIN = D13;  //TODO: VERIFY PIN NUMBER

enum BOX_CONVEYOR_STATE {
  BOX_CONVEYOR_IDLE = 0, 
};
BOX_CONVEYOR_STATE box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE; 

uint8_t read_distance() {
  
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) return range;
  
  // Some error occurred, print it out!
  Serial.print("Error: ");
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    logerr("*** VL6180X System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    logerr("VL6180X: ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    logerr("VL6180X: No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    logerr("VL6180X: Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    logerr("VL6180X: Signal/Noise ratio error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    logerr("VL6180X: Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    logerr("VL6180X: Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    logerr("VL6180X: Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    logerr("VL6180X: Range reading overflow");
  } else {
    logerr("VL6180X: Unknown error");
  }
  return -1;
}

void box_conveyor_move_forward(int speed = 230) {
  digitalWrite(BOX_CONVEYOR_INVERT_PIN, LOW);
  analogWrite(BOX_CONVEYOR_SPEED_PIN, speed); // start
  loginfo("box_conveyor moving forward");
}

void box_conveyor_move_backward(int speed = 230) {
  digitalWrite(BOX_CONVEYOR_INVERT_PIN, HIGH);
  analogWrite(BOX_CONVEYOR_SPEED_PIN, speed); // start
  loginfo("box_conveyor moving backward");
}

bool box_conveyor_beam_broken() {
  return (digitalRead(BOX_CONVEYOR_BEAM_BREAK_PIN) == 0);
}

void start_box_conveyor() {
  //box_conveyor_state = BOX_CONVEYOR_STATE
  box_conveyor_move_forward();
}

void stop_box_conveyor() {
  analogWrite(BOX_CONVEYOR_SPEED_PIN, 0); // stop
  if (box_conveyor_state != BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE) {
    loginfo("stop");
    box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE;
  }
}

void calibrate_box_conveyor() {
  loginfo("calibrate box_conveyor; TODO"); //TODO: Implement calibration
}

void check_box_conveyor() {
  switch (box_conveyor_state){
    case BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE:
      stop_box_conveyor();
      break;
    default:
      logwarn("Invalid box_conveyor state");
      break;
  }
  box_conveyor_module->publish_state((int) box_conveyor_state);
}

bool verify_box_conveyor_complete() {
  return box_conveyor_state == BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE;
}

// ----- loop/setup functions -----
void setup() {
  init_std_node();

  outtake_module = init_module("outtake",
    start_outtake, 
    verify_outtake_complete, 
    stop_outtake,
    calibrate_outtake);

  label_tamper_module = init_module("label_tamper",
    start_tamper, 
    verify_tamper_complete, 
    stop_tamper,
    calibrate_tamper);

  box_conveyor_module = init_module("box_conveyor",
    start_box_conveyor, 
    verify_box_conveyor_complete, 
    stop_box_conveyor,
    calibrate_box_conveyor);
  
  // outtake pins 
  pinMode(OUTTAKE_BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(OUTTAKE_SPEED_PIN,OUTPUT) ;
  pinMode(OUTTAKE_INVERT_PIN, OUTPUT) ;

  // label tamper pins
  pinMode(TAMPER_NEAR_SWITCH_PIN, INPUT_PULLUP) ;
  pinMode(TAMPER_FAR_SWITCH_PIN, INPUT_PULLUP) ;
  pinMode(TAMPER_SPEED_PIN,OUTPUT) ;
  pinMode(TAMPER_INVERT_PIN, OUTPUT) ;

  // box conveyor pins
  pinMode(BOX_CONVEYOR_BEAM_BREAK_PIN, INPUT_PULLUP) ;
  pinMode(BOX_CONVEYOR_SPEED_PIN,OUTPUT) ;
  pinMode(BOX_CONVEYOR_INVERT_PIN, OUTPUT) ;

  if (! vl.begin()) {
    logerr("*** Failed to find VL6180X (Box Conveyor Rangefinder) sensor");
  } else loginfo("VL6180X (Box Conveyor Rangefinder) Sensor found!");

  loginfo("setup() Complete");
}


void loop() {
  periodic_status();
  nh.spinOnce();
  check_outtake();
  check_tamper();

  // ----- testing ----- 
  // if (verify_motion_complete()) {
  //   loginfo("debugging test reset");
  //   delay(5000);
  // }
}
