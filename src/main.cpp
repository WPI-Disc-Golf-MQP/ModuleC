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
int OUTTAKE_BEAM_BREAK_PIN = D2; 
int OUTTAKE_SPEED_PIN = A1;  
int OUTTAKE_INVERT_PIN = D10;  

enum OUTTAKE_STATE {
    OUTTAKE_IDLE = 0,
    OUTTAKE_SEND = 1,
    OUTTAKE_RECIEVE = 2, 
    };
OUTTAKE_STATE outtake_state = OUTTAKE_STATE::OUTTAKE_IDLE; 

bool is_disc_present = false;
long moved_to_OUTTAKE_RELEASE_time = millis();
bool deposited_disc = false;

void outtake_move_forward(int speed = 230) {
  digitalWrite(OUTTAKE_INVERT_PIN, LOW);
  analogWrite(OUTTAKE_SPEED_PIN, speed); // start
  loginfo("outtake moving forward");
}

// void outtake_move_backward(int speed = 230) {
//   digitalWrite(OUTTAKE_INVERT_PIN, HIGH);
//   analogWrite(OUTTAKE_SPEED_PIN, speed); // start
//   loginfo("outtake moving backward");
// }

bool val = 0;

bool outtake_beam_broken() {
  if (digitalRead(OUTTAKE_BEAM_BREAK_PIN) != val) {
    loginfo("Outtake beam break changed state to: "+String(digitalRead(OUTTAKE_BEAM_BREAK_PIN)));
    val = digitalRead(OUTTAKE_BEAM_BREAK_PIN);
  }
}

void start_outtake() {
  //outtake_state = OUTTAKE_STATE
  loginfo("start outtake");
  if (true) {
    outtake_state = OUTTAKE_STATE::OUTTAKE_SEND;
    moved_to_OUTTAKE_RELEASE_time = millis();
    outtake_move_forward();
  } else {
    outtake_state = OUTTAKE_STATE::OUTTAKE_RECIEVE;
  }
  
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
    case OUTTAKE_STATE::OUTTAKE_SEND:
      if (moved_to_OUTTAKE_RELEASE_time+2000 < millis()) {
        is_disc_present = false;
        outtake_state = OUTTAKE_STATE::OUTTAKE_RECIEVE;
        start_outtake();
        outtake_move_forward();
      }
    case OUTTAKE_STATE::OUTTAKE_RECIEVE:
      if (outtake_beam_broken()) {
        is_disc_present = true;
        outtake_state = OUTTAKE_STATE::OUTTAKE_IDLE;
        outtake_module->publish_status(MODULE_STATUS::COMPLETE);
      };
      break;
    default:
      logwarn("Invalid outtake state");
      break;
  }
  outtake_module->publish_state((int) outtake_state);
  outtake_beam_broken();
}

bool verify_outtake_complete() {
  return outtake_state == OUTTAKE_STATE::OUTTAKE_IDLE;
}

// ----- LABEL TAMPER -----

// MODULE* label_tamper_module;
// int TAMPER_NEAR_SWITCH_PIN = D6; 
// int TAMPER_FAR_SWITCH_PIN = D7; 
// int TAMPER_SPEED_PIN = A2; 
// int TAMPER_INVERT_PIN = D9;  

// enum LABEL_TAMPER_STATE {
//   LABEL_TAMPER_IDLE = 0, 
// };
// LABEL_TAMPER_STATE label_tamper_state = LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE; 

// void tamper_move_forward(int speed = 230) {
//   digitalWrite(TAMPER_INVERT_PIN, LOW);
//   analogWrite(TAMPER_SPEED_PIN, speed); // start
//   loginfo("tamper moving forward");
// }

// void tamper_move_backward(int speed = 230) {
//   digitalWrite(TAMPER_INVERT_PIN, HIGH);
//   analogWrite(TAMPER_SPEED_PIN, speed); // start
//   loginfo("tamper moving backward");
// }

// void start_tamper() {
//   //outtake_state = LABEL_TAMPER_STATE
//   tamper_move_forward();
// }

// void stop_tamper() {
//   analogWrite(TAMPER_SPEED_PIN, 0); // stop
//   if (label_tamper_state != LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE) {
//     loginfo("stop");
//     label_tamper_state = LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE;
//   }
// }

// void calibrate_tamper() {
//   loginfo("calibrate label tamper; TODO"); //TODO: Implement calibration
// }

// void check_tamper() {
//   switch (outtake_state){
//     case LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE:
//       stop_tamper();
//       break;
//     default:
//       logwarn("Invalid label tamper state");
//       break;
//   }
//   label_tamper_module->publish_state((int) label_tamper_state);
// }

// bool verify_tamper_complete() {
//   return label_tamper_state == LABEL_TAMPER_STATE::LABEL_TAMPER_IDLE;
// }

// ----- BOX_CONVEYOR -----

 enum BOX_CONVEYOR_STATE {
   BOX_CONVEYOR_IDLE = 0, 
   BOX_CONVEYOR_ALIGN = 1,
   BOX_CONVEYOR_ADVANCE = 2,
  BOX_CONVEYOR_ERROR = 3,
 };

 BOX_CONVEYOR_STATE box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE; 

 Adafruit_VL6180X vl6180x;

 int BOX_CONVEYOR_BEAM_BREAK_PIN = D3; 
 int BOX_CONVEYOR_RANGEFINDER_PIN_SCL = D5; 
 int BOX_CONVEYOR_RANGEFINDER_PIN_SDA = D4;
 int BOX_CONVEYOR_SPEED_PIN = A0;  
 int BOX_CONVEYOR_INVERT_PIN = D13;  

 MODULE* box_conveyor_module;

 unsigned long unbroken_box_beam_start = 0;
 const unsigned long unbroken_box_beam_threshold = 6000;
 uint8_t read_distance();

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

bool move_box_conveyor = (read_distance() > 50) && !box_conveyor_beam_broken;
 void start_box_conveyor() {
  nh.loginfo("Box conveyor is starting");
    box_conveyor_move_forward();


  // if(box_conveyor_state == BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE) {
  //   loginfo("start_box_conveyor in IDLE --> advancing box");
  //   box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ADVANCE;
  //   box_conveyor_move_forward();
  // } else if (box_conveyor_state == BOX_CONVEYOR_STATE::BOX_CONVEYOR_ADVANCE) {
  //   if (move_box_conveyor) {
  //     loginfo("start_box_conveyor in ADVANCE --> aligning box");
  //     box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ALIGN;
  //     box_conveyor_move_backward;
  //     }
  //   }
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
  unsigned long current_time = millis();

  switch (box_conveyor_state) {
     case BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE:
      if(box_conveyor_beam_broken() == true) {
        unbroken_box_beam_start = current_time;
        box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ADVANCE;
      } else {
        if (current_time -  unbroken_box_beam_start > unbroken_box_beam_threshold) {
          stop_box_conveyor();
          logerr("No boxes detected for too long. Add more boxes.");
        }
      }

       break;
     case BOX_CONVEYOR_STATE::BOX_CONVEYOR_ADVANCE:
       if (box_conveyor_beam_broken() == false) {
         start_box_conveyor();
         box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ALIGN;
         unbroken_box_beam_start = current_time;
       }
       break;
     case BOX_CONVEYOR_STATE::BOX_CONVEYOR_ALIGN:
       if (read_distance() > 50) {
         stop_box_conveyor();
         box_conveyor_state = BOX_CONVEYOR_IDLE;
       }
   }
 }

 bool verify_box_conveyor_complete() {
   return box_conveyor_state == BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE;
 }


// uint8_t read_distance() {
//   return -1;
// }

 uint8_t read_distance() {
    uint8_t range = vl6180x.readRange();
    nh.loginfo("Distance reading:");
    if (range == -1) {
      nh.logerror("Failed to read distance.");
    }
    uint8_t status = vl6180x.readRangeStatus();

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

// void check_box_conveyor() {
//   switch (box_conveyor_state){
//     case BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE:
//       stop_box_conveyor();
//       break;
//     default:
//       logwarn("Invalid box_conveyor state");
//       break;
//   }
//   box_conveyor_module->publish_state((int) box_conveyor_state);
// }

// ----- loop/setup functions -----
void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.loginfo("Initializing ROS node");

  init_std_node();

  Wire.begin(BOX_CONVEYOR_RANGEFINDER_PIN_SDA, BOX_CONVEYOR_RANGEFINDER_PIN_SCL);
  vl6180x.begin();

  // outtake_module = init_module("outtake",
  //   start_outtake, 
  //   verify_outtake_complete, 
  //   stop_outtake,
  //   calibrate_outtake);

  // label_tamper_module = init_module("label_tamper",
  //   start_tamper, 
  //   verify_tamper_complete, 
  //   stop_tamper,
  //   calibrate_tamper);

  MODULE* box_conveyor_module = init_module("box_conveyor",
    start_box_conveyor, 
    verify_box_conveyor_complete, 
    stop_box_conveyor,
    calibrate_box_conveyor);
  
  // outtake pins 
  // pinMode(OUTTAKE_BEAM_BREAK_PIN, INPUT_PULLUP) ;
  // pinMode(OUTTAKE_SPEED_PIN,OUTPUT) ;
  // pinMode(OUTTAKE_INVERT_PIN, OUTPUT) ;

  // label tamper pins
  // pinMode(TAMPER_NEAR_SWITCH_PIN, INPUT_PULLUP) ;
  // pinMode(TAMPER_FAR_SWITCH_PIN, INPUT_PULLUP) ;
  // pinMode(TAMPER_SPEED_PIN,OUTPUT) ;
  // pinMode(TAMPER_INVERT_PIN, OUTPUT) ;

  // box conveyor pins
  // pinMode(BOX_CONVEYOR_BEAM_BREAK_PIN, INPUT_PULLUP) ;
  // pinMode(BOX_CONVEYOR_SPEED_PIN,OUTPUT) ;
  // pinMode(BOX_CONVEYOR_INVERT_PIN, OUTPUT) ;

  // if (! vl.begin()) {
  //   logerr("*** Failed to find VL6180X (Box Conveyor Rangefinder) sensor");
  // } else loginfo("VL6180X (Box Conveyor Rangefinder) Sensor found!");

  loginfo("setup() Complete");
}



void loop() {
  periodic_status();
  nh.spinOnce();
  // check_outtake();
  // check_tamper();
  check_box_conveyor();
  loginfo("System is running");
  delay(1000);

  // ----- testing ----- 
  // if (verify_box_conveyor_complete()) {
  //   loginfo("debugging test reset");
  //   delay(5000);
  //   start_box_conveyor();
  // }
  // uint8_t distance = read_distance();
//   bool beam_break_block = box_conveyor_beam_broken();

//   int distance_max = 50; //Placeholder for now
//   bool move_box_conveyor = (distance > distance_max) && !beam_break_block;

//   if (move_box_conveyor) {
//     start_box_conveyor();
//   } else {
//     stop_box_conveyor();

//     if (!beam_break_block) {
//       if (unbroken_box_beam_start == 0) {
//         unbroken_box_beam_start = millis();
//       } else {
//         if ((millis() - unbroken_box_beam_start) >= unbroken_box_beam_threshold) {
//           Serial.println("Add boxes");

//           stop_outtake();
//           stop_tamper();
//           stop_box_conveyor();
//         }
//       }
//     }
//     else {
//       unbroken_box_beam_start = 0;
//     }
//   }

//   delay(100);
 }
