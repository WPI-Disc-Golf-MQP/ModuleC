#define NODE_NAME String("module_c")
#define STATUS_FREQ 1500 // ms

#include <std_node.cpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
// TODO:BACKING
#include "Adafruit_VL6180X.h"
Adafruit_VL6180X vl = Adafruit_VL6180X();

// ----- CHUTE -----
MODULE* chute_module;
int CHUTE_BEAM_BREAK_PIN = 2; //Verified 2/21
int CHUTE_SPEED_PIN = A0; //Updated 2/21
int CHUTE_INVERT_PIN = 5; //Updated 2/21

enum CHUTE_STATE {
  CHUTE_IDLE = 0,           // Idle
  CHUTE_RECIEVE = 1,        // Checks if disc is in the chute
  CHUTE_SEND = 2            // Moves disc through the chute into the box
};

CHUTE_STATE chute_state = CHUTE_STATE::CHUTE_IDLE; 

bool is_disc_present = false;
bool deposited_disc = false;
bool recieve_chute_start_msg = false;
bool box_full = false;

// Moves the chute motor forward
void chute_move_forward(int speed = 230) {
  digitalWrite(CHUTE_INVERT_PIN, HIGH);
  analogWrite(CHUTE_SPEED_PIN, speed); // start
  loginfo("chute moving forward");
}

// Moves the chute motor backward
void chute_move_backward(int speed = 230) {
  digitalWrite(CHUTE_INVERT_PIN, LOW);
  analogWrite(CHUTE_SPEED_PIN, speed); // start
  loginfo("chute moving backward");
}

bool val = 0;
// Checks if the chute beam is broken
bool chute_beam_broken() {
  if (digitalRead(CHUTE_BEAM_BREAK_PIN) != val) {
    loginfo("Chute beam break changed state to: "+String(digitalRead(CHUTE_BEAM_BREAK_PIN)));
    val = digitalRead(CHUTE_BEAM_BREAK_PIN);
  }
  return (digitalRead(CHUTE_BEAM_BREAK_PIN) == 0);
}

// Starts the outtake
void start_chute() {
  // outtake_state = OUTTAKE_STATE
  // loginfo("start outtake");
  if (true) {
    //chute_state = CHUTE_STATE::CHUTE_SEND;
    chute_move_forward();
  } else {
    chute_state = CHUTE_STATE::CHUTE_RECIEVE;
  }
  
}

// Stops the outtake
void stop_chute() {
  analogWrite(CHUTE_SPEED_PIN, 0); // stop
  loginfo("stop");
  // if (chute_state != CHUTE_STATE::CHUTE_IDLE) {
  //   loginfo("stop");
  //   chute_state = CHUTE_STATE::CHUTE_IDLE;
  // }
}

// Calibrates the chute
void calibrate_chute() {
  loginfo("calibrate chute; TODO"); //TODO: Implement calibration
}

// Chute switch case
void check_chute() {
  switch (chute_state){
    case CHUTE_STATE::CHUTE_IDLE:
      if(chute_beam_broken()) {
        deposited_disc = false;
        chute_state = CHUTE_STATE::CHUTE_RECIEVE;
      }
      break;
    case CHUTE_STATE::CHUTE_RECIEVE:
      if(!box_full) {
        chute_state = CHUTE_STATE::CHUTE_SEND;
        start_chute();
      }
      break;
    case CHUTE_STATE::CHUTE_SEND:
      if(chute_beam_broken() == false){
        chute_state = CHUTE_STATE::CHUTE_IDLE;
        stop_chute();
        deposited_disc = true;
      }
      break;
  }
}

// Checks if the chute is idle
bool verify_chute_complete() {
  return chute_state == CHUTE_STATE::CHUTE_IDLE;
}



// ----- BACKING -----
MODULE* backing_module;
int BACKING_SPEED_PIN = 8; //Updates 2/21
int BACKING_INVERT_PIN = 7; //Updated 2/21

// TODO:BACKING
// int BACKING_RANGEFINDER_PIN_SCL = ;
// int BACKING_RANGEFINER_PIN_SDA = ;

enum BACKING_STATE {
  BACKING_IDLE = 0,       // Idle
  BACKING_MOVE = 1
};

BACKING_STATE backing_state = BACKING_STATE::BACKING_IDLE;

// TODO:BACKING
// Adafruit_VL6180X vl6180x;
// uint8_t backing_read_distance();
// int backing_raise_threshold = 10;
// int backing_lower_threshold = 70;

unsigned long backing_start_time = 0;
const unsigned long backing_threshold = 1000;



bool raise = true;
bool prev_raise = true;
bool slotted_box = false;

// Moves the backing motor forward
void backing_move_forward(int speed = 230) {
  digitalWrite(BACKING_INVERT_PIN, LOW);
  analogWrite(BACKING_SPEED_PIN, speed);
  loginfo("backing moving down");
}

// Moves the backing motor backward
void backing_move_backward(int speed = 230) {
  digitalWrite(BACKING_INVERT_PIN, HIGH);
  analogWrite(BACKING_SPEED_PIN, speed);
  loginfo("backing moving up");
}

// Starts the backing motor
void start_backing() {
  if (raise) {
    backing_move_forward();

  }else{
    backing_move_backward();
  }
}

// Stops the backing motor
void stop_backing() {
  analogWrite(BACKING_SPEED_PIN, 0);
}

// Checks whether the backing is raised or lowered
bool check_raise_status() {
  if (prev_raise != raise){
    prev_raise = raise;
    // loginfo("backing returning true to move");
    return true;
  }
  // loginfo("backing returning false to move");
  return false;
}

// Calibrates the backing
void calibrate_backing() {
  loginfo("calibrate backing; TODO"); //TODO:BACKING
  // while (backing_read_distance() > backing_raise_threshold) {
  //   backing_move_forward();
  // }
 }

// Backing switch case
void check_backing() {
  unsigned long backing_current_time = millis();
  switch (backing_state){
    case BACKING_STATE::BACKING_IDLE:
      if (check_raise_status() && !slotted_box) {
        backing_start_time = backing_current_time;
        backing_state = BACKING_STATE::BACKING_MOVE;
        start_backing();
      }
      break;
    case BACKING_STATE::BACKING_MOVE:
      // TODO:BACKING
      // if (raise && backing_read_distance() < backing_raise_threshold || !raise && backing_read_distance() > backing_lower_threshold) {
      //   stop_backing();
      //   backing_state = BACKING_STATE::BACKING_IDLE;
      // }
      if (backing_current_time - backing_start_time > backing_threshold) {
        stop_backing();
        backing_state = BACKING_STATE::BACKING_IDLE;
      }
      break;
  }
}

// Verifies if the backing is complete
bool verify_backing_complete() {
  return backing_state == BACKING_STATE::BACKING_IDLE;
}

//  uint8_t backing_read_distance() { //This is the requesting topics error
//     uint8_t range = vl6180x.readRange();
//     nh.loginfo("Distance reading:");
//     if (range == -1) {
//       nh.logerror("Failed to read distance.");
//     }
//     uint8_t status = vl6180x.readRangeStatus();

//    if (status == VL6180X_ERROR_NONE) return range;
  
//   //  Some error occurred, print it out!
//    Serial.print("Error: ");
  
//    if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
//      logerr("*** VL6180X System error");
//    }
//    else if (status == VL6180X_ERROR_ECEFAIL) {
//      logerr("VL6180X: ECE failure");
//    }
//    else if (status == VL6180X_ERROR_NOCONVERGE) {
//      logerr("VL6180X: No convergence");
//    }
//    else if (status == VL6180X_ERROR_RANGEIGNORE) {
//      logerr("VL6180X: Ignoring range");
//    }
//    else if (status == VL6180X_ERROR_SNR) {
//      logerr("VL6180X: Signal/Noise ratio error");
//    }
//    else if (status == VL6180X_ERROR_RAWUFLOW) {
//      logerr("VL6180X: Raw reading underflow");
//    }
//    else if (status == VL6180X_ERROR_RAWOFLOW) {
//      logerr("VL6180X: Raw reading overflow");
//    }
//    else if (status == VL6180X_ERROR_RANGEUFLOW) {
//      logerr("VL6180X: Range reading underflow");
//    }
//    else if (status == VL6180X_ERROR_RANGEOFLOW) {
//      logerr("VL6180X: Range reading overflow");
//    } else {
//      logerr("VL6180X: Unknown error");
//    }
//    return -1;
//  }



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
MODULE* box_conveyor_module;
int FRONT_BEAM_BREAK_PIN = 4; //Updated 2/21
int BACK_BEAM_BREAK_PIN = 3; //Updated 2/21
int BOX_CONVEYOR_SPEED_PIN = 9; //Verified 2/21
int BOX_CONVEYOR_INVERT_PIN = 6; //Verified 2/21

enum BOX_CONVEYOR_STATE {
  BOX_CONVEYOR_IDLE = 0, 
  BOX_CONVEYOR_ALIGN = 1,
  BOX_CONVEYOR_ADVANCE = 2,
  BOX_CONVEYOR_ERROR = 3,
  BOX_CONVEYOR_EJECT_BOX = 4
};

BOX_CONVEYOR_STATE box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE; 



unsigned long unbroken_back_beam_start = 0;
unsigned long unbroken_front_beam_start = 0;
unsigned long broken_front_beam_start = 0;
const unsigned long unbroken_box_beam_threshold = 6000;
unsigned long box_move_start = 0;
unsigned long box_advance_threshold = 500;
unsigned long eject_box_start;
const unsigned long eject_box_threshold = 2000;
bool eject_box = false;
// uint8_t read_distance();


// Moves box conveyor forward
void box_conveyor_move_forward(int speed = 150) {
  digitalWrite(BOX_CONVEYOR_INVERT_PIN, LOW);
  analogWrite(BOX_CONVEYOR_SPEED_PIN, speed); // start
  loginfo("box_conveyor moving forward");
}

// Moves box conveyor backward
void box_conveyor_move_backward(int speed = 230) {
  digitalWrite(BOX_CONVEYOR_INVERT_PIN, HIGH);
  analogWrite(BOX_CONVEYOR_SPEED_PIN, speed); // start
  loginfo("box_conveyor moving backward");
}

bool front_val = 0;
// Checks if the front beam is broken
bool front_beam_broken() {
  if (digitalRead(FRONT_BEAM_BREAK_PIN) !=front_val) {
    loginfo("Front beam break changed state to: "+String(digitalRead(FRONT_BEAM_BREAK_PIN)));
    front_val = digitalRead(FRONT_BEAM_BREAK_PIN);
  }
  return (digitalRead(FRONT_BEAM_BREAK_PIN) == 0);
}

bool back_val = 0;
// Checks if the back beam is broken
bool back_beam_broken() {
  if (digitalRead(BACK_BEAM_BREAK_PIN) !=back_val) {
    loginfo("Back beam break changed state to: "+String(digitalRead(BACK_BEAM_BREAK_PIN)));
    back_val = digitalRead(BACK_BEAM_BREAK_PIN);
  }
  return (digitalRead(BACK_BEAM_BREAK_PIN) == 0);
}

//bool move_box_conveyor = (read_distance() > 50) && !box_conveyor_beam_broken;
void start_box_conveyor() {
  nh.loginfo("Box conveyor is starting");
  box_conveyor_move_forward();
}

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

void stop_box_conveyor() {
  analogWrite(BOX_CONVEYOR_SPEED_PIN, 0); // stop
  if (box_conveyor_state != BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE) {
    loginfo("stop");
    box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE;
  }
}


void calibrate_box_conveyor() {
  loginfo("calibrate box_conveyor; TODO"); //TODO: Implement calibration
  while (front_beam_broken() || back_beam_broken()) {
    box_conveyor_move_backward();
  }
}

void check_box_conveyor() {
  unsigned long current_time = millis();

  switch (box_conveyor_state) {
    case BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE:
      // no box present / ready for loading: start conveyor to align box
      if (front_beam_broken() == false && back_beam_broken() == false) {
        unbroken_back_beam_start = current_time;
        box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ALIGN;
        start_box_conveyor();
      
      // the backing is not moving and a disc has been deposited: start conveyor to prepare for next disc
      } else if (verify_backing_complete() && deposited_disc) {
        box_move_start = current_time;
        deposited_disc = false;
        box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ADVANCE;
        start_box_conveyor();

      // the box is full: stop the conveyor and raise the backing
      } else if (front_beam_broken() == true && back_beam_broken() == false) {
        stop_box_conveyor();
        if (!slotted_box) {
          raise = true;
        }
        box_full = true;
        loginfo("The box is full. Please replace");
        box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ERROR;
      
      } else if (eject_box){
        box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_EJECT_BOX;
        raise = true;
        start_box_conveyor();
      }
      break;

    case BOX_CONVEYOR_STATE::BOX_CONVEYOR_ADVANCE:
      loginfo("Conveyor Advancing");
      if (current_time - box_move_start > box_advance_threshold) {
        stop_box_conveyor();
        box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE;
      }
      break;

    case BOX_CONVEYOR_STATE::BOX_CONVEYOR_ALIGN:
      loginfo("Conveyor Aligning");
      if (front_beam_broken()) {
        broken_front_beam_start = current_time;
        if (current_time - broken_front_beam_start > box_advance_threshold) {
          stop_box_conveyor();
          box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE;
          if (!slotted_box) {
            raise = false;
          }
          box_full = false;
        }

      } else if (!back_beam_broken() && current_time - unbroken_back_beam_start > unbroken_box_beam_threshold) {
        stop_box_conveyor();
        box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ERROR;
        logerr("No boxes detected for too long. Add more boxes.");
      }
      break;

    case BOX_CONVEYOR_STATE::BOX_CONVEYOR_ERROR:
      if (back_beam_broken()){ // Change to allow user to say when a box is loaded
        box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_ALIGN;
        start_box_conveyor();
      }
      break;
    
    case BOX_CONVEYOR_STATE::BOX_CONVEYOR_EJECT_BOX:
      if (!back_beam_broken() && !front_beam_broken()) {
        eject_box_start = current_time;
        if (current_time - eject_box_start > eject_box_threshold) {
          stop_box_conveyor();
          eject_box = false;
          box_conveyor_state = BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE;
        }  
      }
  }
 }

bool verify_box_conveyor_complete() {
   return box_conveyor_state == BOX_CONVEYOR_STATE::BOX_CONVEYOR_IDLE;
}

// uint8_t read_distance() {
//   return -1;
// }




// ----- loop/setup functions -----
void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.loginfo("Initializing ROS node");

  init_std_node();

  // TODO:BACKING
  // Wire.begin(BACKING_RANGEFINDER_PIN_SDA, BACKING_RANGEFINDER_PIN_SCL);
  // vl6180x.begin();

  MODULE* chute_module = init_module("chute",
    start_chute, 
    verify_chute_complete, 
    stop_chute,
    calibrate_chute);

  MODULE* backing_module = init_module("backing",
    start_backing,
    verify_backing_complete,
    stop_backing,
    calibrate_chute);

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
  
  // chute pins 
  pinMode(CHUTE_BEAM_BREAK_PIN, INPUT_PULLUP);
  pinMode(CHUTE_SPEED_PIN, OUTPUT);
  pinMode(CHUTE_INVERT_PIN, OUTPUT);

  // backing pins
  pinMode(BACKING_SPEED_PIN, OUTPUT);
  pinMode(BACKING_INVERT_PIN, OUTPUT);

  // label tamper pins
  // pinMode(TAMPER_NEAR_SWITCH_PIN, INPUT_PULLUP) ;
  // pinMode(TAMPER_FAR_SWITCH_PIN, INPUT_PULLUP) ;
  // pinMode(TAMPER_SPEED_PIN,OUTPUT) ;
  // pinMode(TAMPER_INVERT_PIN, OUTPUT) ;

  // box conveyor pins
  pinMode(FRONT_BEAM_BREAK_PIN, INPUT_PULLUP);
  pinMode(BACK_BEAM_BREAK_PIN, INPUT_PULLUP);
  pinMode(BOX_CONVEYOR_SPEED_PIN, OUTPUT);
  pinMode(BOX_CONVEYOR_INVERT_PIN, OUTPUT);

  // TODO:BACKING
  // if (! vl.begin()) {
  //   logerr("*** Failed to find VL6180X (backing Rangefinder) sensor");
  // } else loginfo("VL6180X (backing Rangefinder) Sensor found!");

  loginfo("setup() Complete");
}

void loop() {
  periodic_status();
  nh.spinOnce();
  check_chute();
  check_backing();
  // check_tamper();
  check_box_conveyor();
  // loginfo("System is running");
  // delay(1000);
  delay(200);
}

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

//           stop_chute();
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
