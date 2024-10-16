/* ////////////// WIRING NOTES ///////////////////////
* Ensure all five or six grounds are connected:
*   - arduino ground
*   - 2 x potentiometer ground (black wires)
*   - PDP ground
*   - 1 or 2 H-bridges (2 if connecting all 3 motors)
*
* Make sure all four 5V lines are connected
*   - Arduino 5V output
*   - 1 OR 2 H-bridge inputS (take off jumper behind OUT1 and OUT2!!!)
*   - 2 x potentiometer 5V (red wires)
*
* All other pin wirings in the pin definitions
*/


/*
serial input map
input     outcome
0         stop all actuators
1         extend two big actuators (1 and 2)
2         retract two big actuators
3         extend small actuator (3)
4         retract small actuator
*/

/* Debugging */
#define MAIN_DELAY 0
#define PRINT 0
#define DEBUG_MSGS 0
#define SKIP_STARTUP 0

/* min/max voltage values for mapping */
// LFT = lifting actuator
// TLT = tilting actuator
#define MIN_TLT 37 // originally 52
#define MAX_TLT 215 // originally 215
#define MIN_LFT 47 // originally 36
#define MAX_LFT 344 // originally 340


/* For syncing of actuators */
#define DIST_MAP_MAX 100  // higher = more precise mapping = more strict synchronization

/* Pin definitions
* P = motor positive, M = motor negative,
* S = string pot ADC
*/
#define P1 5
#define M1 6
#define P2 10
#define M2 11
#define P3 12
#define M3 13
#define S_TLT A0
#define S_LFT A1

#define NC 15

int v_tlt, v_lft;        // pot readings
int v_tlt_targ, v_lft_targ;
int pos_tlt, pos_lft;  // positions, mapped from voltages
int pos_tlt_targ, pos_lft_targ;
int wait_for_position_flag; // set to 1 when waiting for specific position
int wait_for_voltage_flag; // set to 1 when waiting for specific voltages
int done_tlt, done_lft; // used to indicate whether waiting is completed or not
int interruption_flag; // indicates when waiting is interrupted, which cancels dumping sequence

/* Keeps track of driving direction of motors */
int direction_tlt;
int direction_lft;

/* input for reading from serial */
char input;

/* Stage vars
* Each stage is 10% of full range
* "Checkpoints" at all even stages */ 
int stage;
int targ_stage;

void getPositions() {
  v_tlt = analogRead(S_TLT);
  v_lft = analogRead(S_LFT);
  pos_tlt = map(v_tlt, MIN_TLT, MAX_TLT, 0, DIST_MAP_MAX);
  pos_lft = map(v_lft, MIN_LFT, MAX_LFT, 0, DIST_MAP_MAX);
}

void printPositions() {
  getPositions();
  Serial.print(v_tlt);
  Serial.print("\t");
  Serial.print(v_lft);
  Serial.print("\t");
  // Serial.print(v3);
  // Serial.print("\t");
  Serial.print(pos_tlt);
  Serial.print("\t");
  Serial.print(pos_lft);
  Serial.print("\t");
  Serial.print("\n");
}

// sets each actuator individually
// 1 -> set this actuator forward
// -1 -> set this actuator backwards
// 0 -> stop this actuator
// NC -> do not change the direction of this actuator
void set_actuators(int m1, int m2, int m3) {
  switch (m1) {
    case 1:
      digitalWrite(P1, HIGH);
      digitalWrite(M1, LOW);
      break;
    case 0:
      digitalWrite(P1, LOW);
      digitalWrite(M1, LOW);
      break;
    case -1:
      digitalWrite(P1, LOW);
      digitalWrite(M1, HIGH);
      break;
    case NC:
      break;
    default:
      break;
  }

  switch (m2) {
    case 1:


      //////////////////////////////////////////// watch ////////////////////////////////////////////////////
      // analogWrite(P2, 255);
      digitalWrite(P2, HIGH);




      digitalWrite(M2, LOW);
      break;
    case 0:
      digitalWrite(P2, LOW);
      digitalWrite(M2, LOW);
      break;
    case -1:
      digitalWrite(P2, LOW);
      digitalWrite(M2, HIGH);
      break;
    case NC:
      break;
    default:
      break;
  }

  switch (m3) {
    case 1:
      digitalWrite(P3, HIGH);
      digitalWrite(M3, LOW);
      break;
    case 0:
      digitalWrite(P3, LOW);
      digitalWrite(M3, LOW);
      break;
    case -1:
      digitalWrite(P3, LOW);
      digitalWrite(M3, HIGH);
      break;
    case NC:
      break;
    default:
      break;
  }
}

// sets all target voltages / positions to be the current values so that movement stops
void resetAllTargs() {
  getPositions();
  v_tlt_targ = v_tlt;
  v_lft_targ = v_lft;
  pos_tlt_targ = pos_tlt;
  pos_lft_targ = pos_lft;
  done_tlt = 1;
  done_lft = 1;
  wait_for_voltage_flag = 0;
  wait_for_position_flag = 0;
}

// 0 for tilting, 1 for lifting
void resetTarg(int m) {
  getPositions();
  if (m==0) {
    v_tlt_targ = v_tlt;
    pos_tlt_targ = pos_tlt;
    done_tlt = 1;
  }
  if (m==1) {
    v_lft_targ = v_lft;
    pos_lft_targ = pos_lft;
    done_lft = 1;
  }
}

// using flags, decides which targ values to look at
// sets direction and motion of actuators to move towards targ values
void setMotionFromTargs() {
  getPositions();
  if (wait_for_position_flag) {
    if (DEBUG_MSGS) {
      Serial.print("POSITION TARGETS: ");
      Serial.print(pos_tlt_targ);
      Serial.print('\t');
      Serial.print(pos_lft_targ);
      Serial.print('\n');
    }
    if (pos_tlt < pos_tlt_targ) {
      set_actuators(NC,NC,1);
      direction_tlt = 1;
      done_tlt = 0;
      if (DEBUG_MSGS) {
        Serial.print("Tilt fwd\n");
      }
    } else if (pos_tlt > pos_tlt_targ) {
      set_actuators(NC,NC,-1);
      direction_tlt = -1;
      done_tlt = 0;
      if (DEBUG_MSGS) {
        Serial.print("Tilt bkwd\n");
      }
    }
    if (pos_lft < pos_lft_targ) {
      set_actuators(1,1,NC);
      direction_lft = 1;
      done_lft = 0;
      if (DEBUG_MSGS) {
        Serial.print("Lift fwd\n");
      }
    } else if (pos_lft > pos_lft_targ) {
      set_actuators(-1,-1,NC);
      direction_lft = -1;
      done_lft = 0;
      if (DEBUG_MSGS) {
        Serial.print("Lift bkwd\n");
      }
    }
  }

  if (wait_for_voltage_flag) {
    if (DEBUG_MSGS) {
      Serial.print("VOLTAGE TARGETS: ");
      Serial.print(v_tlt_targ);
      Serial.print('\t');
      Serial.print(v_lft_targ);
      Serial.print('\n');
    }
    if (v_tlt < v_tlt_targ) {
      set_actuators(NC,NC,1);
      direction_tlt = 1;
      done_tlt = 0;
      if (DEBUG_MSGS) {
        Serial.print("Tilt fwd\n");
      }
    } else if (v_tlt > v_tlt_targ) {
      set_actuators(NC,NC,-1);
      direction_tlt = -1;
      done_tlt = 0;
      if (DEBUG_MSGS) {
        Serial.print("Tilt bkwd\n");
      }
    }
    if (v_lft < v_lft_targ) {
      set_actuators(1,1,NC);
      direction_lft = 1;
      done_lft = 0;
      if (DEBUG_MSGS) {
        Serial.print("Lift fwd\n");
      }
    } else if (v_lft > v_lft_targ) {
      set_actuators(-1,-1,NC);
      direction_lft = -1;
      done_lft = 0;
      if (DEBUG_MSGS) {
        Serial.print("Lift bkwd\n");
      }
    }
  }
}

void processSerial() {
  if (Serial.available() > 0) {
    input = Serial.read();
    // Serial.read() returns the first byte of incoming serial data available (or -1 if no data is available). Data type: int.

    if (input == '0') {
      set_actuators(0,0,0);
      direction_lft = 0;
      direction_tlt = 0;
      interruption_flag = 1;
      resetAllTargs();

    } else if (input == '1') {
      set_actuators(1,1,NC);
      direction_lft = 1;
      if(done_tlt==0 || done_lft==0) {
        interruption_flag = 1;
      }
      resetTarg(1);

    } else if (input == '2') {
      set_actuators(-1,-1,NC);
      direction_lft = -1;
      if(done_tlt==0 || done_lft==0) {
        interruption_flag = 1;
      }
      resetTarg(1);

    } else if (input == '3') {
      set_actuators(NC,NC,1);
      direction_tlt = 1;
      if(done_tlt==0 || done_lft==0) {
        interruption_flag = 1;
      }
      resetTarg(0);

    } else if (input == '4') {
      set_actuators(NC,NC,-1);
      direction_tlt = -1;
      if(done_tlt==0 || done_lft==0) {
        interruption_flag = 1;
      }
      resetTarg(0);

    } else if (input == '5') { // reservoir: 112 123
      // 66 94
      wait_for_position_flag = 0;
      wait_for_voltage_flag = 1;
      v_tlt_targ = 112;
      v_lft_targ = 123;
      setMotionFromTargs();
      if(done_tlt==0 || done_lft==0) {
        interruption_flag = 1;
      }

    } else if (input == '6') { // MAIN SCOOP: 162 162
      // used to be scoop 1: 37min 270?
      // 162 152
      wait_for_position_flag = 0;
      wait_for_voltage_flag = 1;


      //////// MAIN SCOOPING THRESHOLDS //////////////
      v_tlt_targ = 162;
      v_lft_targ = 161;



      setMotionFromTargs();
      if(done_tlt==0 || done_lft==0) {
        interruption_flag = 1;
      }

    } else if (input == '7') { // scoop 2: 37min 251
      // wait_for_position_flag = 0;
      // wait_for_voltage_flag = 1;
      // v_tlt_targ = 37;
      // v_lft_targ = 251;
      // setMotionFromTargs();
      // if(done_tlt==0 || done_lft==0) {
      //   interruption_flag = 1;
      // }

    } else if (input == '8') { // scoop 3: 65 219
      // wait_for_position_flag = 0;
      // wait_for_voltage_flag = 1;
      // v_tlt_targ = 65;
      // v_lft_targ = 219;
      // setMotionFromTargs();
      // if(done_tlt==0 || done_lft==0) {
      //   interruption_flag = 1;
      // }

    } else if (input == '9') { // dumping sequence, ultimate targs: 217max 47min
      // extend titlting to at least position 60, then start to retract lifting
      interruption_flag = 0;
      if (v_tlt < 146) {
        v_tlt_targ = 146;
        resetTarg(1);
        wait_for_position_flag = 0;
        wait_for_voltage_flag = 1;
        setMotionFromTargs();
        waitForVoltage();
        if (interruption_flag) return;
      }
      // targs: max min
      v_tlt_targ = 205;
      v_lft_targ = 65;
      wait_for_voltage_flag = 1;
      wait_for_position_flag = 0;
      setMotionFromTargs();
      waitForVoltage();
      // if (interruption_flag) return;
      // // lifting jerk
      // set_actuators(1,1,0);
      // delay(250);
      // set_actuators(-1,-1,0);
      // delay(250);
      // set_actuators(1,1,0);
      // delay(250);
      // set_actuators(-1,-1,0);
      // delay(250);
      // // tilting jerk
      // set_actuators(0,0,-1);
      // delay(250);
      // set_actuators(0,0,1);
      // delay(250);
      // set_actuators(0,0,-1);
      // delay(250);
      // set_actuators(0,0,1);
      // delay(250);
      // // lifting jerk
      // set_actuators(1,1,0);
      // delay(250);
      // set_actuators(-1,-1,0);
      // delay(250);
      // set_actuators(1,1,0);
      // delay(250);
      // set_actuators(-1,-1,0);
      // delay(250);
      // // tilting jerk
      // set_actuators(0,0,-1);
      // delay(250);
      // set_actuators(0,0,1);
      // delay(250);
      // set_actuators(0,0,-1);
      // delay(250);
      // set_actuators(0,0,1);
      // delay(250);
      // set_actuators(0,0,0);
    } 
  }
}

void waitForVoltage() {
  if (DEBUG_MSGS) {
    Serial.print("WaitForVoltage()....\n");
  }
  done_tlt = 0;
  done_lft = 0;
  while(1) {
    processSerial();
    if (wait_for_voltage_flag == 0 || wait_for_position_flag == 1) {
      wait_for_voltage_flag = 0;
      return;
    }
    getPositions();

    if (direction_tlt == 0) done_tlt = 1;
    if (direction_lft == 0) done_lft = 1;

    // CHECK TILTING
    if (done_tlt == 0 &&
        ( 
          (direction_tlt == 1 && v_tlt >= v_tlt_targ) ||
          (direction_tlt == -1 && v_tlt <= v_tlt_targ) 
        )
      ) {
      set_actuators(NC,NC,0);
      direction_tlt = 0;
      done_tlt = 1;
    }

    // CHECK LIFTING
    if (done_lft == 0 && 
        ( 
          (direction_lft == 1 && v_lft >= v_lft_targ) ||
          (direction_lft == -1 && v_lft <= v_lft_targ) 
        )
      ) {
      set_actuators(0,0,NC);
      direction_lft = 0;
      done_lft = 1;
    }

    
    if (done_tlt && done_lft) {
      resetAllTargs();
      return;
    }
  } // end of while(1)
}

int waitForPosition() {
  if (DEBUG_MSGS) {
    Serial.print("WaitForPosition()....\n");
  }
  done_tlt = 0;
  done_lft = 0;
  while(1) {
    processSerial();
    if (wait_for_position_flag == 0 || wait_for_voltage_flag == 1) {
      wait_for_position_flag = 0;
      return 1;
    }
    getPositions();

    if (direction_tlt == 0) done_tlt = 1;
    if (direction_lft == 0) done_lft = 1;

    // CHECK TILTING
    if (done_tlt == 0 &&
        ( 
          (direction_tlt == 1 && pos_tlt >= pos_tlt_targ) ||
          (direction_tlt == -1 && pos_tlt <= pos_tlt_targ) 
        )
      ) {
      set_actuators(NC,NC,0);
      direction_tlt = 0;
      done_tlt = 1;
    }

    // CHECK LIFTING
    if (done_lft == 0 && 
        ( 
          (direction_lft == 1 && pos_lft >= pos_lft_targ) ||
          (direction_lft == -1 && pos_lft <= pos_lft_targ) 
        )
      ) {
      set_actuators(0,0,NC);
      direction_lft = 0;
      done_lft = 1;
    }

    
    if (done_tlt && done_lft) {
      resetAllTargs();
      return;
    }
  } // end of while(1)
}



void setup() {
  // put your setup code here, to run once:
  pinMode(P1, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(P2, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(P3, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(S_TLT, INPUT);
  pinMode(S_LFT, INPUT);

  Serial.begin(9600);

  digitalWrite(P1, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(P2, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(P3, LOW);
  digitalWrite(M3, LOW);

  v_tlt = -1;
  v_lft = -1;

  pos_tlt = -1;
  pos_lft = -1;

  getPositions();
  resetAllTargs();
  interruption_flag = 0;

  input = 0;

  // go to reservoir position
  if (SKIP_STARTUP == 0) {
    wait_for_position_flag = 0;
    wait_for_voltage_flag = 1;
    v_tlt_targ = 112;
    v_lft_targ = 123;
    setMotionFromTargs();
  }
}




void loop() {
  delay(MAIN_DELAY);

  if (PRINT) printPositions();

  processSerial();
  if (wait_for_position_flag) waitForPosition();
  if (wait_for_voltage_flag) waitForVoltage();
}