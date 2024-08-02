#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "mariotune.h"

// Define states for the robot
#define STATE_DRIVE_FORWARDS 0
#define STATE_FOLLOW_LINE 1
#define STATE_ANGLE_TURN 2
#define STATE_TURN_AROUND 3
#define STATE_RETURN_HOME 4
#define STATE_INTERSECTION_TURN 5
#define STATE_Diagnose 6


// Define constants
#define LED_PIN 13  // Pin to activate the orange LED
#define buzzerpin = 6
#define leave_box 2200 // Time to leave initial box
#define join_TimeLine 8000 // Time limit to find line at start (includes leave_box)
#define return_TimeHome 27000 // Time before beginning return home state

// Coordinates to start Home() for Run 1 
#define X_End_1 890
#define Y_End_1 -550
// Coordinates to start Home() for  Run 2
#define X_End_2 750
#define Y_End_2 -890

// Global variables for tracking states and times
float time_update; // Timestamp for updating kinematics and PIDs
float Monitor_ts; // Timestamp for tracking time before return home state
float pwm_l; // Global variables for updatePIDs()
float pwm_r;
float pwm_kin_r;
float pwm_kin_l;
float ave_encoder_l_spd; // Velocity estimate for left wheel
float ave_encoder_r_spd; // Velocity estimate for right wheel
float turn;
float fwd_bias = 0.4;// Kinematics (header) demand
float r_demand;
float l_demand; // Left and right wheel demand (header linefollowing)
float theta_demand;

float theta_end; // Theta facing opposite direction
float theta_end_final; // Theta facing home direction
float x_end;
float y_end;


// Constants for angle calculations
float angle_pi = 3.14159265;
float angle_half_pi = 1.57079633;
float angle_pi_over4 = 0.785398163397;


// State and check variables
int state, check, LineBoxCheck, HomeCheck;
bool TurnCheck, TimeOut,DebugCheck, ThetaCheck, EndCheck;
float NewHomeAngle, theta_practice;

// Class instances
Motors_c motors;
LineSensor_c line_sensor;
Kinematics_c kinematics;
PID_c spd_pid_right,spd_pid_left, heading_line, heading_kin;


void setup() {
  state = STATE_DRIVE_FORWARDS;
  // Reset checks
  TurnCheck = DebugCheck = TimeOut = ThetaCheck = EndCheck = false;
  HomeCheck = LineBoxCheck = NewHomeAngle = check = 0;

  //  Initialising classes
  motors.initialise();
  kinematics.initialise();

  // PID pwm controllers: for 4th argument (0 = left, 1 = right)
  spd_pid_left.initialise(60, 0.0525 , 0, 0);  // Tuned for fwd_bias around 0.4 (100, 0.013, -150)
  spd_pid_right.initialise(60, 0.0525, 0, 1);

  // Header controllers for line following and kinematics 
  heading_line.initialise(0.15 , 0.00014  , 0 , 0);
  heading_kin.initialise(0.75, 0, 0, 0);

  //Setup encoders
  setupEncoderLeft();
  setupEncoderRight();

  time_update = millis();
  Monitor_ts = millis();

  ResetPIDs();

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

}


void loop() {
  // Monitor the current time
  Monitor_ts = millis();

  // Update the state and kinematics
  updateState();
  kinematics.update();

  // Handle different states using a switch-case structure
  switch (state) {
    case STATE_DRIVE_FORWARDS:
      Serial.println("State: Drive Forwards");
      driveForwards();
      // playMarioTheme(6);
      break;

    case STATE_ANGLE_TURN:
      Serial.println("State: Angle Turn");
      AngleTurnFunc();
      // playMarioJump(6);
      break;

    case STATE_TURN_AROUND:
      Serial.println("State: Turning 180 Degrees");
      TurnFunc();
      // playMarioJump(6);
      break;

    case STATE_FOLLOW_LINE:
      Serial.println("State: Follow Line");
      // VelPID(1);
      line_sensor.LineFollowingBehaviour();
      break;

    case STATE_RETURN_HOME:
      Serial.println("State: Return Home");
      ReturnHome();
      // playMarioWin(6);
      break;

    case STATE_INTERSECTION_TURN:
      Serial.println("State: Intersection Turn");
      InterTurn();
      // playMarioJump(6);
      break;

    case STATE_Diagnose:
      Serial.println("State: Diagnose");
      //signalError();
      break;

    default:
      Serial.println("State: Unknown");
      break;
  }
}


void updateState() {
  switch (state) {
    case STATE_DRIVE_FORWARDS:
      if ((LineBoxCheck == 2) && (line_sensor.linesensor_state == true)) {
        line_sensor.L_AngleTurn = true;
        state = STATE_ANGLE_TURN; // Or STATE_FOLLOW_LINE;
        ResetPIDs();
      } else if ((LineBoxCheck < 2) && (TimeOut == false)) {
        // Do nothing, continue driving forwards
      } else {
        Serial.println("Check 3");
        state = STATE_Diagnose;
      }
      break;

    case STATE_FOLLOW_LINE:
      kinematics.update();
      if ((kinematics.X_I_Global >= X_End_1 && kinematics.Y_I_Global >= Y_End_1) ||
          (kinematics.X_I_Global >= X_End_2 && kinematics.Y_I_Global <= Y_End_2)) {
        EndCheck = true;
      }

      if (!EndCheck) {
        line_sensor.readLineSensor();
        if ((line_sensor.linesensor_state == true) && (line_sensor.L_AngleTurn == false)) {
          // Continue following the line
        } else if (line_sensor.Turn == true) {
          Serial.println("Follow line -> Turn Around.");
          state = STATE_TURN_AROUND;
          TurnCheck = false;
        } else if ((line_sensor.L_AngleTurn == true || line_sensor.R_AngleTurn == true) && !line_sensor.Intersection_Check) {
          Serial.println("Follow line -> AngleTurn");
          state = STATE_ANGLE_TURN;
          check = 0;
          TurnCheck = false;
        } else if (line_sensor.Intersection_Check == true) {
          state = STATE_INTERSECTION_TURN;
        } else {
          state = STATE_Diagnose;
        }
      } else {
        line_sensor.readLineSensor();
        if (!line_sensor.linesensor_state) {
          motors.intital_MotorPower(0, 0);
          Serial.println("Follow Line -> Home");
          state = STATE_RETURN_HOME;
        } else if (line_sensor.linesensor_state) {
          // Continue following the line
        } else {
          state = STATE_Diagnose;
        }
      }
      break;

    case STATE_TURN_AROUND:
      if (!line_sensor.Turn && TurnCheck) {
        state = STATE_FOLLOW_LINE;
        Serial.println("Turn Around -> Follow Line");
        TurnCheck = false;
        ResetPIDs();
      } else if (line_sensor.Turn) {
        // Keep turning
      } else {
        state = STATE_Diagnose;
      }
      break;

    case STATE_ANGLE_TURN:
      if ((!line_sensor.L_AngleTurn || !line_sensor.R_AngleTurn) && TurnCheck) {
        Serial.println("AngleTurn -> Follow Line");
        state = STATE_FOLLOW_LINE;
        TurnCheck = false;
        ResetPIDs();
      } else if (line_sensor.L_AngleTurn || line_sensor.R_AngleTurn) {
        // Continue angle turn
      } else {
        state = STATE_Diagnose;
      }
      break;

    case STATE_INTERSECTION_TURN:
      if (!line_sensor.Intersection_Check) {
        state = STATE_FOLLOW_LINE;
      } else if (line_sensor.Intersection_Check) {
        // Continue intersection turn
      }
      break;

    case STATE_RETURN_HOME:
      break;

    case STATE_Diagnose:
      break;

    default:
      Serial.println("State: Unknown");
      state = STATE_Diagnose;
      break;
  }
}


void driveForwards() {
  line_sensor.readLineSensor();
  unsigned long forward_ts = millis();

  // Play buzzer noise in the background
  //playMarioTheme(6);

  if (LineBoxCheck < 1) {
    if (forward_ts <= leave_box) {
      motors.intital_MotorPower(20, 20); // Could change for PID
    } else if (forward_ts > leave_box) {
      LineBoxCheck++;
    }
  } else if (LineBoxCheck == 1) {
    line_sensor.readLineSensor();
    if (line_sensor.line_num[0] < DN1_threshold_sensor && line_sensor.line_num[4] < DN5_threshold_sensor) {
      if (forward_ts <= join_TimeLine) {
        motors.intital_MotorPower(20, 20);
      } else if (forward_ts > join_TimeLine) {
        TimeOut = true;
      }
    } else if (line_sensor.line_num[0] >= DN1_threshold_sensor && line_sensor.line_num[4] >= DN5_threshold_sensor) {
      LineBoxCheck++;
      motors.intital_MotorPower(0, 0);
    }
  }
}


void TurnFunc() {
  // Read the line sensor values
  line_sensor.readLineSensor();
  
  if (line_sensor.Turn == true) {
    // Initiate turning motion
    motors.intital_MotorPower(20, -20);
    
    // Read line sensor values during turn
    line_sensor.readLineSensor();
    
    // Check if the turn is complete based on the line sensor values
    if (line_sensor.line_num[0] >= DN1_threshold_sensor) {
      motors.intital_MotorPower(0, 0); // Stop the motors
      TurnCheck = true; // Mark turn as completed
      line_sensor.Turn = false; // Reset turn flag
    }
  } else if (line_sensor.Turn == false) {
    // Do nothing if turn is not required
  }
}



void AngleTurnFunc() {
  // Handle left angle turn
  if (line_sensor.L_AngleTurn == true) {
    motors.intital_MotorPower(-20, 20);
    line_sensor.readLineSensor();

    if (line_sensor.line_num[0] <= DN1_threshold_sensor) {
      motors.intital_MotorPower(0, 0); // Stop the motors
      Serial.println("Turned Left correctly");
      line_sensor.L_AngleTurn = false; // Reset left turn flag
      TurnCheck = true; // Mark turn as completed
    }
  } 
  // Handle right angle turn
  else if (line_sensor.R_AngleTurn == true) {
    if (check == 0) {
      motors.intital_MotorPower(20, -20);
      line_sensor.readLineSensor();

      if (line_sensor.line_num[3] >= DN4_threshold_sensor) {
        check++; // Move to the next step of the right turn
      }
    } else if (check == 1) {
      motors.intital_MotorPower(20, -20);
      line_sensor.readLineSensor();

      if (line_sensor.line_num[2] >= DN3_threshold_sensor) {
        motors.intital_MotorPower(0, 0); // Stop the motors
        TurnCheck = true; // Mark turn as completed
        line_sensor.R_AngleTurn = false; // Reset right turn flag
      }
    }
  } 
  // Handle no angle turn
  else if (!line_sensor.L_AngleTurn && !line_sensor.R_AngleTurn) {
    // Do nothing if no turn is required
  }
}


void InterTurn() {
  // Handle intersection turn
  if (line_sensor.Intersection_Check == true) {
    motors.intital_MotorPower(-20, 20);
    line_sensor.readLineSensor();

    if (line_sensor.line_num[2] >= DN3_threshold_sensor) {
      motors.intital_MotorPower(0, 0); // Stop the motors
      line_sensor.Intersection_Check = false; // Reset intersection check flag
    }
  } 
  // Handle no intersection turn
  else if (line_sensor.Intersection_Check == false) {
    // Do nothing if no intersection turn is required
  }
}



void ReturnHome() {
  // Initial state: Setup initial motor power and calculate theta demand
  if (HomeCheck == 0) {
    motors.intital_MotorPower(0, 0);
    kinematics.update();
    theta_demand = atan2(kinematics.Y_I_Global, kinematics.X_I_Global);
    theta_end = kinematics.theta_I_Global;
    HomeCheck++;
    return;
  }

  // State 1: Rotate to align with theta_demand
  if (HomeCheck == 1) {
    if (theta_end < theta_demand) {
      motors.intital_MotorPower(18, -18);
      kinematics.update();
      if (kinematics.theta_I_Global >= theta_demand) {
        motors.intital_MotorPower(0, 0);
        HomeCheck++;
      }
    } else if (theta_end > theta_demand) {
      motors.intital_MotorPower(-18, 18);
      kinematics.update();
      if (kinematics.theta_I_Global <= theta_demand) {
        motors.intital_MotorPower(0, 0);
        HomeCheck++;
      }
    }
    return;
  }

  // State 2: Update kinematics and calculate new home angle
  if (HomeCheck == 2) {
    kinematics.update();
    NewHomeAngle = kinematics.theta_I_Global + angle_pi;
    HomeCheck++;
    return;
  }

  // State 3: Rotate to NewHomeAngle
  if (HomeCheck == 3) {
    Serial.print("NewHomeAngle:");
    Serial.println(NewHomeAngle);
    
    if (kinematics.theta_I_Global < NewHomeAngle) {
      motors.intital_MotorPower(18, -18);
      kinematics.update();
      if (kinematics.theta_I_Global >= NewHomeAngle) {
        motors.intital_MotorPower(0, 0);
        kinematics.update();
        x_end = kinematics.X_I_Global;
        y_end = kinematics.Y_I_Global;
        theta_end_final = kinematics.theta_I_Global;

        HomeCheck++;
        ResetPIDs();
      }
    }
    return;
  }

  // State 4: Move to origin (0, 0) using PID control
  if (HomeCheck == 4) {
    VelPID(0, 0);
    kinematics.update();

    bool reachedX = (kinematics.X_I_Global <= 0);
    bool reachedY = (kinematics.Y_I_Global <= 0 && y_end > 0) || (kinematics.Y_I_Global >= 0 && y_end < 0);
    
    if (reachedX && reachedY) {
      motors.intital_MotorPower(0, 0);
      HomeCheck++;
    }
    return;
  }

  // State 5: Indicate that home position is found
  if (HomeCheck == 5) {
    Serial.println("Found Home !!");
  }
}





void ResetPIDs() {
  spd_pid_left.reset();
  spd_pid_right.reset();
  heading_line.reset();
}

// First argument: 0 means drive forwards, 1 means line following, 2 means following theta demand of 0.
// Second Argument: Theta demand for attempted PID
void VelPID(int num, int num_2) {
  // Read the line sensor values
  line_sensor.readLineSensor();

  // Estimate the average encoder speeds
  ave_encoder_l_spd = spd_pid_left.spd_est();
  ave_encoder_r_spd = spd_pid_right.spd_est();

  // Calculate the elapsed time since the last update
  float loop_elapsed_ts = millis() - time_update;

  if (loop_elapsed_ts > 20) {
    // Update kinematics
    kinematics.update();

    if (num == 0) {  // Driving in a straight line
      r_demand = fwd_bias;
      l_demand = fwd_bias;
      
      pwm_l = spd_pid_left.update(l_demand, ave_encoder_l_spd);
      pwm_r = spd_pid_right.update(r_demand, ave_encoder_r_spd);
    } 
    else if (num == 1) {  // Line Following PID
      line_sensor.readLineSensor();
      turn = heading_line.update(0, line_sensor.weightedMeasurement()); // Line heading
      r_demand = fwd_bias - turn;
      l_demand = fwd_bias + turn;
      
      pwm_l = spd_pid_left.update(l_demand, ave_encoder_l_spd);
      pwm_r = spd_pid_right.update(r_demand, ave_encoder_r_spd);
    } 
    else if (num == 2) { // Attempted PID with theta demand heading
      kinematics.update();
      float theta_heading = heading_kin.update(num_2, kinematics.theta_I_Global);
      float theta_right = fwd_bias - theta_heading;
      float theta_left = fwd_bias + theta_heading;
      pwm_kin_l = spd_pid_left.update(theta_left, ave_encoder_l_spd);
      pwm_kin_r = spd_pid_right.update(theta_right, ave_encoder_r_spd);
    }

    // Update the time for the next loop
    time_update = millis();
  }

  // Set motor power based on the control mode
  if (num == 0 || num == 1) {
    motors.intital_MotorPower(pwm_l, pwm_r);
  } else if (num == 2) {
    motors.intital_MotorPower(pwm_kin_l, pwm_kin_r);
  }
}


// void signalError() {
//   motors.intital_MotorPower(0, 0); // Stop the motors

//   if (TimeOut) {
//     Serial.println("Time Out has occurred!");
//   }

//   if (line_sensor.Turn) {
//     Serial.println("Problem with turning 180 degrees CW");
//   }

//   if (line_sensor.L_AngleTurn) {
//     Serial.println("L_AngleTurn is still on");
//   }

//   if (TurnCheck) {
//     Serial.println("TurnCheck is still true");
//   }

//   if (HomeCheck > 0) {
//     Serial.println("Problem with home state");
//   } else {
//     Serial.println("Unknown state failure");
//   }

//   // Print the current state for debugging
//   Serial.println("State:");
//   Serial.println(state);
// }
