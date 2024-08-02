// this #ifndef stops this file
// from being included more than
// once by the compiler.
#ifndef _LINESENSOR_H
#define _LINESENSOR_H
#include "motors.h"

Motors_c motors_line;
class LineSensor_c {

  public:
  //defining sensors
#define EMIT_PIN    11    // IR LED sensor 

/////// line sensors

#define sensor_LEFT_PIN 12   //  DN1 pin
#define sensor_MIDLEFT_PIN 18   //  DN2 pin
#define sensor_MIDDLE_PIN 20   //  DN3 pin
#define sensor_MIDRIGHT_PIN 21   //  DN4 pin
#define sensor_RIGHT_PIN 22   //  DN5 pin


#define SAMPLES 1  // 1 sample per sensor reading

//// threshold for each sensor calculated using test papers

#define DN1_threshold_sensor 1000
#define DN2_threshold_sensor 660
#define DN4_threshold_sensor 650
#define DN3_threshold_sensor 700
#define DN5_threshold_sensor 1000

//// side sensors for turning 
#define DN1_turn_l 750
#define DN1_intersection 800
#define DN5_intersection 800
 ///array
    int ls_pins[5] = {sensor_LEFT_PIN, sensor_MIDLEFT_PIN, sensor_MIDDLE_PIN, sensor_MIDRIGHT_PIN,sensor_RIGHT_PIN };
    int line_num[5] = {0};

// void bangBangLineFollowing() {
//   line_sensor.readLineSensor();
//   if (line_sensor.linesensor_state) {
//     // On the line, go straight
//     motors.intital_MotorPower(50, 50);
//   } else if (line_sensor.line_num[0] > line_sensor.line_num[4]) {
//     // Line is to the left, turn left
//     motors.intital_MotorPower(50, 0);
//   } else {
//     // Line is to the right, turn right
//     motors.intital_MotorPower(0, 50);
//   }
// }

/// line weighted algoritm
    float W_type1;
    float left_PWM = {0}; // Left pwm value after weighting
    float right_PWM = {0}; // Right pwm value after weighting
    float results[ SAMPLES ][5];

    bool linesensor_state;
    bool L_AngleTurn;
    bool R_AngleTurn;
    bool Turn;
    bool Intersection_Check;
    // Constructor, must exist.
    LineSensor_c() {

    }
    void initialise() {
      // Setting some initial pin modes and states
      pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
      pinMode( sensor_LEFT_PIN, INPUT );  // Set line sensors 1-5 pins to input
      pinMode( sensor_MIDLEFT_PIN, INPUT );
      pinMode( sensor_MIDDLE_PIN, INPUT );
      pinMode( sensor_MIDRIGHT_PIN, INPUT );
      pinMode( sensor_RIGHT_PIN, INPUT );

      linesensor_state = false;
      L_AngleTurn = false;
      R_AngleTurn = false;
      Turn = false;
    }

    void readLineSensor() {

      for (int i = 0; i < 5; i++) {

        pinMode( EMIT_PIN, OUTPUT );
        digitalWrite( EMIT_PIN, HIGH );
        pinMode( ls_pins[ i ], OUTPUT ); // changing the specific pins state
        digitalWrite( ls_pins[ i ], HIGH );
        delayMicroseconds( 10 );
        pinMode( ls_pins[ i ], INPUT);
        unsigned long start_t = micros();  

        while ( digitalRead( ls_pins[ i ]) == HIGH ) {
            // getting the reading back untill the light gets back
        }

        unsigned long end_t = micros(); // may need changing
        pinMode( EMIT_PIN, INPUT );

        unsigned long elapsed_time = end_t - start_t;
        line_num[i] = elapsed_time; /// gets the reading 
      }
      
      if (line_num[0] < DN1_threshold_sensor && line_num[2] > DN3_threshold_sensor && line_num[4] > DN5_threshold_sensor) {
        R_AngleTurn = true;
      }

      if ((line_num[0] >= DN1_threshold_sensor) || ((line_num[0] >= DN1_threshold_sensor) && (line_num[4] >= DN5_threshold_sensor))) {
        L_AngleTurn = true;
      }
      else {
        L_AngleTurn = false;
      }
      if ((line_num[0] >= DN1_intersection && line_num[2] < DN3_threshold_sensor) ||
          (line_num[4] >= DN5_intersection && line_num[2] < DN3_threshold_sensor)) {
        Intersection_Check = true;
      }
      if (line_num[0] < DN1_intersection && line_num[2] < DN3_threshold_sensor && 
      line_num[4] < DN5_intersection) {
        Turn = true;
      }
      if (( line_num[1] > DN2_threshold_sensor) || ( line_num[3] > DN4_threshold_sensor)) {
        linesensor_state = true; // Robot is on the line
      }
      else {
        linesensor_state = false; // Robot is off the line.

      }
    }

/// weighted line following algoritm
    float weightedMeasurement() {
      readLineSensor();
      float sum_sensors = line_num[1] + line_num[3]; // sum of sensors DN2 & DN4
      float norm_readings[3] = {0};  // empty array for normalised sensor readings format

      for (int i = 0; i < 3; i += 2) {
        float norm_loop = ((line_num[i + 1]) / sum_sensors);
        norm_readings[i] = norm_loop * 2;
      }
      W_type1 = norm_readings[0] - norm_readings[2];
      return W_type1;
    }

    // Function to follow line while calling line sensing function.
    void LineFollowingBehaviour() {
      int Bias_PWM = 25;  // normally 30
      int MaxTurn_PWM = 30 ; // normally 35

      unsigned long time_loop = millis(); // Constant updated time: Can just use this.
      //  unsigned long elapsed_time = time_loop - time_on_line;
      readLineSensor();

      if (linesensor_state == true) {
        weightedMeasurement();
        left_PWM = (Bias_PWM - (W_type1 * MaxTurn_PWM));
        right_PWM = (Bias_PWM + (W_type1 * MaxTurn_PWM));
        int abs_left_PWM = abs(left_PWM);
        int abs_right_PWM = abs(right_PWM);

        if (abs_left_PWM < Min_PWM && abs_right_PWM < Min_PWM) { 
          motors_line.intital_MotorPower(Min_PWM, Min_PWM);  // Sets robot to keep going, no turning needed
        }
        else if (abs_left_PWM < Min_PWM && abs_right_PWM > Min_PWM) {
          motors_line.intital_MotorPower(0, right_PWM); // Left PWM made to be 0
        }
        else if (abs_left_PWM > Min_PWM && abs_right_PWM < Min_PWM) {
          motors_line.intital_MotorPower(left_PWM, 0); // Right PWM made to be 0
        }
        else {
          Serial.println("Turning within limits");
          motors_line.intital_MotorPower(left_PWM, right_PWM);
        }
      }

      else if (linesensor_state == false) { // No line detected: will change to TurnAround state.
        motors_line.intital_MotorPower(0, 0);
      }
      else {
      }
    }


};



#endif
