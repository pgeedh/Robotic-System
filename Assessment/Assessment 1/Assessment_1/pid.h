// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H
#include "encoders.h"


// Class to contain generic PID algorithm.
class PID_c {
  public:

    // Constructor, must exist.
    PID_c() {
    }
    
    float p_term;
    float i_term;
    float i_sum;
    float d_term;
    float feedback;
    float error;
    float error_old;
    float ave_spd;

    float kp;
    float ki;
    float kd;

    float update_old_ts; // Used for updating PWM ts left wheel
    float ms_last_ts; // Used for update function.
    float spd_est_ts;

    long count_el_old;
    long count_er_old;

    int wheel_choice;

    void initialise(float p_gain, float i_gain, float d_gain, int wheel) {
      p_term = 0;
      i_term = 0;
      i_sum = 0;
      d_term = 0;
      feedback = 0;
      error = 0;
      error_old = 0;
      kp = p_gain;
      ki = i_gain;
      kd = d_gain;
      wheel_choice = wheel;
      //      update_old_ts_l = millis(); // used in main loop
      //      update_old_ts_r = millis();
      ms_last_ts = millis();
      

    }

    void reset() {
      p_term = 0;
      i_term = 0;
      i_sum = 0;
      d_term = 0;
      feedback = 0;

      ms_last_ts = millis();
    }

    float update( float demand, float measurement) {


      unsigned long ms_now_ts = millis();
      unsigned long ms_dt = ms_now_ts - ms_last_ts;

      ms_last_ts = millis();

      float float_dt = (float)ms_dt;

      if (float_dt == 0) return feedback; // Prevents error when dt = 0

      error = demand - measurement;

      // p term
      p_term = kp * error;
      // i term integration
      i_sum = i_sum + (error * float_dt);

      // i term
      i_term = ki * i_sum;

      // d term
      d_term = kd * ((error - error_old) / float_dt);

      // feedback term
      feedback = p_term + i_term + d_term;

      error_old = error;
      return feedback;
    }

    // Gets a speed estimate: 0 = left wheel, 1 = right wheel
    float spd_est() { 
      float elapsed_spd_est;
      elapsed_spd_est = millis() - spd_est_ts;

      if (elapsed_spd_est > 20) {
        spd_est_ts = millis();
        if (wheel_choice == 0) {
          long count_el_change = count_el - count_el_old;
          float el_speed = count_el_change / elapsed_spd_est;
          ave_spd = (ave_spd * 0.7) + (el_speed * 0.3);
          count_el_old = count_el;
        }
        else if (wheel_choice == 1) {
          long count_er_change = count_er - count_er_old;
          float er_speed = count_er_change / elapsed_spd_est;
          ave_spd = (ave_spd * 0.7) + (er_speed * 0.3);
          count_er_old = count_er;
        }
        else {
          Serial.println("Error with wheel selection");
        }
      }
      return ave_spd;
    }
};


#endif
