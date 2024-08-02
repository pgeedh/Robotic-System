// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#include "encoders.h"
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Class to track robot position.
class Kinematics_c {
  public:

  // Constants
    static constexpr float rValue = 16.0;             // Radius value 
    static constexpr float encToMm = 0.2805776302;    // Encoder counts to millimeters conversion factor
    static constexpr float radToDeg = 57.2957795;     // Radians to degrees conversion factor
    static constexpr float lValue = 41.0;  

// Variables to store old encoder counts
    long count_Encoder_left_Old = 0;   // Previous left encoder count
    long count_Encoder_right_Old = 0;   // Previous right encoder count

// Variables for global coordinates and orientation
    float X_I_Global = 0.0;      // Global X coordinate
    float Y_I_Global = 0.0;      // Global Y coordinate
    float theta_I_Global = 0.0;  // Global orientation (theta)

// Variables to store old global coordinates and orientation
    float X_I_GlobalOld = 0.0f;   // Previous global X coordinate
    float Y_I_GlobalOld = 0.0f;   // Previous global Y coordinate
    float theta_I_GlobalOld = 0.0f; // Previous global orientation (theta)

// Variables for relative movements
    float x_rel = 0.0;    // Relative X movement
    float theta_rel = 0.0; // Relative orientation change (theta)

    

    // Constructor, must exist.
    Kinematics_c() {
    }

    void initialise() {
      X_I_Global = 0;
      Y_I_Global = 0;
      X_I_GlobalOld = 0;
      Y_I_GlobalOld = 0;
      theta_I_Global = 0;
      theta_I_GlobalOld = 0;
      x_rel = 0;
      theta_rel = 0;
    }


    // Updates kinematics - if PID works for curve need to use ICR else use this simplified model below.
    void update() { // update with equation for global
      long dec_el = (count_el - count_Encoder_left_Old); // measures change in encoder values
      long dec_er = (count_er - count_Encoder_right_Old);
      float dec_el_flt = (float)(dec_el);
      float dec_er_flt = (float)(dec_er);

      count_Encoder_left_Old = count_el; // adds last updated values to old encoder counts
      count_Encoder_right_Old = count_er;
      // if  ((dec_el_flt > 0 &&  dec_er_flt > 0) || ( dec_el_flt < 0 && dec_er_flt < 0)) {}
      x_rel =  (((dec_el_flt * encToMm) / 2.0) + ((dec_er_flt * encToMm) / 2.0)) ;
      X_I_Global = X_I_GlobalOld + (x_rel * cos(theta_I_GlobalOld));
      Y_I_Global = Y_I_GlobalOld + (x_rel* sin(theta_I_GlobalOld));


      // if (dec_el_flt != dec_er_flt) {}
      theta_rel = ((dec_el_flt * encToMm) / (2.0 * lValue)) - ((dec_er_flt * encToMm) / (2.0 * lValue));
      theta_rel= theta_rel; // In radians
      theta_I_Global = theta_I_GlobalOld + theta_rel;

      X_I_GlobalOld = X_I_Global;
      Y_I_GlobalOld = Y_I_Global;
      theta_I_GlobalOld = theta_I_Global;
    }
};



#endif
