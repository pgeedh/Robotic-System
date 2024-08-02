// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

# define Min_PWM 17
# define Max_PWM 80

//Define PINs 
# define Left_PWM_PIN 10
# define Left_DIR_PIN 16
# define Right_PWM_PIN 9
# define Right_DIR_PIN 15
# define FWD LOW
# define REV HIGH

// Class to operate the motor(s).
class Motors_c {
  public:

    // Constructor, must exist.
    Motors_c() {

    }
    // Use this function to
    // initialise the pins and
    // state of your motor(s).
    void initialise() {
      pinMode(Left_PWM_PIN, OUTPUT);
      pinMode(Left_DIR_PIN, OUTPUT);
      pinMode(Right_PWM_PIN, OUTPUT);
      pinMode(Right_DIR_PIN, OUTPUT);

      digitalWrite(Left_PWM_PIN, 0);
      digitalWrite(Right_PWM_PIN, 0);
      digitalWrite(Left_DIR_PIN, FWD);
      digitalWrite(Right_DIR_PIN, FWD);
    }
    void intital_MotorPower( float left_pwm, float right_pwm ) {
      if (left_pwm >= 0) {
        digitalWrite(Left_DIR_PIN, FWD);
      }
      else {
        digitalWrite(Left_DIR_PIN, REV);
      }
      if (right_pwm >= 0) {
        digitalWrite(Right_DIR_PIN, FWD);
      }
      else {
        digitalWrite(Right_DIR_PIN, REV);
      }

      int abs_left = abs(left_pwm);
      int abs_right = abs(right_pwm);

      if ((Min_PWM < abs_left && abs_left < Max_PWM) || (abs_left == 0)) {
        analogWrite(Left_PWM_PIN, abs_left);
      }
      if ((Min_PWM < abs_right && abs_right < Max_PWM) || (abs_right == 0)){
        analogWrite(Right_PWM_PIN, abs_right);
      }

      else {
        analogWrite(Left_PWM_PIN, 0);
        analogWrite(Right_PWM_PIN, 0);
        Serial.println("PWM values exceeded the limit");

        delay(5);
      }

    }

};



#endif
