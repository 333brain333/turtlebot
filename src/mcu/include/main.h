#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <FastLED.h>

/**
 * Interrupt handlers for encoders
 */
void left_interrupt();
void right_interrupt();

/**
 * Setup and main loop
 */
void setup();
void loop();

/**
 * Command processing and parsing
 */
void processCommand(String command);
bool parseSetPWM(const String& command, int* leftA_PWM, int* leftB_PWM, int* rightA_PWM, int* rightB_PWM);
bool parseSetPose(const String& command, float* x, float* y, float* th);

/**
 * Motor control
 */
void setMotorsPWM(int leftA, int leftB, int rightA, int rightB);

/**
 * Odometry and speed calculation
 */
void updateOdometry();
void computeSpeed();
void computeWheelsPID();
bool parseSetCoeff(const String& command, float* Kp, float* Ki, float* Kd, float* Kff);
bool parseSetSpeed(const String& command, int* speedLeft, int* speedRight);
bool parseSetLed(const String& command, int* brightness, String* colorName);
bool parseColorName(const String& name, CRGB* color);
#endif // MAIN_H
