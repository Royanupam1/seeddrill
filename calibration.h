#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "Config.h"

// Forward declarations for motor control functions
void setMotorForward(int pwm);
void stopMotor();
int getCurrentPWM();

// Main calibration functions
void loadCalibration(EEPROMData &eepromData, float &calib_a, float &calib_b, bool &calibrated);
void performMotorCalibration();

// Banner and display functions
void printStartupBanner(float calib_a, float calib_b, bool calibrated);
void printPinConfiguration();

// Validation function
bool validateCalibration(float calib_a, float calib_b, float pwmVals[], float rpmVals[], int numPoints);

// Internal calibration functions (split for better organization)
void calibrateMotor();
bool collectCalibrationData(float pwmVals[], float rpmVals[], int numPoints, int &validPoints);
bool collectSingleCalibrationPoint(int pointIndex, int totalPoints, float pwmVal, float &rpmVal);
bool computeCalibrationCoefficients(float pwmVals[], float rpmVals[], int numPoints, float &new_calib_a, float &new_calib_b);
void showCalibrationResults(float new_calib_a, float new_calib_b, float pwmVals[], float rpmVals[], int numPoints);
bool saveCalibrationToEEPROM(float new_calib_a, float new_calib_b);

#endif // CALIBRATION_H