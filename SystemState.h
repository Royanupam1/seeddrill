#ifndef SYSTEMSTATE_H
#define SYSTEMSTATE_H

#include <Arduino.h>
#include "Config.h"

// ==================== GLOBAL VARIABLES ====================
// These are defined in MotorControl.cpp
extern ControlState controlState;
extern bool emergencyStop;
extern bool calibrated;
extern float calib_a;
extern float calib_b;

// ==================== FUNCTION DECLARATIONS ====================

// Configuration functions
void loadAdvancedConfig();

// System state management
void updateSystemState(float speed, float rpm, bool gpsValid);
void checkSystemSafety();
void resetEmergencyStop();
bool isSystemReady();

// System diagnostics
void performSystemDiagnostic();

// Error handling
void logSystemError(const char* errorMsg);

// System reset functions
void softSystemReset();
void factoryReset();

// Health check functions
void checkSystemIntegrity();
void checkGPSValidity();
void checkMotorHealth();
void validateCalibrationHealth();
void triggerEmergencyStop(const char *reason);
void checkEmergencyConditions();
void performSystemHealthCheck();

// Stub functions for compatibility
void updateWebClients();
void serializeSystemData(void* response);

// Simple RPM reading function
float readActualRPM();

#endif // SYSTEMSTATE_H