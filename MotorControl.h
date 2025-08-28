#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "Config.h"

// ==================== ENHANCED FUNCTION DECLARATIONS ====================

// Enhanced initialization functions
void initializeEnhancedMotorSystem();
void initializeMotorSystem(); // Compatibility wrapper

// Enhanced motor control functions
void setEnhancedMotorForward(int pwm);
void setMotorForward(int pwm); // Compatibility wrapper
void stopMotor();

// Advanced PWM/RPM calculations
int calculateAdvancedPWM(float targetRPM, bool useCalibration = true);
int advancedRPMtoPWMLookup(float rpm);
int rpmToPWMLookup(float rpm); // Compatibility wrapper
float calculateIntelligentTargetRPM(float speed_ms);
float calculateTargetRPM(float speed_ms); // Compatibility wrapper
int getCurrentPWM();

// Enhanced control and monitoring
void updateEnhancedMotorControl(ControlState &control, float targetRPM, bool speedValid);
void updateMotorControl(ControlState &control, float targetRPM, bool speedValid); // Compatibility wrapper
void updateMotorControlSystem(ControlState &control);
void monitorMotorHealth();

// Enhanced RPM reading and feedback
float readActualRPM();
float readMotorLoad();

// Environmental adaptation
void updateEnvironmentalFactors();

// Advanced calibration system
void performEnhancedCalibration();
void performMotorCalibration(); // Compatibility wrapper
bool validateEnhancedCalibration(float calib_a, float calib_b, float pwmVals[], float rpmVals[], int numPoints);

// Intelligent PID tuning
int advancedPIDControl(float targetRPM, float actualRPM);
void performIntelligentAutoTune();

// Advanced diagnostics and debugging
void performMotorDiagnostics();

// Crop-specific RPM calculation
float calculateCropSpecificRPM(float speed_ms, CropType crop);

// ==================== ENHANCED MOTOR PARAMETER STRUCTURE ====================
struct EnhancedMotorParams {
    // PID Controller Parameters
    float kp;
    float ki;
    float kd;
    float integral;
    float lastError;
    unsigned long lastUpdate;
    
    // Motor Physical Characteristics
    float motorEfficiency;
    float staticFriction;
    float dynamicFriction;
    float loadCompensation;
    float temperatureCoeff;
    float voltageNominal;
    float currentVoltage;
    
    // Advanced PWM Control Parameters
    int minEffectivePWM;
    int maxSafePWM;
    int startupBoostPWM;
    unsigned long startupBoostTime;
    float pwmRampRate;
    
    // Motor Curve Parameters
    float motorConstant;
    float armatureResistance;
    float backEMFConstant;
    float torqueConstant;
    
    // Environmental Factors
    float soilDensityFactor;
    float moistureFactor;
    float seedTypeFactor;
    float implementDrag;
    
    // Filtering Parameters
    float rpmFilterAlpha;
    float pwmFilterAlpha;
};

extern EnhancedMotorParams motorParams;

// ==================== EXTERNAL VARIABLES ====================
// These are defined in MotorControl.cpp
extern float calib_a; // Enhanced calibration slope
extern float calib_b; // Enhanced calibration intercept
extern bool calibrated; // Calibration status
extern ControlState controlState; // System control state
extern bool emergencyStop; // Emergency stop flag
extern bool testMode; // Test mode flag
extern float testSpeedKmh; // Test speed
extern bool testSpeedSet; // Test speed set flag

// ==================== ENHANCED CONSTANTS ====================
// Research-based motor control constants
#define ENHANCED_PWM_FREQUENCY 25000 // 25kHz for optimal efficiency
#define ENHANCED_MIN_EFFECTIVE_PWM 65 // Higher minimum for better torque
#define ENHANCED_MAX_SAFE_PWM 245 // Safe maximum with headroom
#define ENHANCED_STATIC_FRICTION 55 // Typical static friction PWM
#define ENHANCED_STARTUP_BOOST 120 // Startup boost PWM
#define ENHANCED_RAMP_RATE 15.0f // PWM ramping rate

// Environmental adaptation limits
#define MAX_SOIL_DENSITY_FACTOR 1.5f // Maximum soil resistance
#define MIN_SOIL_DENSITY_FACTOR 0.8f // Minimum soil resistance
#define MAX_TEMPERATURE_DERATE 0.15f // 15% max temperature derating
#define MAX_MOISTURE_COMPENSATION 0.3f // 30% moisture compensation

// PID tuning ranges (research-based)
#define PID_KP_MIN 0.5f
#define PID_KP_MAX 2.0f
#define PID_KI_MIN 0.1f
#define PID_KI_MAX 0.5f
#define PID_KD_MIN 0.02f
#define PID_KD_MAX 0.15f

// Calibration quality thresholds
#define MIN_CALIBRATION_R_SQUARED 0.90f // Minimum R² for good calibration
#define MAX_CALIBRATION_RMSE 15.0f // Maximum RMSE for good calibration
#define MIN_CALIBRATION_SLOPE 0.3f // Minimum reasonable slope
#define MAX_CALIBRATION_SLOPE 1.2f // Maximum reasonable slope
#define MIN_CALIBRATION_INTERCEPT 40.0f // Minimum intercept (static friction)
#define MAX_CALIBRATION_INTERCEPT 120.0f // Maximum reasonable intercept

// Performance monitoring thresholds
#define MAX_ACCEPTABLE_ERROR_PCT 15.0f // Maximum acceptable RPM error %
#define HIGH_ERROR_THRESHOLD 25.0f // Threshold for high error warnings
#define STALL_DETECTION_PWM 200 // PWM threshold for stall detection
#define STALL_DETECTION_RPM 100 // RPM threshold for stall detection

#endif // MOTORCONTROL_H
