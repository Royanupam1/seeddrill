#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Arduino.h>

// Forward declaration - don't include Config.h to avoid circular dependency
struct ControlState;

// ─────────────────────────────────────────────
// GPS Constants  
// ─────────────────────────────────────────────
const unsigned long GPS_DATA_MAX_AGE = 2000;

// ─────────────────────────────────────────────
// Kalman Filter Constants
// ─────────────────────────────────────────────
const float TRACTOR_KALMAN_Q_SPEED       = 0.04f;
const float TRACTOR_KALMAN_Q_ACCEL       = 0.02f;
const float TRACTOR_KALMAN_R_BASE        = 0.08f;
const float TRACTOR_IDLE_THRESHOLD       = 1.0f;
const float TRACTOR_CRAWL_THRESHOLD      = 2.5f;
const float TRACTOR_WORKING_THRESHOLD    = 8.0f;
const float TRACTOR_VIBRATION_FILTER     = 0.15f;
const float TRACTOR_IMPLEMENT_NOISE      = 0.1f;
const float TRACTOR_TURN_DETECTION       = 2.0f;

// ─────────────────────────────────────────────
// Kalman Filter State Structure
// ─────────────────────────────────────────────
struct KalmanState {
    float speed;                  // Filtered speed (m/s)
    float accel;                  // Filtered acceleration
    float P[2][2];                // Covariance matrix
    unsigned long lastUpdate;    // Timestamp of last update
    bool initialized;            // Initialization flag
    float speedBuffer[5];        // Rolling buffer for smoothing
    int bufferIndex;             // Index for buffer rotation
    float vibrationLevel;        // Vibration metric
    bool inTurn;                 // Turn detection flag
    float avgAccel;              // Smoothed acceleration
};

// ─────────────────────────────────────────────
// Function Prototypes
// ─────────────────────────────────────────────
bool isValidTractorGPSData(float speed_kmh, int satellites, float hdop,
                           bool hasValidFix, unsigned long dataAge);

void initTractorKalman(KalmanState &kalman);
void detectTractorConditions(KalmanState &kalman, float currentSpeed);
void tractorKalmanPredict(KalmanState &kalman, float dt);
void tractorKalmanUpdate(KalmanState &kalman, float measurement,
                         float R_variance, float speedKmh);

float calculateTractorMeasurementNoise(int satellites, float hdop,
                                       float speedKmh, unsigned long dataAge,
                                       bool inField);

void processTractorGPSMeasurement(KalmanState &kalman, ControlState &control,
                                  float rawSpeedKmh, int satellites, float hdop,
                                  bool hasValidFix, unsigned long dataAge,
                                  bool inField = true);

void applyTractorPostProcessing(KalmanState &kalman, ControlState &control);

#endif // KALMANFILTER_H